# ADCS 任务设计（S4）— pocat-lektron-sw

> 状态：S4a 完成，S4b 已接线 · 2026-10-22（代码已具备并能编译；实测、PIL 待硬件）
> 关联：`docs/adcs_integration_roadmap.md` §3 S4 · `docs/adcs_hw_interface.md`（HAL / 驱动） · `docs/adcs_orbit_env_models.md`（参考矢量）

本文说明 ADCS FreeRTOS 任务如何调度控制周期、如何在各模式之间切换、如何配合 OBC 的暂停/复位协议，以及 ADCS 与 OBC 之间的接口。

---

## 1. 结构

```
adcs.c          FreeRTOS 外壳：截止时间、通知、暂停/恢复、踢狗、从 Flash 载入 TLE、统计
  │  adcs_ctrl_cycle(&ctrl, &board_io, time_get_unix(), dt)
  ▼
adcs_ctrl.c     一个控制周期（不依赖 FreeRTOS/HAL，可在主机编译）：
                读传感器 → CSS 太阳矢量 → 参考矢量 → 模式监督 → adcs_mode_step() → MTQ 输出
  │  adcs_io_t（函数指针）
  ▼
adcs_hal_*      板级实现（adcs_hal_board.c）；主机以后可以绑定 test/sim
```

- `adcs_ctrl_t`（算法状态、TLE/SGP4 记录、最近一次星历、期望模式、计数器）是 `adcs.c` 中的静态变量，不占任务栈。
- 板级绑定：`board_io = { adcs_hal_read_mag, adcs_hal_read_gyro, adcs_hal_read_css, adcs_hal_mtq_enable, mtq_apply_command }`。
- 主机构建会把 `adcs_ctrl.c` 加入 `ADCS_SOURCES`，以此检查它不依赖 FreeRTOS/HAL。目前还没有测试调用它（按 2026-10-22 的决定，新测试推迟）。

## 2. 一个控制周期（`adcs_ctrl_cycle`）

| 步骤 | 内容 | 说明 |
|------|------|------|
| 1 | 线圈已通电时，先 `mtq_apply(零指令)` | 分时死区开始：coast，不产生力矩 |
| 2 | 读陀螺 → CSS → 磁强计 | 磁强计最后读，前两项读数的时间（约 1.5 ms）用作线圈电流衰减时间，不再额外延时 |
| 3 | `determination_css_sun()` | 6 路电压加权面法向求 `sun_body`；最亮面 ≥ `ADCS_CSS_SUN_THRESHOLD_V`（0.1 V，TBC S6）时 `sun_visible` = 1 |
| 4 | `adcs_ephem_update()` + `adcs_ephem_to_state()` | 只在当前模式为 IDLE 或 NADIR 时计算；消旋不需要参考矢量，因此跳过，在低主频下省去约 0.2 M 周期。星历有效且处于阴影中时，强制 `sun_visible` = 0（排除地球反照） |
| 5 | 模式监督（§3） | |
| 6 | `state.dt = dt`，`adcs_mode_step()` | `dt` 是两次采样之间实测的 tick 差 |
| 7 | 输出 | DETUMBLING/NADIR：需要时先 `mtq_enable(true)`，再 `mtq_apply()`；其他模式：`mtq_enable(false)`，PoL 拉低，DRV8214 进入休眠（输出 Hi-Z） |
| 8 | 返回下一周期 | DETUMBLING：`detumble_select_dt()`（1–2 s）；NADIR：`ADCS_CONTROL_DT`（1 s）；IDLE：`ADCS_IDLE_PERIOD_S`（10 s），但如果刚刚消旋收敛、还要进入 NADIR，则为 1 s |

分时死区的估算值 `DETUMBLE_DEAD_TIME_S` 从 3.5 ms 改为 25 ms（磁强计 SET/RESET 一对约 18 ms，加上温度、陀螺、CSS 和关线圈的 I²C 写），待 S4b 实测。

## 3. 模式监督

OBC 设置**期望模式**，`state.mode` 是**当前模式**。所有转换都经过 IDLE，所以 `adcs_mode_request()` 的校验不用改。

| 期望 | 条件 | 当前模式变为 |
|------|------|-------------|
| 任意 | 陀螺或磁强计连续 `ADCS_SENSOR_FAIL_LIMIT`（3）个周期无效 | IDLE（置 `SENSOR_FAULT`）；期望模式保留，读数恢复后重新进入 |
| IDLE | — | IDLE |
| DETUMBLING | 当前不是 DETUMBLING，且本周期读数有效 | DETUMBLING |
| DETUMBLING | 收敛（\|ω\| < 1 °/s 持续 30 个周期） | IDLE，期望模式也改为 IDLE |
| NADIR | 在 IDLE 中，\|ω\| > `ADCS_NADIR_ENTRY_OMEGA`（2 °/s） | DETUMBLING（不需要星历） |
| NADIR | 在 IDLE 中，\|ω\| ≤ 2 °/s，星历有效且 TLE 未过期 | NADIR |
| NADIR | 在 DETUMBLING 中收敛 | IDLE，下一周期（1 s 后）进入 NADIR |
| NADIR | 在 NADIR 中，星历无效（`ADCS_EPH_INVALID_MASK`）或 TLE 过期（`ADCS_EPH_TLE_STALE`） | IDLE |
| NADIR | 在 NADIR 中，陀螺 \|ω\| > `ADCS_FDIR_NADIR_OMEGA`（5 °/s）连续 `ADCS_FDIR_OMEGA_COUNT`（10）个周期 | DETUMBLING（置 `FDIR_OMEGA`，`fdir_count`++） |

- FDIR 用的是原始陀螺的 \|ω\|，而不是减去偏置估计后的值：偏置估计只有约 0.01 °/s，但如果参考矢量出错，偏置估计可能把真实的转动吸收掉。
- 进入门限 2 °/s 是消旋退出门限的 2 倍，避免刚消旋完又被打回去。
- 所有门限都标注 TBC，在 `adcs_config.h` 的 “Task Supervision / FDIR” 一节中。

**算法层的保护**（`adcs_nadir.c` / `adcs_detumble.c`）：

- 进入 nadir 后，第一次日照 TRIAD 之前不输出（`attitude_valid`，发现 #20）。在阴影中进入时，卫星先自由漂移，日出后才开始控制。
- 陀螺或磁强计无效的那一步输出零磁矩。nadir 中陀螺无效时会丢弃姿态估计（这段时间无法传播），等下一次 TRIAD。
- `determination_update()` 只剩日照 TRIAD；原来走不到的阴影分支已删除（发现 #19）。

`adcs_ctrl_t.flags` 记录最近一个周期的状态位（`ADCS_CTRL_MAG_FAIL` … `ADCS_CTRL_MTQ_FAIL`，见 `adcs_ctrl.h`），留给 S5 的遥测使用。

## 4. 调度（`adcs.c`）

- **绝对截止时间。** 每个周期结束后 `next_wake += period`。如果一个周期结束时已经过了下一个截止时间，就记一次 overrun，并从当前时刻重新开始排程。
- **踢狗与控制周期解耦。** `wait_for_notification()` 的超时取“距离截止时间”和 `ADCS_KICK_PERIOD_MS`（1 s）中较小的一个，每次循环都 `health_kick()`。IDLE 的 10 s 周期不会触发 5 s 的健康检查。
- **期望模式通知**到达时立即运行一个周期（`next_wake = now`），不用等 IDLE 周期结束。多个位同时置位时按 IDLE > DETUMBLING > NADIR 取一个（安全优先）。
- **统计**（`adcs_get_task_stats()`）：周期数、overrun 次数、最长周期（从采样到 MTQ 输出，单位 tick）、栈水位（`uxTaskGetStackHighWaterMark`，单位 word）。供 S4b 实测和 S5 遥测使用。
- **时间。** `time_get_unix()` 只有 1 s 分辨率；500 km 轨道上 1 s 对应约 7.6 km，nadir 方向误差 < 0.1°，因此不做处理。时间是否有效由 OBC/OBDH 负责（发现 #9）。

## 5. 暂停、恢复与复位

| 事件 | ADCS 的动作 |
|------|-------------|
| 收到 `N_TASK_PAUSE` | **先** `adcs_ctrl_quiesce()`（PoL 拉低，线圈休眠），**再**调用 `tm_check_pause()`，后者立即 ACK |
| 暂停期间 | 只等待通知；不读传感器，也不计算（与现在 OBC 的 SSM/SM 行为一致） |
| 收到 `N_TASK_RESUME` | `adcs_ctrl_restart()`：当前模式切到 IDLE，清零计数；采样时刻和截止时间重置。期望模式保留，之后按 §3 重新进入（nadir 会重新初始化滤波器，并等待第一次 TRIAD） |
| 健康检查复位（`tm_reset_task`） | 任务被删除后重新创建，`setup_adcs()` 的**第一步**就是 `adcs_hal_mtq_enable(false)`；然后重新初始化传感器和控制器，并从 Flash 重新载入 TLE。期望模式回到 IDLE，OBC 需要时要重新发送 |
| 暂停期间收到的其他通知 | 由 `tm_check_pause()` 延后，恢复后处理 |

已知限制（新增发现 #24、#25）：

- `tm_reset_task()` 可能在任务持有 I²C3 互斥量时把它删除。现在 `i2c_bus` 等待互斥量最多 100 ms，超时就返回错误（`i2c_bus_lock_timeout_count()`），不会永久阻塞；但被删除任务持有的互斥量本身仍然无法释放，需要与 OBC 的任务管理一起解决。
- `OBDH_Read_Request()` 等待 OBDH 的应答时没有超时。如果 OBDH 任务没有运行，ADCS 在载入 TLE 时会一直阻塞，最终因为没有踢狗而被健康检查复位。

## 6. TLE 来源

- 上电（以及健康复位）时，以及收到 `N_ADCS_NEW_TLE` 时，从 `TLE_ADDR` 读取 138 B，按 **2 × 69 字符、无换行** 切成两行（与 `flash.h` 中“138 bytes”的注释一致），交给 `adcs_orbit_load_tle()`。这个函数会做行校验和、SGP4 初始化和历元试传播，全部通过后才替换当前 TLE；失败时保留旧 TLE，并在 UART4 打印 `ADCS: TLE rejected (<code>)`。
- 没有 TLE 时（Flash 为空，例如首次上电），星历状态为 `ADCS_EPH_NO_TLE`，不会进入 nadir；消旋不受影响。
- 记录格式、A/B 双份，以及清理重复定义的 `TLE_ADDR`/`TLE_ADDR1/2`，都留到 S5 与 TC 上传一起确定。

## 7. 与 OBC 的接口（默认行为）

按 2026-10-22 的决定，ADCS 侧**不修改 OBC 文件**：

- 上电后期望模式为 **IDLE**：每 10 s 采一次传感器和参考矢量，线圈休眠。
- ADCS 只服从 `N_ADCS_DESIRED_STATE_IDLE / _DETUMBLING / _NADIR`（`notifications.h`）。**目前代码中没有任何地方发送这些通知**：由谁、在什么时候发送（LEOP 自动消旋、NM 才允许 nadir、CM 只允许消旋），由 OBC 负责人决定（路线图 §6 待决策事项）。
- OBC 换态时暂停/恢复 ADCS 的方式不变；ADCS 保证暂停后线圈不通电。“SUNSAFE/SURVIVAL 下只采 HK”需要 OBC 改为发送 IDLE 而不是暂停，这也由 OBC 决定。

## 8. 资源（静态，2026-10-22）

| 项目 | 值 |
|------|-----|
| FLASH | 130 796 B（12.47%），比 S2a 的 dev `80f1db2` 增加 50 540 B：ADCS 目标文件 +35.0 KB，libm +12.5 KB，libgcc +0.5 KB，其他 +2.6 KB（HAL I²C 等）。算法、星历和 libm 都是第一次被链接进固件 |
| RAM | 61 960 B（63.03%），+1 392 B（主要是静态的 `adcs_ctrl_t`，约 1.0 KB） |
| 栈，最坏路径 | 载入 TLE 2 084 B；控制周期（nadir → TRIAD）2 000 B；读磁强计（间接调用）约 560 B；均低于 4 096 B。加上任务上下文约 200 B，约占 56% |

详见 `docs/adcs_resource_budget.md` S4 节。

## 9. 待硬件（S4b / PIL）

- [ ] DWT 实测单周期 CPU 时间与死区（更新 `DETUMBLE_DEAD_TIME_S`），在 80 MHz 和最低运行频率下
- [ ] 控制周期抖动（GPIO 翻转或 `adcs_get_task_stats()`），包括 Flash 擦写时的情况
- [ ] 栈水位实测（`stats.stack_hwm_words`）
- [ ] 暂停、恢复、健康复位后用示波器确认 PoL 为低、线圈无电流
- [ ] ADCS 任务优先级评估（代码中为 2，文档表中为 4）
- [ ] PIL：Nucleo 运行完整固件，主机仿真经 UART 注入传感器读数，复现 `test_detumble` / `test_nadir`
- [ ] CSS 可见门限（太阳模拟器，S6）
