# ADCS 集成路线图 — pocat-lektron-sw

> 状态：草案 · 2026-10-14
> 目标：把 `src/subsystems/adcs/` 中已在主机仿真里验证过的 ADCS 算法完整接入 FreeRTOS 飞控软件，并在 FlatSat / 亥姆霍兹线圈上完成验收。

已验证的算法（主机仿真，`test/`）：

- ω×B 消旋（自适应 ΔT，Monte Carlo 50/50 通过）
- TRIAD + 陀螺零偏估计 + 磁强计互补滤波
- 2-DOF 偏航不变 nadir 控制（48 h：均值 3.6°，峰值 18.3°）
- Σ-Δ 磁力矩器量化

任务约束（来自 `../ref/cat-structure-subsystems-dmc.md` 与 AOCS 专家会议 PPT）：

- 发射 2027 Q3，450–550 km 极地/近极地轨道
- 载荷 APE < 20°（相对 X、Y 轴，95% 时间）
- 模式许可：INIT 允许消旋；Nadir 仅 NOMINAL；SUNSAFE/SURVIVAL 下 AOCS 只采 HK

---

## 1. 现状关键发现

这些事实决定了路线图的形状，按影响阶段标注。

| # | 发现 | 位置 | 阶段 |
|---|------|------|------|
| 1 | `obc.c` 残留冲突标记，来自 2026-06-01 合并 `9385c52`。上游侧 `fcf80c6` 是干净版本，与 `task_management.c` 的建/重置任务逻辑一致 | `src/subsystems/obc/obc.c:72-87, 123-271` | S1 |
| 2 | `adcs.c` 自定义 `bit0=DETUMBLE, bit1=NADIR`，与 `notifications.h` 的 `bit0=IDLE, bit1=DETUMBLING, bit2=NADIR` 冲突；`adcs_task` 只是空壳 | `src/subsystems/adcs/adcs.c:25-26`<br>`include/utils/notifications.h:25-29` | S1 / S4 |
| 3 | 开发机上没有 `arm-none-eabi-gcc`、`st-flash` / `openocd`；固件尚未在本机编译过 | — | S1 |
| 4 | **当前 ADCS 板 `PQ_ADCS_HBridge` 使用 4×DRV8214 H 桥**，固件与仿真仍按旧板 BD2606MVV 建模；头文件写 0.5–32 mA，`adcs_config.h` 写 150 mA | `../pocat-hw/README.md`<br>`include/subsystems/adcs/adcs_magnetorquer.h:6`<br>`include/subsystems/adcs/adcs_config.h:78-80` | S2（回灌算法） |
| 5 | ADCS 所有 I²C 器件（IIM-42652、MMC5983MA@0x30、4×DRV8214）挂在 **SDA3/SCL3**；CSS = TMUX1108（`SEL_PH0..2`）+ 单路 `ADC_PH`；板供电由 `PoL_ADCS_POWER_EN` 控制 | ADCS / OBC 原理图 global labels | S2 |
| 6 | `HAL_I2C_MODULE_ENABLED` 被注释；`.ioc`（2026-09-30）中没有 I2C3，落后于 2026-10-09 的 OBC 原理图；`periph_reconfigure_for_freq()` 不处理 I²C；ADC1 被 `temperature.c` 无锁使用 | `include/core/stm32l4xx_hal_conf.h:43`<br>`src/utils/periph.c:52-76`<br>`src/utils/temperature.c` | S2 |
| 7 | MCU 只有单精度 FPU（`fpv4-sp-d16`），ADCS 全部用 `double` → 软浮点 | `CMakeLists.txt:16-20` | S3 / S4 |
| 8 | 参考 IGRF 为 IGRF-13（有效期到 2025），发射在 2027 Q3 → 需 IGRF-14；`igrf13()` 把 29 KB 的 `double gh[3645]` 放在**局部变量**（ADCS 栈仅 4 KB）；`igrf13.h` 声明 `igrf13_ngdc`，实现叫 `igrf12_ngdc`；`sgp4.c` 使用全局可变量 + `printf` | `reference/ADCS/igrf13syn.c:53-55`<br>`reference/ADCS/igrf13.c:339`<br>`reference/ADCS/sgp4.c:6` | S3 |
| 9 | RTC 失去备份后复位到 2000-01-01，系统中没有“时间有效”概念 | `src/utils/periph.c:298-336` | S3 / S4 |
| 10 | 代码无 INIT 态；SUNSAFE/SURVIVAL 直接暂停 ADCS（文档要求只采 HK）；频率表（文档 CONTINGENCY = 26 MHz）与任务优先级表也与代码不一致 | `src/subsystems/obc/state_machine.c:54-88` vs 文档 §Satellite Operational Modes | S4 / S5 |
| 11 | 没有“设置 ADCS 模式”的 TC；`TC_UPLOAD_ADCS_TLE/CAL` 只有注释；Flash 中 TLE 地址重复定义（`TLE_ADDR` vs `TLE_ADDR1/2`）；HT 存储（`ht_handling.c`）与 EPS 电量读取均为空桩 | `src/subsystems/comms/tc_handler.c:71-117`<br>`include/utils/flash.h:57-65, 97-98` | S5 |
| 12 | 健康检查周期 5 s（`obc.c` 中 `health_config`），暂停 ACK 超时 10 s；`tm_check_pause()` 收到 PAUSE 立即 ACK → ADCS 必须在 ACK 前关闭线圈 | `src/subsystems/obc/task_management.c:134-171` | S4 |
| 13 | APE 要求 < 20°；当前 48 h 仿真峰值 18.3°，余量薄，且基于旧 MTQ 量化模型 | `docs/adcs_nadir_yaw_invariance_analysis.md` | S2 / S6 |

---

## 2. 相对原始 6 阶段方案的调整

1. **S2 与 S3 并行。** S3（轨道/环境模型）是纯软件，可在主机上验证，不必等驱动完成。S4 的任务骨架也可以先用 stub HAL 开发。
2. **S2 的结果必须回灌算法。** 用 DRV8214 的实际电流分辨率和范围替换 `mtq_compute_command()` 的量化参数与 `test/sim/sim_mtq_driver.c`，重跑 `test_nadir` / `test_monte_carlo`，确认 APE 仍满足。
3. **处理器在环（PIL）提前到 S4。** 在 Nucleo 上跑真实 ADCS 任务（真实软浮点 + 真实调度），主机仿真经 UART 注入传感器读数、回收 MTQ 指令。S6 只保留真正的物理测试。
4. **每个阶段都有可量化的出口准则**，并贯穿资源预算（Flash / RAM / 栈水位 / CPU 周期）。
5. **显式列出需要与 OBC / COMMS / OBDH / EPS 负责人对齐的接口决策**（INIT 态、SUNSAFE 语义、新 TC ID、beacon 字段）。

### 依赖关系

```
S1 ─┬─> S2  驱动（需要硬件）──────┐
    ├─> S3  轨道/环境（主机）────┼─> S4b 真实 HAL 集成 + PIL ─> S5 ─> S6
    └─> S4a 任务骨架（stub HAL）─┘
```

---

## 3. 各阶段

### S1 编译基线与清理

**任务**

- 安装工具链：ARM GNU Toolchain（arm-none-eabi-gcc + newlib-nano），以及 `stlink`（`flash.sh` 依赖 `st-flash`）或 openocd。
- 先运行 `scripts/sync-upstream.sh` 同步上游，再在 `dev` 上以上游 `fcf80c6` 的版本重新解决 `obc.c` 冲突（任务的创建与复位已由 `task_management.c` 负责）。
- `adcs.c`：删除本地 `ADCS_*_MODE` 宏，改用 `N_ADCS_DESIRED_STATE_IDLE/DETUMBLING/NADIR`；为 `N_ADCS_NEW_CALIBRATION` / `N_ADCS_NEW_TLE` 预留空的处理分支。
- 注释修正：`adcs_nadir.h` 写的是 “+Y body axis”，实际为 +Z；`adcs_magnetorquer.h` 中的 32 mA 与 `adcs_config.h` 中的 150 mA 标注“待 S2 确认”。
- 主机回归：`test/CMakeLists.txt` 加 `enable_testing()` + `add_test()`（各测试已返回 0/1），`ctest` 作为每次合并的门禁。

**出口准则**

- [ ] `./build.sh` 零错误生成 `.elf`，记录 `--print-memory-usage` 基线
- [ ] Nucleo-L476RG 上电打印 banner，所有任务创建成功，连续运行 10 min 无 IWDG 复位
- [ ] `ctest` 全部通过

### S2 BSP 与驱动

目标硬件：**`PQ_ADCS_HBridge`（DRV8214）板**（已确认）。

**任务**

- **冻结引脚表。** 从 OBC 原理图网表确认 `SCL3/SDA3`（推测 PC0/PC1，AF4，需核实）、`SEL_PH0..2`、`ADC_PH` 对应的 ADC 通道、`PoL_ADCS_POWER_EN`；同步更新 `.ioc` 与 `src/utils/periph.c`。
- **I²C3。** 启用 HAL I2C；在 `periph_reconfigure_for_freq()` 中加入 I²C TIMINGR（80 / 8 / 2 MHz）；实现超时、错误计数、总线锁死恢复（9 个 SCL 脉冲），必要时经 PoL 给 ADCS 板断电重启。
- **驱动**（每个器件一个文件，放在 `src/subsystems/adcs/drivers/`）：
  - **MMC5983MA**：SET/RESET 消磁、单次测量触发、温度读取、传感器轴 → 本体系旋转。
  - **IIM-42652**：量程 ≥ ±250 dps（覆盖 90°/s 设计工况）、ODR、LPF、温度读取。
  - **CSS**：TMUX1108 通道切换 + LPV542 建立时间延迟 + ADC 采样；ADC1 加互斥（与 `temperature.c` 共用）。
  - **DRV8214 ×4**：确认线圈↔驱动器映射（4 个驱动器对应 3 轴）、I²C 地址、控制方式（I²C 桥控 vs PWM 引脚）、电流调节分辨率与上限。
- **HAL 抽象。** 沿用 `mtq_apply_command()` 的 weak 符号模式，定义 `adcs_hal.h`（读 mag / gyro / CSS、写 MTQ、电源开关）。目标板链接真实实现，`test/` 链接仿真实现。
- **校准数据结构。** 定义 `adcs_calib_t`（磁强计硬/软铁矩阵 + 偏置、陀螺零偏/温度多项式、光电二极管偏置），作为 S5 上传的目标格式。
- **回灌算法。** 用 DRV8214 的实际分辨率更新 `MTQ_*_INTENSITY_*` 与 `sim_mtq_driver.c`，重跑 nadir 与 Monte Carlo。

**出口准则**

- [ ] 裸机/单任务测试固件稳定采集 ≥ 1 h，I²C 错误率有记录
- [ ] 轴向与极性验证表：转动板子核对符号；单轴通电，用外部磁强计/罗盘核对极性
- [ ] 新量化模型下 `test_nadir` 仍满足 APE < 20°（95% 时间）

### S3 轨道与环境模型

主机先行，可与 S2 并行。

**任务**

- **SGP4（Vallado 版）。** 去掉全局可变量与 `printf`，`satrec` 静态分配；用 Vallado `SGP4-VER.TLE` 测试向量做主机单测。保持 `double`（时间 × 平均运动需要精度）。
- **IGRF-14**（2025 epoch + SV 至 2030）。系数以 `static const float` 存入 Flash。阶数通过仿真权衡（n = 1 倾斜偶极 / 8 / 13：方向误差 vs CPU 周期 vs TRIAD 误差）确定，不预先定死“简化模型”。
- **太阳模型。** 低精度星历（约 0.01°）+ 圆柱阴影模型，用于与 CSS 的 `sun_visible` 交叉校验。
- **坐标系统一。** 位置（SGP4 输出 TEME）、B 场、太阳方向放在同一惯性系；用 GMST 做 ECEF ↔ ECI 转换。TEME 与 J2000 之间约 0.3° 的差异记为已知误差。
- **时间。** `time_get_unix()` → JD(UTC)；引入 `time_valid` 标志（RTC 复位或从未授时则无效），以及 TLE 老化上限（例如 14 天）。
- **仿真中分离“真值模型”与“星上模型”。** `test/sim/sim_environment.c` 的真值使用高阶 IGRF + 真实太阳运动，星上算法使用截断模型，以暴露模型误差。

**出口准则**

- [ ] 主机黄金向量比对通过（SGP4 位置与 Vallado 结果相差 < 1 m；IGRF 与官方实现的偏差符合所选阶数的预期）
- [ ] 在目标板上用 DWT 周期计数测出每次参考矢量更新的 CPU 时间、栈水位与 Flash 增量，记入预算表

### S4 ADCS 任务闭环

**S4a — 任务骨架**（可与 S2/S3 并行，使用 stub HAL）

- `adcs_state_t` 静态分配。把“一个控制周期”抽成纯函数 `adcs_cycle(state, hal)`，RTOS 任务只做调度外壳，便于主机测试。
- **周期调度。** 按绝对截止时间计算 `xTaskNotifyWait` 的超时；`health_kick` 与控制周期解耦（每 ≤ 1 s 一次），以支持消旋 1–2 s 和未来 nadir 10 s 的自适应周期。
- **周期时序（分时死区）。** MTQ 关 → 等待线圈电流衰减 → 读磁强计（含 SET/RESET）/ 陀螺 / CSS → 更新参考矢量 → `adcs_mode_step()` → 施加 MTQ。实测死区并更新 `DETUMBLE_DEAD_TIME_S`。
- **暂停/复位协议。** 收到 `N_TASK_PAUSE` 时先关线圈，再交给 `tm_check_pause()` 发 ACK；健康复位后的 setup 也先关线圈；恢复时重置计时与滤波器。
- **模式管理。** 区分“期望模式”与“当前模式”，由任务层负责排序（例：期望 NADIR 但 |ω| 过大 → 先 DETUMBLING → IDLE → NADIR）。加入 FDIR 转换：nadir 中 |ω| 超限 → 消旋；`time_valid` / TLE 失效或传感器无效 → IDLE。`nadir_step` 检查各传感器的 `valid` 标志。

**S4b — 真实 HAL 集成**（需要 S2 + S3）

- 接入真实 HAL。
- 评估 ADCS 任务优先级（代码中为 2，文档表中为 4），以及 Flash 擦写造成的抖动（用 GPIO / DWT 实测）。

**PIL**

- Nucleo 运行完整固件；主机仿真通过 UART 注入传感器读数、回收 MTQ 指令，复现 `test_nadir` / `test_detumble` 的场景。

**出口准则**

- [ ] PIL 下的消旋与 48 h nadir 场景结果与纯主机仿真一致（统计误差在容差内）
- [ ] 控制周期抖动、CPU 占用（80 MHz 及最低运行频率）、栈水位有记录
- [ ] 暂停、恢复、健康复位后都不会留下通电的线圈

### S5 TC/TM 与星务联动

**任务**

- **TC。** 实现 `TC_UPLOAD_ADCS_TLE`（多包传输 + 行校验和 + 试传播通过后再提交；A/B 双份保留旧 TLE）和 `TC_UPLOAD_ADCS_CAL`；**新增** ADCS 模式/参数 TC（ID 需与地面站软件同步）。
- **Flash。** 合并为单一 ADCS 配置块（版本号 + CRC），清理重复定义的 TLE 地址；优先放在 bank 2，避免擦写时停顿 bank 1 的取指。
- **TM。** 提供 `adcs_get_telemetry()` 快照接口。beacon 目前只用了 48 字节中的 12 字节，为 ADCS 定义字段：模式、|ω|、估计指向误差、太阳可见、传感器健康、TLE 年龄、`time_valid`。历史遥测依赖 OBDH 的 `ht_handling` 实现。
- **星务联动。** 与 OBC 负责人对齐以下问题：
  - INIT/LEOP 态（30 min 静默后允许自动消旋）；
  - SUNSAFE/SURVIVAL 下 ADCS 是“只采 HK”还是暂停；
  - OBC 换态时如何通知 ADCS 当前允许的模式；
  - PoL 断电时传感器标为无效，但任务不能挂死。

  低电量判定本身归 EPS/OBC。ADCS 侧先用 `TC_TRANSIT_TO_*` 驱动换态来测试。

**出口准则**

- [ ] 经 RF（或脐带缆）完成 TLE/校准上传，重启后仍然生效
- [ ] 在所有 OBC 态切换下，ADCS 行为符合模式许可表
- [ ] beacon 中的 ADCS 字段能被地面解码

### S6 物理 / HIL 验收

- **亥姆霍兹线圈**
  - 整星上电状态下的磁强计校准（含其他子系统的剩磁）
  - 逐轴测量 MTQ 磁矩与极性
  - 线圈场旋转下的 ω×B 闭环
- **悬线或气浮台**：消旋演示（PocketQube 的尺寸适合悬线）
- **太阳模拟器**：CSS 标定，验证 TRIAD 的太阳模式
- **FlatSat/EGSE 长时浸泡**：≥ 48 h，全部任务运行，覆盖换态、Flash 写入、看门狗、RF 上传 TLE

**验收**

- [ ] APE < 20°（95% 时间），由 PIL/SIL 证明（地面无法物理复现 nadir）
- [ ] 消旋收敛时间：从 30°/s（文档工况）与 90°/s（设计工况）出发
- [ ] MTQ 占空比纳入功耗预算

---

## 4. 贯穿各阶段的工作

| 项目 | 内容 |
|------|------|
| 资源预算表 | 每个阶段更新：Flash、RAM、各任务栈水位（开启 `configCHECK_FOR_STACK_OVERFLOW=2`）、ADCS 单周期 CPU 时间（80 / 26 / 8 / 2 MHz） |
| 回归门禁 | `ctest`（主机算法）+ 固件编译 + 阶段性 PIL 场景 |
| ICD 文档 | 通知位、TC/TM 格式、Flash 布局、HAL 接口，放在 `docs/` |
| 分支 | 每个阶段一个 feature 分支 → `dev` → 向上游 `nanosatlab` 提 PR（见 `docs/branch-management.md`）；改动 OBC/COMMS/OBDH/EPS 文件前先与对应负责人对齐 |

## 5. 主要风险

| # | 风险 | 缓解 |
|---|------|------|
| 1 | MTQ 硬件模型不一致（DRV8214 vs BD2606MVV），现有 3.6° / 18° 结论可能变化 | S2 开始时先在仿真中建立 DRV8214 量化模型 |
| 2 | IGRF-13 已过期，发射在 2027 Q3 | S3 直接使用 IGRF-14 |
| 3 | 双精度软浮点在低主频状态下的 CPU 开销 | S3/S4 用 DWT 实测；只在测量证明必要时才把 IGRF/控制改为 `float` |
| 4 | RTC 失效时用错误时间算参考矢量，导致错误指向 | `time_valid` 门控 nadir；授时 TC |
| 5 | 6 个器件共享一条 I²C3，总线锁死会让 ADCS 全部失明 | 超时 + 9 脉冲恢复 + PoL 断电重启 |
| 6 | 星体剩磁和线圈对磁强计的污染 | 分时死区 + 整星亥姆霍兹校准 |
| 7 | 依赖他人未完成的模块（EPS 电量、OBDH 历史遥测、TC ID 与地面站同步） | 尽早定义接口；ADCS 侧先用 TC 驱动测试 |

## 6. 待决策事项

| 决策 | 涉及方 | 最晚阶段 |
|------|--------|---------|
| 是否在代码中加入 INIT 态，以及 LEOP 自动消旋的触发条件 | OBC | S4a |
| SUNSAFE/SURVIVAL 下 ADCS：只采 HK 还是暂停 | OBC | S4a |
| ADCS 任务优先级与各态主频（CONTINGENCY 26 MHz？） | OBC | S4b |
| IGRF 阶数 | ADCS（仿真结果） | S3 |
| 新 ADCS TC 的 ID 与 beacon 字段布局 | COMMS + 地面站 | S5 |
| 重启后是否恢复上一次的 ADCS 模式 | OBC + ADCS | S5 |
