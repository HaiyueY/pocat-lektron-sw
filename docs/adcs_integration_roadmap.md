# ADCS 集成路线图 — pocat-lektron-sw

> 状态：草案 · 2026-10-22 · S1 已完成（上板一项待硬件）· S3 已完成（DWT 周期实测待硬件）· S2a 已完成（S2b 上板验证待硬件）· S4a 已完成、S4b 已接线（实测与 PIL 待硬件）
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
| 1 | ~~`obc.c` 残留冲突标记，来自 2026-06-01 合并 `9385c52`~~ **S1 已解决**：合并上游 `15a4761`，`obc.c` 逐字取上游版本（ADCS 的建/复位/健康登记由 `task_management.c` 负责） | `src/subsystems/obc/obc.c` | S1 |
| 2 | ~~`adcs.c` 自定义 `bit0=DETUMBLE, bit1=NADIR`，与 `notifications.h` 的 `bit0=IDLE, bit1=DETUMBLING, bit2=NADIR` 冲突（**S1 已改用 `N_ADCS_*`**）；`adcs_task` 只是空壳~~ **S4 已解决**：任务运行 `adcs_ctrl_cycle()`，见 `docs/adcs_task_design.md` | `src/subsystems/adcs/adcs.c:25-26`<br>`include/utils/notifications.h:25-29` | S1 / S4 |
| 3 | ~~开发机上没有 `arm-none-eabi-gcc`~~ **S1 已解决**：Arm GNU Toolchain 14.3.Rel1 装在 `~/opt/`，固件已在本机编译通过；`st-flash` 仍未装（等硬件） | — | S1 |
| 4 | ~~当前 ADCS 板使用 4×DRV8214，固件与仿真仍按 BD2606MVV 建模~~ **S2 已解决**：实际是 **3 片** DRV8214（每轴一片），每轴两只线圈串联，3.3 V 电压调节开环驱动。线圈 R 和 N·A 由 PCB 走线算出：X/Z 43.3 Ω、Y 34.3 Ω，最大电流 71/90 mA，**最大磁矩约为旧值 150 mA 时的一半**；Y 轴 coil factor 从 0.056 改为 0.080。量化与仿真均已更新 | `docs/adcs_hw_interface.md` §4 | S2 |
| 5 | ADCS 的 I²C 器件挂在 SDA3/SCL3（PC0/PC1）；CSS = TMUX1108（PC2/PA11/PA12）+ `ADC_PH`（PC5 = ADC12_IN14）。**`PoL_ADCS_POWER_EN`（PA7）只接 DRV8214 的 nSLEEP，传感器常电**。**X 轴 DRV8214 的地址 0x30 与 MMC5983MA 冲突（硬件 bug，需改板）**。没有中断/故障线，全部轮询。详见 ICD §1–§3 | `docs/adcs_hw_interface.md` | S2 / 硬件 |
| 6 | ~~`HAL_I2C_MODULE_ENABLED` 被注释；`periph_reconfigure_for_freq()` 不处理 I²C；ADC1 被 `temperature.c` 无锁使用~~ **S2 已解决**：启用 I2C3（按频率设置 TIMINGR）；CSS 改用专用的 ADC2，不再与 ADC1 共用。`.ioc` 仍落后于原理图（交硬件负责人，ICD H5） | `src/utils/periph.c`<br>`src/utils/i2c_bus.c` | S2 |
| 7 | MCU 只有单精度 FPU（`fpv4-sp-d16`），ADCS 全部用 `double` → 软浮点。S3 估算参考矢量更新约 0.2 M 周期/次（2 MHz 下约 100 ms），IGRF 递推中的开方已改为查表；实测待 `ADCS_BENCH` | `CMakeLists.txt:16-20`<br>`docs/adcs_resource_budget.md` S3 节 | S3 / S4 |
| 8 | ~~参考 IGRF 为 IGRF-13（有效期到 2025），发射在 2027 Q3 → 需 IGRF-14；`igrf13()` 把 29 KB 的 `double gh[3645]` 放在**局部变量**（ADCS 栈仅 4 KB）；`igrf13.h` 声明 `igrf13_ngdc`，实现叫 `igrf12_ngdc`；`sgp4.c` 使用全局可变量 + `printf`~~ **S3 已解决**：新写 `adcs_sgp4`（Vallado，WGS-72）、`adcs_igrf`（IGRF-14，栈占用与阶数无关）、`adcs_tle`、`adcs_sun`、`adcs_ephemeris`；`reference/ADCS/` 保持不动 | `src/subsystems/adcs/adcs_{sgp4,igrf,tle,sun,ephemeris}.c`<br>`docs/adcs_orbit_env_models.md` | S3 |
| 9 | RTC 失去备份后复位到 2000-01-01，系统中没有“时间有效”概念。**时间有效性不归 ADCS**（2026-10-14 决定），由 OBC/OBDH 负责；ADCS 直接使用 `time_get_unix()` | `src/utils/periph.c:298-336` | OBC / OBDH |
| 10 | 代码无 INIT 态；SUNSAFE/SURVIVAL 直接暂停 ADCS（文档要求只采 HK）；频率表（文档 CONTINGENCY = 26 MHz）与任务优先级表也与代码不一致 | `src/subsystems/obc/state_machine.c:54-88` vs 文档 §Satellite Operational Modes | S4 / S5 |
| 11 | 没有“设置 ADCS 模式”的 TC；`TC_UPLOAD_ADCS_TLE/CAL` 只有注释；Flash 中 TLE 地址重复定义（`TLE_ADDR` vs `TLE_ADDR1/2`）；HT 存储（`ht_handling.c`）与 EPS 电量读取均为空桩 | `src/subsystems/comms/tc_handler.c:71-117`<br>`include/utils/flash.h:57-65, 97-98` | S5 |
| 12 | 健康检查周期 5 s（`obc.c` 中 `health_config`），暂停 ACK 超时 10 s；`tm_check_pause()` 收到 PAUSE 立即 ACK → ADCS 必须在 ACK 前关闭线圈。**S4 已处理**：PAUSE 先关线圈再 ACK，setup 第一步关线圈，踢狗 ≤ 1 s 与控制周期无关 | `src/subsystems/obc/task_management.c:134-171` | S4 |
| 13 | APE 要求 < 20°；当前 48 h 仿真峰值 18.3°，余量薄，且基于旧 MTQ 量化模型。S3 真值环境（500 km SSO、实际 IGRF、运动太阳）下 10 个工况的最差 APE 为 95.2%，偏离均出现在阴影中开始 nadir 之后（见 #20） | `docs/adcs_nadir_yaw_invariance_analysis.md`<br>`docs/adcs_orbit_env_models.md` §5.2 | S2 / S4a / S6 |
| 14 | 上游 `15a4761` 把 printf 改到 **UART4（PC10/PC11）**；Nucleo 的 ST-LINK 虚拟串口在 USART2，看不到输出 | `src/utils/log.c`、`src/utils/periph.c` | S1（上板） |
| 15 | `build.sh` 需要环境变量 `POCAT_PSK`（32 位十六进制，生成 `psk.h`）；`make` 失败时 `build.sh` 仍返回 0 | `build.sh` | S1 |
| 16 | `./build.sh --mock` 编译失败：`radiolib_wrapper_mock.c` 仍用 TT&C v2 已删除的 `RadioEvents_t`（归 COMMS）。无射频的 Nucleo 上板与 PIL 需要它 | `src/utils/RadioLib/radiolib_wrapper_mock.c` | S1（上板）/ S4 PIL |
| 17 | 状态已改名为 `OBC_STATE_BOOT/NM/CM/SSM/SM/SAFE_HOLD`；`BOOT` 有枚举，但 `tasks_for_state()` 的 default 分支会启动全部任务，没有 INIT 逻辑 → #10 仍成立 | `include/subsystems/obc/state_machine.h` | S4 / S5 |
| 18 | ~~飞控端 `adcs_mode_init()` 只初始化 `mode`、计数器和 `dt`，其余字段未定义~~ **S4 已解决**：整个 `adcs_state_t` 先清零 | `src/subsystems/adcs/adcs_mode_manager.c:28` | S4a |
| 19 | ~~`determination_update()` 的阴影分支用从未更新的 `b_field_eci_prev` 求 dB/dt~~ **S4 已解决**：删除这个不可达的分支和不用的字段，函数只做日照 TRIAD | `src/subsystems/adcs/adcs_determination.c:107-141`<br>`src/subsystems/adcs/adcs_nadir.c:51` | S4a |
| 20 | ~~`nadir_init()` 把姿态估计置为单位四元数；在阴影中进入 nadir 时，第一次 TRIAD 之前从任意初值做陀螺传播~~ **S4 已解决**：第一次日照 TRIAD（`attitude_valid`）之前不输出。`nadir_ephem` 的 APE 从 99.1% 变为 96.4%（仍 ≥ 95%）：测试把日出前自由漂移的偶然穿越算作捕获，见预算表 S4 节 | `src/subsystems/adcs/adcs_nadir.c:55`<br>`docs/adcs_orbit_env_models.md` §7.1 | S4a |
| 21 | ~~星上没有由 6 路 CSS 电压求太阳矢量和 `sun_visible` 的代码~~ **S4 已解决**：`determination_css_sun()`（门限 0.1 V TBC S6），并用星历阴影否决可见性 | `src/subsystems/adcs/adcs_hal_board.c` | S4a |
| 22 | ~~`DETUMBLE_DEAD_TIME_S` = 3.5 ms 偏小~~ **S4 已更新估算**为 25 ms（磁强计 SET/RESET 约 18 ms + 温度 + 陀螺 + CSS + 关线圈）；实测待 S4b | `include/subsystems/adcs/adcs_config.h`<br>`docs/adcs_hw_interface.md` §6 | S4a |
| 23 | `periph_reconfigure_for_freq()` 重新初始化 I2C3/ADC2 时不取 `i2c_bus` 的锁，换频期间如果 ADCS 正在传输会出错 | `src/utils/periph.c` | S4b（与 OBC 对齐） |
| 24 | `tm_reset_task()` 可能在 ADCS 持有 I²C3 互斥量时删除任务，互斥量随之永远被占用。S4 把 `i2c_bus` 的等待改为最多 100 ms 后报错（不再永久阻塞），但这个互斥量本身仍然无法释放 | `src/utils/i2c_bus.c`<br>`src/subsystems/obc/task_management.c:103-120` | S4b（与 OBC 对齐） |
| 25 | `OBDH_Read_Request()` 等待 OBDH 应答没有超时；ADCS 载入 TLE 时如果 OBDH 没有运行，就会一直阻塞，直到被健康检查复位 | `src/utils/flash.c:221-233` | S5（与 OBDH 对齐） |

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

- [x] 安装工具链：Arm GNU Toolchain 14.3.Rel1（`~/opt/`）；开发用 `POCAT_PSK` 放在 `~/.config/pocat/env`。`stlink` 等拿到硬件再装。
- [x] 合并上游 `15a4761` 到 `dev`；`obc.c` 逐字取上游版本，`.gitignore` 两侧都保留。
- [x] `adcs.c`：删除本地 `ADCS_*_MODE` 宏，改用 `N_ADCS_DESIRED_STATE_IDLE/DETUMBLING/NADIR`；为 IDLE、`N_ADCS_NEW_CALIBRATION`、`N_ADCS_NEW_TLE` 预留空分支。
- [x] 注释修正：nadir 轴 +Y → +Z；`adcs_nadir.h/.c` 的算法说明更新为约化姿态误差 + 轨道角速度补偿；MTQ 电流上下限标注“TBC in S2”。
- [x] 主机回归：`ctest` 登记 `mode_manager`、`detumble`、`nadir`、Monte Carlo 30°/s 和 90°/s（各 N = 50），全套约 14 s。顺带修复：测试中 `adcs_state_t` 未初始化（detumble NaN）；`test_nadir` 判据“≥ 3 次捕获”改为任务 APE 判据（首次捕获后 < 20° 的时间 ≥ 95%）。

**出口准则**

- [x] `./build.sh` 零错误生成 `.elf`，基线见 `docs/adcs_resource_budget.md`（FLASH 7.54%，RAM 61.43%，ADCS 栈静态估算 ≈ 60%）
- [ ] `./build.sh --mock` 零错误 —— 被发现 #16 阻塞（COMMS）
- [x] `ctest` 全部通过（5/5）
- [ ] **待硬件**：Nucleo-L476RG 上电打印 banner，所有任务创建成功，连续运行 10 min 无 IWDG 复位。接线：3.3 V USB-TTL 接 PC10 = TX、PC11 = RX、GND，115200 8N1。判定：10 min 内 banner 只出现一次（IWDG 超时 ≈ 4095 × 256 / 32 kHz ≈ 32.8 s，复位会重新打印 banner），且没有 `Error creating` 或 `task reset due to health check` 输出。无射频模块时需要先解决 #16。

### S2 BSP 与驱动

目标硬件：**`PQ_ADCS_HBridge`（DRV8214）板**（已确认）。接口、驱动与硬件问题清单见 `docs/adcs_hw_interface.md`。

没有硬件，所以 S2 分成 **S2a**（代码具备、能编译，2026-10-21 完成）和 **S2b**（上板验证）。按决定（2026-10-21）先解决"有无"：驱动主机单元测试、`--adcs-hwtest` 上板测试固件、线圈 R 偏差扫描推迟到测试阶段。

**任务**

- [x] **冻结引脚表。** 从 OBC 原理图与 PCB 网表确认：SCL3/SDA3 = PC0/PC1（AF4），SEL_PH0..2 = PC2/PA11/PA12，ADC_PH = PC5（ADC12_IN14），PoL_ADCS_POWER_EN = PA7。`.ioc` 不由 ADCS 修改（ICD H5）。
- [x] **I²C3。** 启用 HAL I2C；按 80 / 8 / 2 MHz 设置 TIMINGR，并在 `periph_reconfigure_for_freq()` 中重设。`i2c_bus`：互斥、10 ms 超时、按地址计错、9 个 SCL 时钟 + STOP 的锁死恢复。PoL 只控制 DRV8214 的 nSLEEP，**不能给传感器断电**。
- [x] **驱动**（`src/subsystems/adcs/drivers/`，只依赖 `adcs_bus.h`，不依赖 STM32）：
  - **MMC5983MA**：单次测量 18 bit，用 SET/RESET 对消除偏置，读温度，检查产品 ID。传感器轴 → 本体系的转换放在 `adcs_calib_t`。
  - **IIM-42652**：只开陀螺，±250 dps，ODR 100 Hz，低通 10 Hz，读温度。
  - **CSS**：TMUX1108 依次切换通道，以 S7（GND）作为零点；改用**专用 ADC2**，不再需要与 `temperature.c` 互斥。
  - **DRV8214 ×3**（不是 4 片）：只能 I²C 桥控，使用电压调节（8 bit，15.37 mV/LSB），关闭电流调节和失速检测，故障轮询读取。**X 轴地址与 MMC5983MA 冲突，改板前固件不访问 X 轴驱动**。
- [x] **HAL 抽象。** `adcs_hal.h` 提供本体系的校准读数和 MTQ 电源开关，MTQ 输出沿用 `mtq_apply_command()`（板级实现为强符号）。板级实现是 `adcs_hal_board.c`。没有为 HAL 再加 weak 默认实现：主机测试不调用 HAL，PIL（S4）需要时再加。
- [x] **校准数据结构。** `adcs_calib_t`：磁强计硬铁 + 软铁 + 安装旋转，陀螺零偏 + 一阶温度系数 + 安装旋转，CSS 偏置/增益，线圈 R 与温度系数；带版本号、长度和 CRC 字段（CRC 的计算放在 S5）。
- [x] **回灌算法。** 线圈 R 和 N·A 由 PCB 走线算出（`scripts/adcs/coil_from_pcb.py`），按轴设置步长与上限。`sim_mtq_driver.c` 按 code → 电流 → 磁矩的路径仿真，预留 R 误差注入。`ctest` 7/7 通过（未调参），前后对比见 ICD §7。

**出口准则**

- [ ] **S2b**：裸机/单任务测试固件稳定采集 ≥ 1 h，I²C 错误率有记录
- [ ] **S2b**：轴向与极性验证表：转动板子核对符号；单轴通电，用外部磁强计/罗盘核对极性；用万用表实测线圈 R
- [x] 新量化模型下 `test_nadir` 仍满足 APE < 20°（95% 时间）：100%，末轨均值 3.39°（ICD §7）
- [ ] **硬件**：X 轴 DRV8214 地址改板（ICD H1）

### S3 轨道与环境模型

主机先行，可与 S2 并行。设计、验证与误差预算见 `docs/adcs_orbit_env_models.md`。

**任务**

- [x] **SGP4（Vallado 版）。** `adcs_sgp4`：逐行移植近地分支，使用 WGS-72 常数；没有全局量和 `printf`，记录由调用方持有，传播函数只读、可重入；深空根数直接拒绝。`adcs_tle`：定列解析 + 校验和。保持 `double`。
- [x] **IGRF-14**（2025 epoch + SV 至 2030）。系数以 `static const float` 存入 Flash，由 `scripts/adcs/gen_igrf14_coeffs.py` 从 NCEI 文件生成；按 m 列递推，栈占用与阶数无关，递推中的开方改为查表。阶数经开环 + 闭环权衡定为 **n = 7**：方向误差 95 分位 0.25°，闭环与 n = 13 无显著差异；倾斜偶极（n = 1）的 95 分位达 21°。
- [x] **太阳模型。** 天文年历低精度星历（与 astropy 实测相差 0.0092°）+ 圆柱阴影模型，用于与 CSS 的 `sun_visible` 交叉校验。
- [x] **坐标系统一。** 全部参考矢量用 TEME 表达，地磁场经 GMST 旋转在地固系中计算；由于坐标系统一，TEME 与 J2000 之间约 0.37° 的差异不影响姿态。
- [x] **时间。** `adcs_time`：Unix ↔ JD、十进制年、TLE 历元、GMST。**时间有效性不归 ADCS**（2026-10-14 决定，由 OBC/OBDH 负责）；ADCS 只上报 TLE 年龄（超过 `ADCS_TLE_MAX_AGE_S` = 14 天时置 `ADCS_EPH_TLE_STALE`）。
- [x] **仿真中分离“真值模型”与“星上模型”。** `sim_env_init_ephem()` 的真值为设计 TLE（500 km SSO）+ IGRF-14 n = 13 + 运动的太阳；`test_nadir --ephem` 的控制器从 `adcs_ephem_update(ADCS_IGRF_ORDER)` 取参考矢量（ctest `nadir_ephem`）。原 `sim_env_init()` 不变，S1 的回归输出逐位相同。

**出口准则**

- [x] 主机黄金向量比对通过（ctest `ephemeris`，64 项）：SGP4 与 Vallado 相差 7.3 µm（< 1 m），9 组近地根数共 158 个历元，报错截止点一致；IGRF n = 13 / 8 / 1 与 ppigrf 相差 0.0005 nT；太阳与 astropy 相差 0.0092°
- [ ] 在目标板上用 DWT 周期计数测出每次参考矢量更新的 CPU 时间与栈水位 —— **待硬件**：测量钩子已就绪（`./build.sh --adcs-bench`）。Flash 增量（接入后约 +30.7 KB）与静态栈（1.4 / 1.7 KB，都低于 nadir 路径）已记入预算表

### S4 ADCS 任务闭环

2026-10-22 完成 S4a，S4b 已接线（任务外壳直接绑定 `adcs_hal_*`，固件能编译）。设计见 `docs/adcs_task_design.md`。按决定：上电默认 IDLE，不改 OBC 文件；暂不新增主机测试（`adcs_ctrl.c` 只加入主机编译）；PIL 与各项实测等硬件。

**S4a — 任务骨架**（可与 S2/S3 并行，使用 stub HAL）

- [x] `adcs_ctrl_t`（含 `adcs_state_t`、`adcs_orbit_t`）静态分配。一个控制周期抽成不依赖 FreeRTOS/HAL 的 `adcs_ctrl_cycle(ctrl, io, unix_s, dt)`，I/O 通过 `adcs_io_t` 注入；RTOS 任务只做调度外壳。
- [x] **周期调度。** 按绝对截止时间计算等待超时；`health_kick` 与控制周期解耦（每 ≤ 1 s 一次）；周期由模式决定（消旋 1–2 s、nadir 1 s、IDLE 10 s）；`dt` 用实测的 tick 差。
- [x] **周期时序（分时死区）。** MTQ 置零 → 陀螺 → CSS → 磁强计（最后读，前面的读数时间用作电流衰减）→ 参考矢量 → `adcs_mode_step()` → 施加 MTQ。`DETUMBLE_DEAD_TIME_S` 估算改为 25 ms，**实测待硬件**。
- [x] **暂停/复位协议。** 收到 `N_TASK_PAUSE` 时先关线圈，再交给 `tm_check_pause()` 发 ACK；setup（上电或健康复位）第一步关线圈；恢复时当前模式回到 IDLE，计时与滤波器重置。
- [x] **参考矢量。** 每个周期（消旋时除外）先 `adcs_ephem_update()` 再 `adcs_mode_step()`；上电和收到 `N_ADCS_NEW_TLE` 时从 `TLE_ADDR` 载入（暂定 2 × 69 字符，S5 定稿）。
- [x] **模式管理与 FDIR。** 期望/当前模式分离，经 IDLE 排序；nadir 中 |ω| > 5 °/s 持续 10 个周期 → 消旋；星历无效或 TLE 过期 → IDLE（消旋不受影响）；陀螺/磁强计连续 3 个周期无效 → IDLE；第一次 TRIAD 之前不输出（#20）；`nadir_step`/`detumble_step` 检查 `valid`；删除阴影分支（#19）；CSS 求太阳矢量（#21）。门限均 TBC。

**S4b — 真实 HAL 集成**（需要 S2 + S3）

- [x] 接入真实 HAL（`board_io` 绑定 `adcs_hal_*` 与 `mtq_apply_command`）。`i2c_bus` 等锁改为有限超时（#24）。
- [ ] 评估 ADCS 任务优先级（代码中为 2，文档表中为 4），以及 Flash 擦写造成的抖动（用 GPIO / DWT / `adcs_get_task_stats()` 实测）—— **待硬件**

**PIL**

- Nucleo 运行完整固件；主机仿真通过 UART 注入传感器读数、回收 MTQ 指令，复现 `test_nadir` / `test_detumble` 的场景。

**出口准则**

- [ ] PIL 下的消旋与 48 h nadir 场景结果与纯主机仿真一致（统计误差在容差内）
- [ ] 控制周期抖动、CPU 占用（80 MHz 及最低运行频率）、栈水位有记录
- [ ] 暂停、恢复、健康复位后都不会留下通电的线圈

### S5 TC/TM 与星务联动

**任务**

- **TC。** 实现 `TC_UPLOAD_ADCS_TLE`（多包传输 + 行校验和 + 试传播通过后再提交，后两步由 `adcs_orbit_load_tle()` 完成；A/B 双份保留旧 TLE）和 `TC_UPLOAD_ADCS_CAL`；**新增** ADCS 模式/参数 TC（ID 需与地面站软件同步）。
- **Flash。** 合并为单一 ADCS 配置块（版本号 + CRC），清理重复定义的 TLE 地址；优先放在 bank 2，避免擦写时停顿 bank 1 的取指。
- **TM。** 提供 `adcs_get_telemetry()` 快照接口。beacon 目前只用了 48 字节中的 12 字节，为 ADCS 定义字段：模式、|ω|、估计指向误差、太阳可见、传感器健康、TLE 年龄（`tle_age_s`）、星历状态位（`eph.status`）。历史遥测依赖 OBDH 的 `ht_handling` 实现。
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
| 1 | MTQ 硬件模型不一致（DRV8214 vs BD2606MVV），现有 3.6° / 18° 结论可能变化 | **S2 已缓解**：DRV8214 模型（PCB 线圈参数）下 nadir APE 100%、消旋更快；开环 R(T) 增益误差（约 ±25%）的扫描推迟到测试阶段 |
| 2 | IGRF-13 已过期，发射在 2027 Q3 | **S3 已缓解**：使用 IGRF-14（有效至 2030） |
| 3 | 双精度软浮点在低主频状态下的 CPU 开销 | S3 估算约 0.2 M 周期/次（2 MHz 下约 100 ms），IGRF 开方已查表；S4 用 DWT 实测；必要时先降低参考矢量的更新频率，只在测量证明必要时才把 IGRF/控制改为 `float` |
| 4 | RTC 失效时用错误时间算参考矢量，导致错误指向 | 时间有效性由 OBC/OBDH 负责（ADCS 不处理，2026-10-14 决定）；授时 TC |
| 5 | 5 个 ADCS 器件和侧板温度传感器共享一条 I²C3，总线锁死会让 ADCS 全部失明 | 超时 + 9 个 SCL 时钟恢复（S2 已实现）。**PoL 只控制 DRV8214 的 nSLEEP，无法给传感器断电重启**（2026-10-21 原理图核查），所以恢复只能靠时钟；如需断电手段，要向硬件负责人提出 |
| 6 | 星体剩磁和线圈对磁强计的污染 | 分时死区 + 整星亥姆霍兹校准 |
| 7 | 依赖他人未完成的模块（EPS 电量、OBDH 历史遥测、TC ID 与地面站同步） | 尽早定义接口；ADCS 侧先用 TC 驱动测试 |

## 6. 待决策事项

| 决策 | 涉及方 | 最晚阶段 |
|------|--------|---------|
| 是否在代码中加入 INIT 态，以及 LEOP 自动消旋的触发条件（ADCS 侧已定：上电默认 IDLE，只服从 `N_ADCS_DESIRED_STATE_*`；目前没有任何地方发送这些通知） | OBC | S4a → S5 |
| SUNSAFE/SURVIVAL 下 ADCS：只采 HK 还是暂停（两种 ADCS 都已支持：暂停时线圈断电；“只采 HK”= OBC 发送 IDLE 而不暂停） | OBC | S4a → S5 |
| ADCS 任务优先级与各态主频（CONTINGENCY 26 MHz？） | OBC | S4b |
| ~~IGRF 阶数~~ **已决：n = 7**（S3 权衡研究，`docs/adcs_orbit_env_models.md` §5） | ADCS | S3 |
| 新 ADCS TC 的 ID 与 beacon 字段布局 | COMMS + 地面站 | S5 |
| 重启后是否恢复上一次的 ADCS 模式 | OBC + ADCS | S5 |
