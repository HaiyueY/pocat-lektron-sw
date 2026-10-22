# ADCS 硬件接口（ICD）与驱动（S2）— pocat-lektron-sw

> 状态：S2a 完成 · 2026-10-21（代码已具备并能编译；上板验证 S2b 待硬件）
> 关联：`docs/adcs_integration_roadmap.md` §3 S2
> 事实来源：`../pocat-hw` 的原理图与 PCB 网表（ADCS `pq_adcs/PQ_ADCS_HBridge`，OBC `pq_obc_comms/pq_obc_comms`，2026-10-09），以及数据手册 DRV8214 SLVSH04（2023-11）、MMC5983MA Rev A（2019-04-03）、IIM-42652 DS-000440 rev 1.0

本文记录 `PQ_ADCS_HBridge` 板与 OBC 之间的接口、固件中的驱动/HAL 结构，以及需要硬件负责人决定的问题。

---

## 1. 需要硬件负责人处理的问题

| # | 问题 | 证据 | 影响 / 建议 |
|---|------|------|-------------|
| H1 | **X 轴 DRV8214（U5）与 MMC5983MA 的 I²C 地址冲突**：U5 的 A1 = A0 = GND，地址为 0x30（DRV8214 Table 8-28：7 位地址 = 0x30 + A3..A0），与 MMC5983MA 的固定地址 0x30 相同 | ADCS 原理图 U5 A0/A1 网络 | 两个器件的寄存器 0x00–0x17 重叠（例如 0x09 在 MMC 上会触发测量/SET，在 DRV 上是 CONFIG0），软件无法规避。**必须改板**，例如把 U5 的 A0 接 VCC，得到 0x32。改板前固件不访问 X 轴驱动（`DRV8214_ADDR_X = DRV8214_ADDR_NONE`） |
| H2 | U8（Y 轴）的地址引脚与注释不一致：A1 悬空、A0 接地，按地址表应为 0011b = **0x33**；原理图注释却写 0001b（0x31） | ADCS 原理图 U8 | 固件按实际连线使用 0x33；请确认设计意图 |
| H3 | 光电二极管极性：TIA 为单电源（2 kΩ ‖ 100 pF，+IN 接地）。如果光电流**流入** −IN，输出会变负并被钳到 0 V | 原理图 LPV542 级、侧板 SLCD-61N8 `PH+`/`PH−` | 请核对 6 路的方向；接反的通道将一直读 0 V |
| H4 | 本体轴系约定：README 把顶/底板称为 ±Z 面，但 ADCS 板的丝印与线圈连接把底板放在 −Y | README、ADCS 板 J6–J12 丝印 | 固件按"底板 = −Y，MagY 板 = +Y"使用；传感器到本体的旋转矩阵待 S2b 极性表确定 |
| H5 | `.ioc`（`pq_obc_comms/STM32L476RG_config/`，2026-09-30）落后于原理图：没有 I2C3；PC0 配成 GPIO；PA7 配成 ADC 输入（实际是 PoL 使能输出）；PC1、PC5 没有配置 | `.ioc` | 固件以原理图为准，`.ioc` 不由 ADCS 修改 |
| H6 | PoL_ADCS_POWER_EN 只接 3 片 DRV8214 的 nSLEEP；传感器、mux 和运放都由 `3V3_PERM` 常供电，OBC 无法给它们断电 | OBC U4（BD2232G）输出网络 | I²C 总线锁死时只能靠 9 个 SCL 时钟恢复，无法断电重启传感器（roadmap 风险 #5 的缓解措施已据此修改） |

---

## 2. 引脚

STM32L476RGT6（LQFP64）。所有网络经堆叠连接器 J1–J4 直通，OBC 与 ADCS 两侧引脚定义相同。

| 网络 | MCU 引脚 | 固件配置 | 用途 |
|------|---------|---------|------|
| SCL3 | PC0 | I2C3_SCL，AF4，开漏 | ADCS I²C 总线；4.7 kΩ 上拉在 OBC 上（R1/R2） |
| SDA3 | PC1 | I2C3_SDA，AF4，开漏 | 同上 |
| SEL_PH0 | PC2 | 推挽输出 | TMUX1108 A0 |
| SEL_PH1 | PA11 | 推挽输出 | TMUX1108 A1 |
| SEL_PH2 | PA12 | 推挽输出 | TMUX1108 A2 |
| ADC_PH | PC5 | ADC2_IN14（模拟，ASCR） | 太阳敏感器 mux 输出 |
| PoL_ADCS_POWER_EN | PA7 | 推挽输出，上电为低 | BD2232G 高边开关 → `ADCS_POWER` → 3 × DRV8214 nSLEEP（高 = 唤醒）；板上有 10 kΩ 下拉 |

宏定义在 `include/utils/periph.h`（`ADCS_*`）。ADCS 板上没有任何中断线或故障线连到 MCU：IIM-42652 的 INT1 悬空、INT2 接地，MMC5983MA 的 INT 悬空，DRV8214 的 nFAULT 和 RC_OUT 悬空。J3.9 上的 `FAULT`（PC4）是 EPS 充电器 LTC4040 的故障输出，与 ADCS 无关。

---

## 3. I²C3 上的器件

| 器件 | 7 位地址 | 依据 |
|------|---------|------|
| MMC5983MA（U7） | 0x30 | 固定；Product ID 0x30 |
| IIM-42652（U4） | 0x68 | AP_AD0 = GND；WHO_AM_I 0x6F |
| DRV8214 X（U5） | ~~0x30~~ **TBD** | 冲突，见 H1 |
| DRV8214 Y（U8） | 0x33 | A1 悬空、A0 = GND（见 H2） |
| DRV8214 Z（U9） | 0x34 | A1、A0 都悬空 |
| TCN75A ×N（侧板） | 0x48–0x4F，由 0R 跳线选择 | 装配时决定，目前未知；不归 ADCS |

总线速率：80 / 8 MHz 下 400 kHz，2 MHz 下约 100 kHz（`i2c3_timing_for_freq()`）。

---

## 4. 磁力矩器

### 4.1 线圈（从 PCB 走线计算）

每轴由对面的两只 PCB 线圈串联而成，由 `scripts/adcs/coil_from_pcb.py` 计算（35 µm 铜，20 °C）：

| 线圈 | 板 | R | N·A |
|------|----|---|-----|
| ±X、±Z | `PQ_Lat_Board` | 21.66 Ω | 0.052935 m² |
| −Y | `PQ_BotBoard` | 21.81 Ω | 0.052255 m² |
| +Y | `PQ_MagYBoard` | 12.49 Ω | 0.028066 m² |

| 轴 | R（20 °C） | coil factor | 最大电流 | 最大磁矩 | 电流步长 |
|----|-----------|-------------|---------|---------|---------|
| X、Z | 43.32 Ω | 0.105870 m² | 71.3 mA | 7.55 × 10⁻³ A·m² | 0.355 mA |
| Y | 34.30 Ω | 0.080321 m² | 90.1 mA | 7.24 × 10⁻³ A·m² | 0.448 mA |

- 侧板和 +Y 线圈的 N·A 与 `Max_mag_moment_calc.m` 的结果相差 0.4% 以内。旧模型假设 Y 轴是两只 +Y 尺寸的线圈（0.056065 m²），实际是一大一小，所以 coil factor 改为 0.0803。
- **旧模型按 150 mA 计算**（15.9 / 8.4 × 10⁻³ A·m²）。改用 3.3 V 电源和上述线圈电阻后，最大磁矩约为原来的一半（Y 轴下降 14%）。
- 铜的电阻温度系数为 +0.39 %/K。开环电压驱动下，−40 °C 和 +85 °C 分别对应约 −24% 和 +26% 的增益误差。仿真接口 `sim_mtq_set_r_error()` 已经预留，扫描推迟到测试阶段。
- 以上数值都需要在 S2b 用万用表实测确认（TBC）。

### 4.2 DRV8214 的配置

EN/IN1、PH/IN2 接地，只能通过 I²C 控制桥（`I2C_BC = 1`）。VM = VCC = VREF = `3V3_PERM`，RIPROPI = 22 kΩ。

| 寄存器 | 值 | 含义 |
|--------|----|------|
| CONFIG4 (0x0D) | 0x0C + IN1/IN2 | PMODE = 1（PWM 真值表：00 惰行，10 正向 OUT1→OUT2，01 反向，11 制动），I2C_BC = 1，不在 nFAULT 上报 |
| CONFIG3 (0x0C) | 0x23 | **IMODE = 00**：关闭电流调节。复位值 01 会在 VREF = 3.3 V、RIPROPI = 22 kΩ 下把电流削掉。OCP/TSD 自动重试 |
| RC_CTRL0 (0x11) | 0x0B | 关闭纹波计数；CS_GAIN_SEL = 011b（1125 µA/A，满量程 0.5 A；电流在约 83 mA 以内时 IPROPI 处于线性区） |
| REG_CTRL0 (0x0E) | 0x18 | **EN_SS = 0**（设定值立即生效），REG_CTRL = 11b（电压调节），PWM 50 kHz |
| REG_CTRL1 (0x0F) | \|code\| | WSET_VSET，15.37 mV/LSB（VM_GAIN_SEL = 1，0–3.92 V），固件上限 201 = 3.09 V |
| REG_CTRL2 (0x10) | 0xC0 | OUT_FLT = 1 kHz（比 PWM 低 50 倍） |
| CONFIG0 (0x09) | 0xC8 (+CLR_FLT) | EN_OUT、EN_OVP、VM_GAIN_SEL；**EN_STALL = 0** |

- nSLEEP 拉低后，寄存器会复位。所以每次唤醒都要先等 tWAKE（410 µs，固件取 500 µs），再重新执行 `drv8214_init()`。
- 读回寄存器：VMTR 每 LSB 62.5 mV，IMTR 每 LSB 2.60 mA，FAULT 位定义见 `drv8214.h`。
- 指令路径：`mtq_compute_command()` 量化出的电流都是步长的整数倍，`mtq_intensity_to_code()` 把它精确换成带符号的 code，再由 `drv8214_set_code()` 写入。code 的符号与磁矩方向的对应关系，取决于线圈的绕向（TBC，见 S2b 极性表）。

---

## 5. 太阳敏感器（CSS）

6 个面各有一只 SLCD-61N8 光电二极管和一级 LPV542 跨阻放大器，TMUX1108 再把 6 路选到 `ADC_PH` 上（mux 的 EN 常开）：

| mux 输入 | S1 | S2 | S3 | S4 | S5 | S6 | S7、S8 |
|----------|----|----|----|----|----|----|--------|
| 面 | −X | +X | −Z | +Z | −Y | +Y（`PH_PL`，也接到载荷连接器 J4.10） | GND |

- `css_read()` 先采 S7（GND）作为 ADC 零点，再按本体面顺序 +X、−X、+Y、−Y、+Z、−Z（与 `sun_sensor_data_t` 一致）采样。每次切换后等待 20 µs，每通道平均 4 次。
- CSS 使用 **ADC2**，与 `temperature.c` 使用的 ADC1 分开，所以不需要互斥锁（roadmap 发现 #6 的 ADC 部分已解决）。
- 换算时 VDDA 取 3.3 V 标称值。

---

## 6. 固件结构

```
ADCS 任务（S4a）
   │  adcs_hal.h：adcs_hal_init / read_mag / read_gyro / read_css / mtq_enable / mtq_status
   │  adcs_magnetorquer.h：mtq_apply_command（板级强符号覆盖 weak 默认）
   ▼
adcs_hal_board.c ── adcs_calib.c（原始值 → 本体系，adcs_calib_t）
   │  drivers/adcs_bus.h：adcs_bus_read/write、delay、css_select/sample、mtq_power
   ▼
drivers/mmc5983ma.c  iim42652.c  drv8214.c  css.c   （不依赖 STM32）
   ▼
i2c_bus.c（I2C3：互斥、10 ms 超时、按地址计错、9 时钟恢复）  periph.c（I2C3、ADC2、GPIO）
```

| 文件 | 内容 |
|------|------|
| `include/subsystems/adcs/adcs_hal.h` | 任务侧接口；读数已经过校准并转到本体系 |
| `include/subsystems/adcs/adcs_calib.h`、`src/.../adcs_calib.c` | `adcs_calib_t`（S5 上传格式，float，带版本号、长度和 CRC 字段）与修正模型；默认值为单位阵和零 |
| `src/subsystems/adcs/adcs_hal_board.c` | 板级实现（只编进固件；主机测试显式列出源文件，不包含它） |
| `include/subsystems/adcs/drivers/*.h`、`src/subsystems/adcs/drivers/*.c` | 4 个器件驱动和 `adcs_bus.h`；固件的 `GLOB_RECURSE` 会自动收录 |
| `include/utils/i2c_bus.h`、`src/utils/i2c_bus.c` | I2C3 总线层；`main()` 中调用 `i2c_bus_init()` |

传感器配置：

- **MMC5983MA**：单次测量，18 bit，BW = 100 Hz（每次测量 8 ms）。每次读场都做一对 SET/RESET，H = (SET − RESET)/2，这样可以消除偏置及其温漂。一次读场约 18 ms，另加一次温度测量。
- **IIM-42652**：只开陀螺，低噪声模式，±250 dps（131 LSB/(°/s)），ODR 100 Hz，UI 低通 10 Hz；加速度计关闭。一次突发读同时取得角速度和温度；读到 −32768 时视为无效。

---

## 7. 对算法的影响（主机回归，2026-10-21）

只对量化模型和最大磁矩做了替换，**没有调参**。`ctest` 7/7 通过。

| 场景 | S2 前（150 mA、0.5 mA 步长） | S2 后（DRV8214 模型） |
|------|------------------------------|-----------------------|
| `nadir`：APE / 末轨均值 / 峰值 | 100% / 4.28° / 20.0° | 100% / 3.39° / 20.0° |
| `nadir_ephem`：APE / 末轨均值 / 峰值 | 99.1% / 3.28° / 23.7° | 99.1% / 3.28° / 23.7° |
| 消旋 Monte Carlo 30 °/s（N = 50）：通过率 / 收敛时间均值 / 最大值 | 50/50，4419 s / 5766 s | 50/50，2884 s / 3352 s |
| 消旋 Monte Carlo 90 °/s（N = 50）：通过率 / 收敛时间均值 / 最大值 | 50/50，6071 s / 8258 s | 50/50，4150 s / 5674 s |

最大磁矩减半后，消旋反而变快了。原因是 ω×B 增益 `BDOT_GAIN_K` 由平均最大磁矩推导而来，会随之下降，饱和区间缩小，轴间耦合造成的能量再分配也随之减少。这一机理还没有专门验证。nadir 稳态的磁矩远小于新上限，量化步长又与旧值相近，所以结果几乎不变。

---

## 8. 已知缺口（留给后续阶段）

| 项 | 阶段 |
|----|------|
| ~~ADCS 任务尚未调用 `adcs_hal_*`，驱动和板级 HAL 会被 `--gc-sections` 裁掉~~ **S4 已接入**，入账见 `docs/adcs_resource_budget.md` S4 节 | S4a/S4b |
| ~~星上还没有由 6 路电压求太阳矢量和可见性的代码~~ **S4 已解决**：`determination_css_sun()` | S4a |
| ~~`DETUMBLE_DEAD_TIME_S` = 3.5 ms 的假设偏小~~ **S4 已改为 25 ms 估算值**，实测待 S4b | S4a/S4b |
| `periph_reconfigure_for_freq()` 重新初始化 I2C3/ADC2 时没有取总线锁；换频期间如果正在传输，会出错 | S4b |
| CRC 的计算与校验 | S5 |
| 驱动的主机单元测试、`--adcs-hwtest` 上板测试固件、线圈 R ±25% 扫描 | 测试阶段（推迟） |
| 1 h 采集、I²C 错误率、轴向/极性表、线圈 R 实测 | S2b（到板后） |
