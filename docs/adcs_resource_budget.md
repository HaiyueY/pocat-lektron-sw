# ADCS 资源预算表 — pocat-lektron-sw

> 每个阶段结束时更新一次。测量方法见文末。
> 关联：`docs/adcs_integration_roadmap.md` §4“资源预算表”

## 汇总

| 阶段 | 固件源码 | FLASH | RAM | ADCS 栈（4096 B） | ADCS 单周期 CPU（80 / 26 / 8 / 2 MHz） |
|------|---------|-------|-----|-------------------|----------------------------------------|
| S1 | dev `0510f06` | 79 092 B（7.54%） | 60 384 B（61.43%） | 静态估算 ≈ 2.4 KB（≈ 60%） | — （S3/S4 用 DWT 实测） |
| S2 | | | | | |
| S3 | dev `a585e78` | 79 092 B（7.54%）不变（未接入，被 gc）；接入后约 +30.7 KB | 60 384 B 不变；模型本身无静态 RAM | 参考矢量更新 1428 B / 载入 TLE 1740 B，都低于 nadir 路径，最坏仍 ≈ 2.4 KB | 估算约 0.2 M 周期/次（±2×）≈ 2.5 / 7.7 / 25 / 100 ms；待 `ADCS_BENCH` 实测 |
| S4 | dev `88fc8eb`（S4a + S4b 接线） | 130 796 B（12.47%）：+50 540 B，算法、模型和 libm 第一次被链接 | 61 960 B（63.03%）：+1 392 B（静态 `adcs_ctrl_t`） | 静态：载入 TLE 2 084 B，控制周期 2 000 B，≈ 56%；实测水位待硬件 | 待硬件（`adcs_get_task_stats()` + DWT） |

---

## S1 基线（2026-10-14）

- 工具链：Arm GNU Toolchain 14.3.Rel1（GCC 14.3.1），`-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard`
- 构建：`./build.sh`（Debug，`-Og -g3`，`RADIO_MOCK=OFF`）
- 源码：dev `0510f06` = 上游 `15a4761` + ADCS 算法 + S1 清理

### 整体占用

| 区域 | 已用 | 容量 | 占比 | 备注 |
|------|------|------|------|------|
| FLASH | 79 092 B | 1 MB | 7.54% | text 78 708 B + data 376 B |
| RAM | 60 384 B | 96 KB | 61.43% | bss 60 008 B，其中 FreeRTOS 堆 49 152 B（`configTOTAL_HEAP_SIZE` = 48 KB，heap_4） |
| RAM2 | 0 B | 32 KB | 0% | 未使用 |

### ADCS 代码

`adcs_task` 目前只是一个外壳，不调用任何算法，所以 `--gc-sections` 把算法代码从 `.elf` 中全部去掉了，固件里只有 `adcs_task` / `process_adcs` / `setup_adcs`（共 62 B）。下表是各目标文件的大小，即 S4 接入后 Flash 增量的上限估计（不含 libm / libgcc 软双精度例程）：

| 目标文件 | text | bss |
|----------|------|-----|
| `adcs_nadir.c` | 5 868 B | 0 |
| `adcs_determination.c` | 3 446 B | 0 |
| `adcs_detumble.c` | 1 150 B | 0 |
| `adcs_magnetorquer.c` | 540 B | 24 B |
| `adcs_mode_manager.c` | 254 B | 0 |
| `adcs.c` | 62 B | 4 B |
| **合计** | **11 320 B** | **28 B** |

`sizeof(adcs_state_t)` = 552 B（ARM，AAPCS 下 `double` 按 8 字节对齐）。S4a 计划改为静态分配。

### ADCS 任务栈（静态分析）

`ADCS_STACK_SIZE` = 1024 words = 4096 B（`include/subsystems/obc/obc.h:25`）。

| 路径 | 字节 |
|------|------|
| `adcs_mode_step → nadir_step(896) → determination_update(504) → triad_compute(400) → build_triad(120) → vec3d_normalize(160) → vec3d_zero(80)` | 2 168 |
| `detumble_step(152) → vec3d_cross(184) → __aeabi_dmul` | ≈ 384 |
| 任务外壳 `adcs_task` + `process_adcs` + `wait_for_notification` | ≈ 40 |
| FreeRTOS 切换上下文（含 FPU 扩展帧，估计） | ≈ 200 |
| **最坏合计（估计）** | **≈ 2.4 KB（≈ 60%）** |

说明：

- libm 的 `sin` / `cos` 在大参数下会进入 `__kernel_rem_pio2`，整条链约 768 B。它不在上面的最坏路径上（nadir 路径中经 `quat_gyro_propagate` 调用，合计约 1.9 KB），但如果以后在更深的调用里用到三角函数，要重新计算。
- 这是 `-Og` 下的静态估计，不含 S2 驱动、`printf`（newlib-nano 下单次调用可能占用数百字节）和中断嵌套。S4 用 `uxTaskGetStackHighWaterMark()` 实测，同时把 `configCHECK_FOR_STACK_OVERFLOW` 从 1 改为 2。

### 编译 warning

共 9 条，**ADCS 文件 0 条**。其余都在其他子系统（`main.c` 1、`obc.c` 1、`ht_handling.c` 4、`compression.c` 2、`radiolib_wrapper_mock.c` 1），不在 ADCS 范围内，没有处理。

### 已知问题

- `./build.sh --mock` 编译失败：`src/utils/RadioLib/radiolib_wrapper_mock.c` 仍在使用 TT&C v2 中已删除的 `RadioEvents_t`（归 COMMS）。在没有射频模块的 Nucleo 上做上板测试时需要它。
- `build.sh` 没有开 `set -e`，`make` 失败时仍然返回 0。要以“是否生成 `.elf`”或日志中有无 `error:` 来判断。

---

## S3 参考模型（2026-10-14）

- 源码：dev `a585e78`（新增 `adcs_time` / `adcs_tle` / `adcs_sgp4` / `adcs_igrf` / `adcs_sun` / `adcs_ephemeris` / `adcs_bench`，`ADCS_IGRF_ORDER` = 7）
- 构建：与 S1 相同（Debug，`-Og -g3`）。另做一次 `ADCS_BENCH=ON` 构建，让基准代码引用全部模型，从而得到真实的链接增量（包括 libm）。

### 整体占用

| 构建 | FLASH | RAM | 说明 |
|------|-------|-----|------|
| 默认 `./build.sh` | 79 092 B（7.54%） | 60 384 B（61.43%） | 与 S1 相同：`adcs_task` 还没有调用模型，`--gc-sections` 把它们全部去掉 |
| `./build.sh --adcs-bench` | 111 288 B（10.61%） | 60 744 B（61.79%） | +32 196 B / +360 B |

### FLASH 增量构成（`test/tools/map_diff.py` 比较两个 `.map`）

| 来源 | 字节 | 主要组成 |
|------|------|---------|
| ADCS 模型 | 18 124 | `adcs_sgp4` 7 628、`adcs_igrf` 4 800（代码 1 840 + 系数与递推表 2 960）、`adcs_tle` 1 926、`adcs_ephemeris` 1 650、`adcs_sun` 1 152、`adcs_time` 960 |
| libm | 12 220 | `pow` 3 436（SGP4 使用）、`sin`/`cos` 的 `rem_pio2` 链约 3 700、`fmod` 1 064、`atan`/`atan2` 1 400、`sqrt` 616、`floor` 264 等 |
| libgcc | 396 | 双精度比较与转换（加减乘除在 S1 固件中已经链接） |
| 基准本身（不属于飞行代码） | 1 338 + 298 | `adcs_bench` 代码与字符串；FreeRTOS `uxTaskGetStackHighWaterMark` |
| RAM | 360 | 基准用的静态 `adcs_orbit_t`；模型本身没有静态数据 |

S4 接入后预计增加约 30.7 KB（模型 18.1 KB + libm 12.2 KB + libgcc 0.4 KB）。其中 `sin`/`cos`/`sqrt`/`atan2` 也会被 nadir 与定姿算法使用，届时增量的归属会有重叠。如果以后 Flash 紧张，可以先把 SGP4 中的两处 `pow` 改写（可省约 3.4 KB），但目前 Flash 只用了 10.6%，暂不需要。

### 静态栈（`test/tools/stack_depth.py`，基于 bench ELF 的反汇编）

ADCS 函数的帧大小与 GCC `-fstack-usage` 的结果一致；libm 的帧由反汇编得到。

| 入口 | 最坏深度 | 最坏路径 |
|------|---------|---------|
| `adcs_ephem_update` | 1 428 B | `adcs_ephem_update`(368) → `adcs_igrf_field_ecef`(256) → `sin`(24) → `__ieee754_rem_pio2`(96) → `__kernel_rem_pio2`(648) → `floor`(24) → `__adddf3`(12) |
| `adcs_orbit_load_tle` | 1 740 B | `adcs_orbit_load_tle`(416) → `adcs_sgp4_init`(520) → `sin` → … 同上 |
| `adcs_sgp4_propagate` | 1 028 B | |
| `adcs_igrf_field_ecef` | 1 060 B | 与阶数无关（不需要 P[n][m] 表） |

S4 的任务按顺序调用 `adcs_ephem_update()` 和 `adcs_mode_step()`，两者不嵌套，所以 ADCS 栈的最坏值仍由 S1 的 nadir 路径决定（2 168 B + 外壳 40 B + 上下文约 200 B ≈ 2.4 KB，约 60%）。新 TLE 在 ADCS 任务中载入时为 1 740 B，同样低于这个值。

注意：`-Og` 下按值传递的 `vec3d_t` 会产生较大的帧（例如 `vec3d_rot_z` 160 B）。S4 如果改用 `-O2`，需要重新测量。

### CPU（待硬件）

在目标板上测量：

```bash
./build.sh --adcs-bench     # 烧录后在 UART4（115200 8N1）读取 [ADCS_BENCH] 行
```

输出依次为 `SystemCoreClock`、SGP4、太阳模型、IGRF（n = 1 / 4 / 8 / 13）和完整 `adcs_ephem_update` 的 min / avg / max 周期数，以及 ADCS 栈的剩余字节数。各测 16 个历元，沿设计轨道间隔 600 s。

在拿到实测值之前，先用运算量做数量级估算。M4 的 FPU 只支持单精度，双精度全部走软件：libgcc 的乘、加约 30–60 周期；newlib 的 `sin`/`cos` 约 2–3 k 周期；`pow` 约 5–10 k 周期。

| 部分 | 运算量 | 估算周期数 |
|------|-------|-----------|
| IGRF n = 7 | 35 项 × 约 38 次双精度运算 + `atan2`/`sin`/`cos`/`sqrt` | 约 70 k |
| SGP4 | 约 250 次双精度运算 + 约 25 次三角函数 + 2 次 `pow` + 5 次 `fmod` + 4 次 `sqrt` | 约 90 k |
| 太阳模型、GMST、两次 R3 旋转、十进制年 | | 约 40 k |
| **合计** | | **约 0.2 M（±2×）** |

折合 80 / 26 / 8 / 2 MHz 下约为 2.5 / 7.7 / 25 / 100 ms。在 2 MHz、1 Hz 控制频率下约占 10% CPU；必要时可以降低参考矢量的更新频率（例如 10 s 更新一次并插值）。在 IGRF 递推中改为查表平方根常数之前，n = 13 时还要多约 208 次软 `sqrt`。

---

## S4 任务闭环（2026-10-22）

- 源码：dev `88fc8eb`（`adcs_ctrl` 控制周期 + `adcs.c` 任务外壳 + `i2c_bus` 锁超时），对比基准为 S2a 的 dev `80f1db2`
- 构建：`./build.sh --clean`（Debug，`-Og -g3`）。warning 9 个，与基准相同，ADCS 与 `i2c_bus` 中 0 个

### 整体占用

| 构建 | FLASH | RAM |
|------|-------|-----|
| S2a dev `80f1db2` | 80 256 B（7.65%） | 60 568 B（61.61%） |
| S4 dev `88fc8eb` | 130 796 B（12.47%） | 61 960 B（63.03%） |
| 增量 | +50 540 B | +1 392 B |

### FLASH 增量构成（`map_diff.py`）

| 来源 | 字节 | 主要组成 |
|------|------|---------|
| ADCS 目标文件 | 34 965 | `adcs_sgp4` 7 628、`adcs_nadir` 5 920、`adcs_determination` 3 204、`adcs_igrf` 4 800、`adcs_ephemeris` 1 806、`adcs_tle` 1 790、`adcs_detumble` 1 226、`adcs_ctrl` 1 166、`adcs_sun` 1 152、`adcs_time` 960、`adcs_calib` 902、`adcs_hal_board` 816、`adcs` 684、驱动 1 388 |
| libm | 12 504 | 与 S3 估算一致（`pow`、`rem_pio2` 链、`fmod`、`atan`/`atan2`、`sqrt` 等） |
| libgcc | 460 | |
| 其他 | 2 610 | HAL I²C 1 782、`i2c_bus` 638 |

S3 预计“接入后约 +30.7 KB”只包括模型；实际增量更大，是因为 S2 的驱动/HAL 和 S1 的算法也是在这一步第一次被任务引用、不再被 gc 掉。

RAM +1 392 B：`adcs.c` 的 bss 1 002 B（`adcs_ctrl_t`，其中 `adcs_sgp4_t` 和 `adcs_tle_t` 占大部分）、`adcs_hal_board.c` 228 B（校准记录）、其余 FreeRTOS/HAL。

### 静态栈（`stack_depth.py`，默认构建的 ELF）

| 入口 | 最坏深度 | 最坏路径 |
|------|---------|---------|
| `adcs_task` | 2 084 B | `adcs_task`(8) → `process_adcs`(16) → `load_tle_from_flash`(312) → `adcs_orbit_load_tle`(416) → `adcs_sgp4_init`(520) → `cos` → `__kernel_rem_pio2`(648) → … |
| `run_cycle` | 2 000 B | `run_cycle`(32) → `adcs_ctrl_cycle`(32) → `adcs_mode_step`(8) → `nadir_step`(896) → `determination_update`(272) → `triad_compute`(400) → … |
| 间接调用：`read_sensors` → `adcs_hal_read_mag` | 约 560 B | `adcs_hal_read_mag` 480 B |
| 间接调用：`apply_output` → `mtq_apply_command` | 约 430 B | |
| `update_references` | 1 444 B | 与 `adcs_mode_step` 顺序调用，不嵌套 |

最坏为 2 084 B，加上任务上下文约 200 B 后约为 2.3 KB / 4 096 B（≈ 56%）。工具不跟踪函数指针，所以 `adcs_io_t` 的调用单独列出；它们都比 nadir 路径浅。`printf` 在 `load_tle_from_flash` 中是在 TLE 载入返回之后才调用的，不与上面的路径嵌套。

### 回归（主机）

- 第 1 个提交（#18/#19/#20 + 有效标志 + 消旋 Σ-Δ 复位）：`nadir`、`detumble` 逐位不变。Monte Carlo 只有末端 |ω| 有微小变化：Σ-Δ 累加器是静态变量，原来会从上一次试验带到下一次，现在每次进入消旋都会复位（30 °/s 平均收敛时间 2884.0 → 2884.1 s，90 °/s 4150.1 → 4148.7 s，均为 50/50）。`nadir_ephem` 在阴影中起步：原来从任意初值出发做陀螺传播，碰巧把误差带到了 15°；现在日出前不施加控制，卫星自由漂移时经过一次 < 20°（“首次捕获”计时从这里开始），日出时漂移到 68°，约 650 s 后重新收敛。APE 99.06% → 96.41%（门限 95%，仍通过），末轨均值 3.28° → 3.19°，峰值 23.7° → 68.5°。峰值来自测试把未受控的偶然穿越算作捕获。
- 第 3 个提交（死区 3.5 → 25 ms）：30 °/s 平均收敛时间 2884.1 → 2880.0 s，90 °/s 4148.7 → 4213.3 s，均为 50/50；`test_detumble` 从 5668.9 s 变为 5687.9 s。
- 其余提交（CSS 太阳矢量、`adcs_ctrl`、任务外壳、`i2c_bus`）：所有输出逐位不变。`ctest` 7/7。

---

## 测量方法

```bash
export PATH="$HOME/opt/arm-gnu-toolchain-14.3.rel1-x86_64-arm-none-eabi/bin:$PATH"
source ~/.config/pocat/env                     # POCAT_PSK
./build.sh --clean                             # 日志中的 Memory region 表即 FLASH/RAM

# ADCS 目标文件大小
arm-none-eabi-size $(find build -path '*subsystems/adcs/*.obj')

# 静态栈：单独的构建目录 + 调用图
cmake -S . -B <tmp>/build-su -DCMAKE_C_FLAGS="-fdata-sections -ffunction-sections -fstack-usage"
cmake --build <tmp>/build-su -j
#   对 src/subsystems/adcs/*.c 另加 -fcallgraph-info=su 编译，
#   沿 .ci 调用图累加各函数 .su 栈帧，求最坏路径
```

S3 起新增（库函数没有 `.su` 文件，所以改为直接分析 ELF）：

```bash
# 链接增量：默认构建 vs 让模型全部被引用的 bench 构建
cmake -S . -B <tmp>/fw-bench -DADCS_BENCH=ON && cmake --build <tmp>/fw-bench -j
python3 -I test/tools/map_diff.py build/pocat_lektron_sw.map <tmp>/fw-bench/pocat_lektron_sw.map

# 静态栈（包括 libm / libgcc）
python3 -I test/tools/stack_depth.py <tmp>/fw-bench/pocat_lektron_sw.elf \
    adcs_ephem_update adcs_orbit_load_tle
```
