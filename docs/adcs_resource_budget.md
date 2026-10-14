# ADCS 资源预算表 — pocat-lektron-sw

> 每个阶段结束时更新一次。测量方法见文末。
> 关联：`docs/adcs_integration_roadmap.md` §4“资源预算表”

## 汇总

| 阶段 | 固件源码 | FLASH | RAM | ADCS 栈（4096 B） | ADCS 单周期 CPU（80 / 26 / 8 / 2 MHz） |
|------|---------|-------|-----|-------------------|----------------------------------------|
| S1 | dev `0510f06` | 79 092 B（7.54%） | 60 384 B（61.43%） | 静态估算 ≈ 2.4 KB（≈ 60%） | — （S3/S4 用 DWT 实测） |
| S2 | | | | | |
| S3 | | | | | |
| S4 | | | | 实测水位 | |

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
