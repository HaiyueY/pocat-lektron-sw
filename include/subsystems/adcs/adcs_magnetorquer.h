/**
 * @file adcs_magnetorquer.h
 * @brief Magnetorquer driver abstraction layer.
 *
 * Handles dipole-to-intensity conversion, intensity quantization
 * matching the BD2606MVV driver constraints (0.5-32 mA, 0.5 mA step),
 * and dipole clamping to hardware limits.
 *
 * MATLAB reference: Detumbling.m L144-184, Nadir_pointing.m L196-238
 */

#ifndef ADCS_MAGNETORQUER_H
#define ADCS_MAGNETORQUER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "adcs_types.h"

/**
 * @brief Clamp dipole to maximum hardware limits and compute quantized intensity.
 *
 * 1. Clamps each dipole component to ±max_dipole per axis.
 * 2. Converts dipole to intensity [mA] using coil factors.
 * 3. Quantizes intensity to 0.5 mA steps within [0.5, 32] mA range.
 * 4. Recomputes dipole from quantized intensity for consistency.
 *
 * @param[in]  desired_dipole  Desired magnetic dipole [A·m²]
 * @param[out] cmd             Output command with clamped dipole and quantized intensity
 */
void mtq_compute_command(vec3d_t desired_dipole, mtq_command_t *cmd);

/**
 * @brief Reset sigma-delta current accumulator.
 *
 * Call when switching ADCS modes (e.g., detumble → nadir pointing)
 * to avoid stale accumulated commands from the previous mode.
 */
void mtq_reset_accumulator(void);

/**
 * @brief Send magnetorquer command to hardware (or simulator).
 *
 * Platform-specific implementation: writes I2C registers on STM32,
 * or logs to file in simulation mode.
 *
 * @param[in] cmd  Magnetorquer command to apply
 * @return 0 on success, -1 on error
 */
int mtq_apply_command(const mtq_command_t *cmd);

#ifdef __cplusplus
}
#endif

#endif /* ADCS_MAGNETORQUER_H */
