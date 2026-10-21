/**
 * @file adcs_magnetorquer.h
 * @brief Magnetorquer driver abstraction layer.
 *
 * Handles dipole-to-intensity conversion, intensity quantization and
 * dipole clamping to hardware limits. The quantization models the DRV8214
 * in voltage regulation mode: one code = MTQ_VSET_LSB_V across the coil,
 * i.e. a per-axis current step MTQ_INTENSITY_STEP_MA_{X,Y,Z} up to
 * MTQ_VSET_MAX_CODE codes (see adcs_config.h).
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
 * 3. Quantizes intensity to the per-axis step MTQ_INTENSITY_STEP_MA_{X,Y,Z}
 *    (Σ-Δ dithered), limited to ±MTQ_MAX_INTENSITY_MA_{X,Y,Z}.
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
 * @brief Convert a quantized axis current to a signed DRV8214 voltage code.
 *
 * The magnitude is the WSET_VSET value and the sign the bridge direction.
 * Exact for currents produced by mtq_compute_command(); anything else is
 * rounded to the nearest code and limited to ±MTQ_VSET_MAX_CODE.
 *
 * @param axis          0 = X, 1 = Y, 2 = Z
 * @param intensity_ma  Axis current [mA]
 * @return Signed code in [-MTQ_VSET_MAX_CODE, MTQ_VSET_MAX_CODE]
 */
int mtq_intensity_to_code(int axis, double intensity_ma);

/**
 * @brief Send magnetorquer command to hardware (or simulator).
 *
 * Platform-specific implementation: writes the DRV8214s over I2C on STM32,
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
