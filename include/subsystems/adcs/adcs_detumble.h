/**
 * @file adcs_detumble.h
 * @brief Gyroscope-based proportional detumbling controller (ω×B law).
 *
 * Implements the proportional ω×B law for satellite detumbling:
 *   m[i] = clamp(k × (ω × B)_i, -m_max[i], +m_max[i])
 *
 * Mathematically equivalent to ideal continuous B-DOT (since dB_body/dt ≈ -ω × B),
 * but uses the gyroscope directly — eliminating finite-difference phase error and
 * sinc attenuation. This enables reliable detumbling at 1 Hz control rate.
 *
 * MATLAB reference: ref/PoCat-Lektron-ADCS/ADCS/Detumbling.m
 * (upgraded from finite-difference B-DOT to ω×B)
 */

#ifndef ADCS_DETUMBLE_H
#define ADCS_DETUMBLE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "adcs_types.h"

/**
 * @brief Initialize detumbling mode state.
 *
 * Resets the previous magnetic field reading and stable counter.
 *
 * @param[in,out] state  ADCS state to initialize for detumbling
 */
void detumble_init(adcs_state_t *state);

/**
 * @brief Select adaptive control period for detumbling.
 *
 * Follows ESA 4× Nyquist rule: f_ctrl = 4 × f_rot = 2|ω|/π.
 * Returns ΔT = π/(2|ω|), clamped to [1.0 s, 2.0 s] (0.5–1 Hz).
 *
 * This maintains constant ZOH efficiency G = sinc(π/4) ≈ 0.900,
 * reducing control frequency at low ω to save power.
 *
 * @param[in] state  ADCS state (reads gyro angular velocity)
 * @return Control period ΔT ∈ [1.0, 2.0] s
 */
double detumble_select_dt(const adcs_state_t *state);

/**
 * @brief Execute one detumbling control step.
 *
 * Gyroscope-based ω×B law with per-axis saturation:
 *   1. Compute ω × B_body (equivalent to ideal -dB/dt)
 *   2. m_raw[i] = k × (ω × B)_i; m[i] = clamp(m_raw, ±m_max)
 *   3. Quantize intensity via mtq_compute_command()
 *   4. Check angular velocity threshold for exit condition
 *
 * @param[in,out] state  ADCS state (reads mag/gyro, writes mtq_cmd)
 * @return 1 if detumbling is complete (ω below threshold), 0 otherwise
 */
int detumble_step(adcs_state_t *state);

#ifdef __cplusplus
}
#endif

#endif /* ADCS_DETUMBLE_H */
