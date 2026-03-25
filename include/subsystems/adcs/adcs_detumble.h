/**
 * @file adcs_detumble.h
 * @brief Proportional B-DOT detumbling controller with adaptive saturation.
 *
 * Implements the proportional B-DOT law for satellite detumbling:
 *   m[i] = clamp(-k × dB[i]/dt, -m_max[i], +m_max[i])
 *
 * The adaptive gain k = BDOT_GAIN_COEFF / ΔT is derived from orbit-averaged
 * discrete stability analysis. At high ω the law saturates (bang-bang
 * equivalent); at low ω it provides smooth proportional convergence.
 *
 * MATLAB reference: ref/PoCat-Lektron-ADCS/ADCS/Detumbling.m
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
 * @brief Select the adaptive control period for detumbling.
 *
 * Two-tier scheme (see docs/adcs_detumble_high_rate_analysis.md §7.7):
 *   |ω| > DETUMBLE_OMEGA_SWITCH → ΔT_fast (10 Hz, reduces phase lag)
 *   |ω| ≤ DETUMBLE_OMEGA_SWITCH → ΔT_slow (1 Hz, maintains dB/dt SNR)
 *
 * @param[in] state  ADCS state (reads gyro angular velocity)
 * @return Recommended control period ΔT [s]
 */
double detumble_select_dt(const adcs_state_t *state);

/**
 * @brief Execute one detumbling control step.
 *
 * Proportional B-DOT with per-axis saturation:
 *   1. Compute dB/dt = (B_body - B_body_prev) / dT
 *   2. k = BDOT_GAIN_COEFF / dT  (adaptive gain)
 *   3. m_raw[i] = -k * dB_dt[i]; m[i] = clamp(m_raw, ±m_max)
 *   4. Quantize intensity via mtq_compute_command()
 *   5. Check angular velocity threshold for exit condition
 *
 * @param[in,out] state  ADCS state (reads mag/gyro, writes mtq_cmd)
 * @return 1 if detumbling is complete (ω below threshold), 0 otherwise
 */
int detumble_step(adcs_state_t *state);

#ifdef __cplusplus
}
#endif

#endif /* ADCS_DETUMBLE_H */
