/**
 * @file adcs_detumble.h
 * @brief B-DOT detumbling controller.
 *
 * Implements the sign-based B-DOT law for satellite detumbling after
 * deployment or perturbation. Uses the maximum magnetic moment and the
 * sign of the magnetic field derivative to generate control commands.
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
 * @brief Execute one detumbling control step.
 *
 * Algorithm (from Detumbling.m L119-184):
 *   1. Compute dB/dt = (B_body - B_body_prev) / dT
 *   2. moment[i] = -max_moment[i] * sign(dB_dt[i])
 *   3. Clamp and quantize intensity via mtq_compute_command()
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
