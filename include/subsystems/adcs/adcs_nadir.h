/**
 * @file adcs_nadir.h
 * @brief Nadir pointing controller using magnetic control law.
 *
 * Computes the required magnetic dipole moment to align the satellite's
 * +Y body axis with the nadir direction (pointing toward Earth).
 * Uses the magnetic control law with PID-derived gains.
 *
 * MATLAB reference: ref/H-bridge-simulations/Simulation Nadir Pointing/
 *                   Nadir_pointing.m L152-238
 */

#ifndef ADCS_NADIR_H
#define ADCS_NADIR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "adcs_types.h"

/**
 * @brief Initialize nadir pointing mode state.
 *
 * Resets controller internal state.
 *
 * @param[in,out] state  ADCS state to initialize for nadir pointing
 */
void nadir_init(adcs_state_t *state);

/**
 * @brief Execute one nadir pointing control step.
 *
 * Algorithm (from Nadir_pointing.m L152-238):
 *   1. Run attitude determination (TRIAD)
 *   2. Compute target quaternion: q_target = U2Q(nadir_eci, body_axis)
 *   3. Compute error quaternion: q_err = conj(q_est) * q_target
 *   4. Extract ε (vector part of error quaternion)
 *   5. Compute moment: m = (kP * cross(B, ε) - kR * cross(B, ω)) / ||B||
 *   6. Clamp and quantize via mtq_compute_command()
 *
 * @param[in,out] state  ADCS state (reads sensors/ECI refs, writes mtq_cmd)
 */
void nadir_step(adcs_state_t *state);

#ifdef __cplusplus
}
#endif

#endif /* ADCS_NADIR_H */
