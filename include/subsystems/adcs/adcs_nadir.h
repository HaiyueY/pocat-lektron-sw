/**
 * @file adcs_nadir.h
 * @brief Nadir pointing controller using magnetic control law.
 *
 * Computes the required magnetic dipole moment to align the satellite's
 * +Z body axis (NADIR_BODY_AXIS_*) with the nadir direction (pointing
 * toward Earth).
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
 * Algorithm (adapted from Nadir_pointing.m L152-238):
 *   1. Attitude determination (TRIAD when sunlit, gyro propagation in eclipse)
 *   2. Reduced-attitude error ε = ẑ_body × nadir_body (2-DOF, yaw-invariant)
 *   3. Compute moment: m = (kP * cross(B, ε) - kR * cross(B, ω̃)) / ||B||,
 *      with ω̃ = ω_gyro - bias_est - ω_orbit (orbit-rate-compensated damping)
 *   4. Clamp and quantize via mtq_compute_command()
 *
 * @param[in,out] state  ADCS state (reads sensors/ECI refs, writes mtq_cmd)
 */
void nadir_step(adcs_state_t *state);

#ifdef __cplusplus
}
#endif

#endif /* ADCS_NADIR_H */
