/**
 * @file adcs_determination.h
 * @brief Attitude determination using TRIAD algorithm.
 *
 * Implements the TRIAD algorithm for estimating the satellite rotation
 * matrix from two pairs of reference/body vectors. Supports both
 * sun-visible and eclipse modes as described in the subsystem documentation.
 *
 * MATLAB reference: ref/H-bridge-simulations/Simulation Nadir Pointing/
 *                   Nadir_pointing.m L109-147
 */

#ifndef ADCS_DETERMINATION_H
#define ADCS_DETERMINATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "adcs_types.h"

/**
 * @brief TRIAD attitude determination algorithm.
 *
 * Estimates rotation matrix from two vector pairs (reference frame / body frame).
 * The first vector pair is assumed more accurate than the second.
 *
 * @param[in]  ref1   First reference vector (ECI frame, more accurate)
 * @param[in]  ref2   Second reference vector (ECI frame)
 * @param[in]  body1  First body vector (body frame, more accurate)
 * @param[in]  body2  Second body vector (body frame)
 * @param[out] rot    Estimated rotation matrix (ECI → Body)
 */
void triad_compute(vec3d_t ref1, vec3d_t ref2,
                   vec3d_t body1, vec3d_t body2,
                   mat3d_t *rot);

/**
 * @brief Full attitude determination pipeline.
 *
 * Selects vector pairs based on sun visibility:
 *   - Sun visible: uses (B_ECI, Sun_ECI) and (B_body, Sun_body)
 *   - Eclipse: uses (B_ECI, dB_ECI/dt) and (B_body, dB_body/dt + ω×B_body)
 *
 * Updates state->q_eci_body with the estimated quaternion.
 *
 * @param[in,out] state  ADCS state (reads sensor data, writes q_eci_body)
 */
void determination_update(adcs_state_t *state);

#ifdef __cplusplus
}
#endif

#endif /* ADCS_DETERMINATION_H */
