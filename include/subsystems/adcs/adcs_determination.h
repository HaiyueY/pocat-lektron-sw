/**
 * @file adcs_determination.h
 * @brief Attitude determination using TRIAD algorithm.
 *
 * Implements the TRIAD algorithm for estimating the satellite rotation
 * matrix from two pairs of reference/body vectors (Sun and magnetic field).
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
 * @brief Sun direction and visibility from the six CSS voltages.
 *
 * Sun direction = voltage-weighted sum of the face normals (+X, −X, +Y,
 * −Y, +Z, −Z), normalised. Sets sun_visible only if the reading is valid,
 * the brightest face exceeds ADCS_CSS_SUN_THRESHOLD_V and the sum is
 * non-zero; otherwise sun_body is zero.
 *
 * @param[in,out] sun  Reads voltage[] and valid, writes sun_body and sun_visible
 */
void determination_css_sun(sun_sensor_data_t *sun);

/**
 * @brief Sun/magnetometer TRIAD attitude fix.
 *
 * Uses (Sun_ECI, B_ECI) and (Sun_body, B_body). Call only while
 * state->sun.sun_visible is set; in eclipse the caller propagates the
 * previous estimate with the gyro.
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
