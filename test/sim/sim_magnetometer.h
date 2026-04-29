/**
 * @file sim_magnetometer.h
 * @brief Simulated MMC5983MA magnetometer for ADCS testing.
 */

#ifndef SIM_MAGNETOMETER_H
#define SIM_MAGNETOMETER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "adcs_types.h"
#include "sim_environment.h"

/**
 * @brief Generate a simulated magnetometer reading.
 *
 * Transforms the ECI magnetic field to body frame using the true
 * satellite attitude, then adds Gaussian noise matching the
 * MMC5983MA sensor specs (±0.4 mG RMS).
 *
 * @param[in]  env       Orbital environment with B-field in ECI
 * @param[in]  q_true    True satellite attitude quaternion (ECI → Body)
 * @param[out] mag_out   Magnetometer reading in body frame [T]
 */
void sim_mag_read(const sim_env_t *env, quat_t q_true, mag_data_t *mag_out);

#ifdef __cplusplus
}
#endif

#endif /* SIM_MAGNETOMETER_H */
