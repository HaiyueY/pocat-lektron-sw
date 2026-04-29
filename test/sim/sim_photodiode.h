/**
 * @file sim_photodiode.h
 * @brief Simulated SLCD-61N8 photodiode sun sensor array.
 */

#ifndef SIM_PHOTODIODE_H
#define SIM_PHOTODIODE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "adcs_types.h"
#include "sim_environment.h"

/**
 * @brief Generate simulated photodiode readings.
 *
 * Models 6 face-mounted photodiodes with cosine response.
 * Computes sun direction in body frame from photodiode outputs.
 *
 * @param[in]  env       Orbital environment (sun direction, eclipse)
 * @param[in]  q_true    True attitude quaternion (ECI → Body)
 * @param[out] sun_out   Sun sensor data with body-frame sun vector
 */
void sim_photodiode_read(const sim_env_t *env, quat_t q_true,
                         sun_sensor_data_t *sun_out);

#ifdef __cplusplus
}
#endif

#endif /* SIM_PHOTODIODE_H */
