/**
 * @file sim_gyroscope.h
 * @brief Simulated IIM-42652 gyroscope for ADCS testing.
 */

#ifndef SIM_GYROSCOPE_H
#define SIM_GYROSCOPE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "adcs_types.h"

/** Gyroscope simulator state (bias drift) */
typedef struct {
    vec3d_t bias;   /**< Current gyro bias [rad/s] */
} sim_gyro_state_t;

/**
 * @brief Initialize gyroscope simulator.
 * @param[out] gs  Gyroscope simulator state
 */
void sim_gyro_init(sim_gyro_state_t *gs);

/**
 * @brief Generate a simulated gyroscope reading.
 *
 * Adds Angle Random Walk (ARW) noise and slowly drifting bias
 * matching IIM-42652 specs.
 *
 * @param[in,out] gs       Gyroscope simulator state (updated bias)
 * @param[in]     omega    True angular velocity [rad/s]
 * @param[in]     dt       Timestep [s]
 * @param[out]    gyro_out Gyroscope reading with noise [rad/s]
 */
void sim_gyro_read(sim_gyro_state_t *gs, vec3d_t omega, double dt,
                   gyro_data_t *gyro_out);

#ifdef __cplusplus
}
#endif

#endif /* SIM_GYROSCOPE_H */
