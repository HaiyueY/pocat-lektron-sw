/**
 * @file sim_environment.h
 * @brief Simulated orbital environment for ADCS testing.
 *
 * Provides orbital state, magnetic field in ECI, sun direction,
 * nadir direction, and eclipse status for closed-loop ADCS testing
 * without hardware.
 */

#ifndef SIM_ENVIRONMENT_H
#define SIM_ENVIRONMENT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "adcs_types.h"

/** Orbital environment state */
typedef struct {
    vec3d_t pos_eci;         /**< Satellite position in ECI [m] */
    vec3d_t vel_eci;         /**< Satellite velocity in ECI [m/s] */
    vec3d_t b_field_eci;     /**< Geomagnetic field in ECI [T] */
    vec3d_t sun_eci;         /**< Sun direction in ECI (unit) */
    vec3d_t nadir_eci;       /**< Nadir direction in ECI (unit) */
    int     eclipse;         /**< 1 if in eclipse */
    double  time_s;          /**< Elapsed time [s] */
} sim_env_t;

/**
 * @brief Initialize environment with a polar low-Earth orbit.
 * @param[out] env  Environment state to initialize
 */
void sim_env_init(sim_env_t *env);

/**
 * @brief Advance the orbital environment by one timestep.
 *
 * Uses a simplified circular orbit model with time-varying
 * magnetic field and sun direction.
 *
 * @param[in,out] env  Environment state
 * @param[in]     dt   Timestep [s]
 */
void sim_env_step(sim_env_t *env, double dt);

#ifdef __cplusplus
}
#endif

#endif /* SIM_ENVIRONMENT_H */
