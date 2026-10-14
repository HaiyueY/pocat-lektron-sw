/**
 * @file sim_environment.h
 * @brief Simulated orbital environment for ADCS testing.
 *
 * Provides orbital state, magnetic field in ECI, sun direction,
 * nadir direction, and eclipse status for closed-loop ADCS testing
 * without hardware.
 *
 * Two environments:
 *   - sim_env_init():       legacy reference case — 400 km circular polar
 *                           orbit, tilted dipole, Sun fixed along +X. Kept
 *                           unchanged so the S1 regression results stay
 *                           comparable.
 *   - sim_env_init_ephem(): "truth" for testing the on-board models — SGP4
 *                           orbit from a TLE, full IGRF-14 (or another
 *                           degree), moving Sun, cylindrical shadow, all in
 *                           TEME at real UTC dates. The controller is then
 *                           fed by adcs_ephem_update() with the on-board
 *                           degree, so the closed loop sees the model error.
 */

#ifndef SIM_ENVIRONMENT_H
#define SIM_ENVIRONMENT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "adcs_types.h"
#include "adcs_ephemeris.h"

/** PoCat design orbit: 500 km SSO, LTDN 10:30, epoch 2027-09-15 00:00 UTC (synthetic) */
#define SIM_DESIGN_TLE1 \
    "1 99999U 27001A   27258.00000000  .00002000  00000-0  10000-3 0  9995"
#define SIM_DESIGN_TLE2 \
    "2 99999  97.4000 330.0000 0010000  90.0000 270.0000 15.21937171    10"

/** Truth-environment configuration for sim_env_init_ephem() */
typedef struct {
    const char *tle1;        /**< TLE line 1 of the truth orbit */
    const char *tle2;        /**< TLE line 2 of the truth orbit */
    double start_unix;       /**< Simulation start time [s since 1970, UTC] */
    int truth_igrf_order;    /**< IGRF degree of the truth field (normally 13) */
} sim_env_ephem_cfg_t;

/** Orbital environment state */
typedef struct {
    vec3d_t pos_eci;         /**< Satellite position in ECI [m] */
    vec3d_t vel_eci;         /**< Satellite velocity in ECI [m/s] */
    vec3d_t b_field_eci;     /**< Geomagnetic field in ECI [T] */
    vec3d_t sun_eci;         /**< Sun direction in ECI (unit) */
    vec3d_t nadir_eci;       /**< Nadir direction in ECI (unit) */
    int     eclipse;         /**< 1 if in eclipse */
    double  time_s;          /**< Elapsed time [s] */

    /* Truth-environment mode (sim_env_init_ephem) */
    int     ephem_mode;      /**< 0: legacy model, 1: TLE/IGRF/Sun truth */
    double  start_unix;      /**< Simulation start [s since 1970, UTC] */
    double  unix_s;          /**< Current UTC [s since 1970] (ephem mode) */
    int     truth_igrf_order;/**< IGRF degree used for the truth field */
    adcs_orbit_t orbit;      /**< Truth orbit source */
} sim_env_t;

/**
 * @brief Initialize environment with a polar low-Earth orbit.
 * @param[out] env  Environment state to initialize
 */
void sim_env_init(sim_env_t *env);

/**
 * @brief Initialize the truth environment from a TLE at a UTC date.
 *
 * Truth and on-board models share the SGP4 orbit, the Sun ephemeris and
 * the shadow model; they differ in the IGRF degree. Orbit, Sun and shadow
 * model errors are quantified separately (docs/adcs_orbit_env_models.md).
 *
 * @param[out] env  Environment state to initialize
 * @param[in]  cfg  Truth configuration
 * @return 0 on success, -1 if the TLE is rejected or the models fail
 */
int sim_env_init_ephem(sim_env_t *env, const sim_env_ephem_cfg_t *cfg);

/**
 * @brief Advance the orbital environment by one timestep.
 *
 * Legacy mode: simplified circular orbit with a tilted dipole and a fixed
 * Sun. Ephem mode: SGP4/IGRF/Sun evaluated at start_unix + time_s.
 *
 * @param[in,out] env  Environment state
 * @param[in]     dt   Timestep [s]
 */
void sim_env_step(sim_env_t *env, double dt);

#ifdef __cplusplus
}
#endif

#endif /* SIM_ENVIRONMENT_H */
