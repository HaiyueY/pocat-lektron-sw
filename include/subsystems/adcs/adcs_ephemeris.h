/**
 * @file adcs_ephemeris.h
 * @brief On-board reference vectors: orbit, geomagnetic field and Sun.
 *
 * Combines SGP4 (adcs_sgp4), IGRF-14 (adcs_igrf) and the solar ephemeris
 * (adcs_sun) into the inertial reference vectors used by attitude
 * determination and control. Everything is expressed in TEME:
 *
 *   - position/velocity come out of SGP4 in TEME;
 *   - the field is evaluated in the pseudo-Earth-fixed frame
 *     r_ecef = R3(GMST) · r_teme and rotated back with R3(−GMST);
 *   - the Sun direction (mean equator of date) is used as TEME (< 0.005°).
 *
 * The *_eci fields of adcs_state_t therefore hold TEME vectors. Because all
 * references share that frame, the TEME↔J2000 offset (≈ 0.3° of
 * precession) does not affect the attitude solution.
 *
 * This layer only reports status; the policy (e.g. leave NADIR on a stale
 * TLE) belongs to the ADCS task (S4a). It does not judge whether the system
 * time is valid — that is OBC/OBDH's responsibility.
 */

#ifndef ADCS_EPHEMERIS_H
#define ADCS_EPHEMERIS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "adcs_types.h"
#include "adcs_tle.h"
#include "adcs_sgp4.h"

/* ---- adcs_ephem_t.status bits ---- */
#define ADCS_EPH_OK                 0x00u
#define ADCS_EPH_NO_TLE             0x01u  /**< No element set loaded: outputs invalid */
#define ADCS_EPH_SGP4_ERROR         0x02u  /**< SGP4 failed (see sgp4_error): outputs invalid */
#define ADCS_EPH_TLE_STALE          0x04u  /**< |TLE age| > ADCS_TLE_MAX_AGE_S: degraded */
#define ADCS_EPH_IGRF_EXTRAPOLATED  0x08u  /**< Date outside IGRF-14 2025–2030: degraded */
#define ADCS_EPH_IGRF_ERROR         0x10u  /**< Field could not be computed: outputs invalid */

/** Status bits that make the reference vectors unusable */
#define ADCS_EPH_INVALID_MASK \
    (ADCS_EPH_NO_TLE | ADCS_EPH_SGP4_ERROR | ADCS_EPH_IGRF_ERROR)

/** adcs_orbit_load_tle() return codes */
typedef enum {
    ADCS_ORBIT_OK         =  0,
    ADCS_ORBIT_ERR_NULL   = -1, /**< NULL argument */
    ADCS_ORBIT_ERR_PARSE  = -2, /**< adcs_tle_parse() rejected the lines */
    ADCS_ORBIT_ERR_INIT   = -3, /**< adcs_sgp4_init() rejected the elements (e.g. deep space) */
    ADCS_ORBIT_ERR_EPOCH  = -4  /**< Trial propagation at epoch failed */
} adcs_orbit_status_t;

/** Orbit source: the element set in use and its SGP4 record */
typedef struct {
    adcs_tle_t  tle;
    adcs_sgp4_t sgp4;
    uint8_t     loaded;   /**< 1 once a TLE has been accepted */
} adcs_orbit_t;

/** One evaluation of the on-board reference models */
typedef struct {
    double  unix_s;        /**< Evaluation time [s since 1970, UTC] */
    double  tle_age_s;     /**< unix_s − TLE epoch (negative before epoch) [s] */
    vec3d_t pos_eci;       /**< Satellite position, TEME [m] */
    vec3d_t vel_eci;       /**< Satellite velocity, TEME [m/s] */
    vec3d_t nadir_eci;     /**< Unit nadir direction (−r̂), TEME */
    vec3d_t b_field_eci;   /**< Geomagnetic field, TEME [T] */
    vec3d_t sun_eci;       /**< Unit Sun direction, TEME */
    uint8_t eclipse;       /**< 1 if in the cylindrical Earth shadow */
    uint8_t status;        /**< ADCS_EPH_* bits */
    int8_t  sgp4_error;    /**< adcs_sgp4_status_t of the last propagation */
} adcs_ephem_t;

/**
 * @brief Initialise an orbit source with no TLE loaded.
 * @param[out] orbit  Orbit source
 */
void adcs_orbit_init(adcs_orbit_t *orbit);

/**
 * @brief Parse, initialise and trial-propagate a TLE, then commit it.
 *
 * The current element set is replaced only if every step succeeds, so a
 * rejected upload leaves the previous TLE in use. Uses ≈ 0.4 KB of the
 * caller's stack for the staging copy.
 *
 * @param[in,out] orbit  Orbit source
 * @param[in]     line1  TLE line 1
 * @param[in]     line2  TLE line 2
 * @return ADCS_ORBIT_OK or a negative adcs_orbit_status_t code
 */
int adcs_orbit_load_tle(adcs_orbit_t *orbit, const char *line1, const char *line2);

/**
 * @brief Evaluate the reference models at a given time.
 *
 * @param[in]  orbit       Orbit source
 * @param[in]  unix_s      Evaluation time [s since 1970, UTC]
 * @param[in]  igrf_order  IGRF truncation degree (normally ADCS_IGRF_ORDER)
 * @param[out] eph         Result; vectors are meaningful only if
 *                         (status & ADCS_EPH_INVALID_MASK) == 0
 * @return eph->status
 */
uint8_t adcs_ephem_update(const adcs_orbit_t *orbit, double unix_s,
                          int igrf_order, adcs_ephem_t *eph);

/**
 * @brief Copy valid reference vectors into the ADCS state.
 *
 * Writes pos_eci, vel_eci, nadir_eci, b_field_eci and sun_eci. Leaves the
 * state untouched if the ephemeris is invalid.
 *
 * @param[in]     eph    Result of adcs_ephem_update()
 * @param[in,out] state  ADCS state
 * @return 0 if copied, -1 if the ephemeris was invalid
 */
int adcs_ephem_to_state(const adcs_ephem_t *eph, adcs_state_t *state);

#ifdef __cplusplus
}
#endif

#endif /* ADCS_EPHEMERIS_H */
