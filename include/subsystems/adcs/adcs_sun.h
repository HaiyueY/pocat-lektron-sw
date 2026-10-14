/**
 * @file adcs_sun.h
 * @brief Low-precision solar ephemeris and cylindrical Earth-shadow model.
 *
 * Sun direction from the Astronomical Almanac low-precision formulae
 * (Vallado Algorithm 29, ≈ 0.01° over 1950–2050). The result is expressed
 * in the mean-equator-of-date frame, which differs from TEME by less than
 * 0.005°; it is used directly as the TEME sun direction.
 */

#ifndef ADCS_SUN_H
#define ADCS_SUN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "adcs_types.h"

/** Earth equatorial radius used for the shadow cylinder [m] */
#define ADCS_SHADOW_EARTH_RADIUS_M  6378137.0

/**
 * @brief Geocentric unit vector towards the Sun.
 *
 * @param[in] unix_s  Seconds since 1970-01-01 00:00:00 UTC
 * @return Unit Sun direction (mean equator of date ≈ TEME)
 */
vec3d_t adcs_sun_direction(double unix_s);

/**
 * @brief Cylindrical Earth-shadow test.
 *
 * The satellite is in shadow when it is on the night side (r · ŝ < 0) and
 * within one Earth radius of the Earth–Sun line. Penumbra is ignored
 * (it lasts ≈ 8 s per pass in LEO).
 *
 * @param[in] r_eci_m  Satellite position (same inertial frame as @p sun_dir) [m]
 * @param[in] sun_dir  Unit Sun direction
 * @return 1 if in shadow, 0 if sunlit
 */
uint8_t adcs_eclipse_cylindrical(vec3d_t r_eci_m, vec3d_t sun_dir);

#ifdef __cplusplus
}
#endif

#endif /* ADCS_SUN_H */
