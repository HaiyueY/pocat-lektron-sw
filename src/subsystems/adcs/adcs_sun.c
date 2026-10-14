/**
 * @file adcs_sun.c
 * @brief Low-precision solar ephemeris and cylindrical Earth-shadow model.
 *
 * Sun: Vallado, "Fundamentals of Astrodynamics and Applications",
 * Algorithm 29 (Astronomical Almanac). TT is approximated by UTC (69 s
 * offset → 0.0008° of solar longitude). Only the direction is returned, so
 * the distance term is not computed.
 */

#include "adcs_sun.h"
#include "adcs_time.h"
#include "adcs_math.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

vec3d_t adcs_sun_direction(double unix_s)
{
    const double deg2rad = M_PI / 180.0;

    /* Julian centuries since J2000 */
    double t = (unix_s - ADCS_UNIX_J2000) / (ADCS_SECONDS_PER_DAY * 36525.0);

    /* Mean longitude and mean anomaly [deg] */
    double mean_long = fmod(280.460 + 36000.771285 * t, 360.0);
    double mean_anom = fmod(357.5291092 + 35999.05034 * t, 360.0) * deg2rad;

    /* Ecliptic longitude and obliquity [rad] */
    double ecl_long = (mean_long
                       + 1.914666471 * sin(mean_anom)
                       + 0.019994643 * sin(2.0 * mean_anom)) * deg2rad;
    double obliquity = (23.439291 - 0.0130042 * t) * deg2rad;

    double cos_l = cos(ecl_long);
    double sin_l = sin(ecl_long);

    return vec3d_make(cos_l,
                      cos(obliquity) * sin_l,
                      sin(obliquity) * sin_l);
}

uint8_t adcs_eclipse_cylindrical(vec3d_t r_eci_m, vec3d_t sun_dir)
{
    double along = vec3d_dot(r_eci_m, sun_dir);
    if (along >= 0.0) {
        return 0;
    }
    /* Distance from the Earth–Sun line */
    vec3d_t perp = vec3d_sub(r_eci_m, vec3d_scale(sun_dir, along));
    return (vec3d_dot(perp, perp) <
            ADCS_SHADOW_EARTH_RADIUS_M * ADCS_SHADOW_EARTH_RADIUS_M) ? 1 : 0;
}
