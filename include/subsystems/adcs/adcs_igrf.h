/**
 * @file adcs_igrf.h
 * @brief IGRF-14 geomagnetic field model with selectable truncation degree.
 *
 * Main field at epoch 2025.0 (degree ≤ 13) plus linear secular variation
 * (degree ≤ 8), valid 2025.0–2030.0. The field is synthesised in geocentric
 * spherical coordinates and returned as a Cartesian vector in the same
 * Earth-fixed frame as the input position.
 *
 * Cost grows with (n_max + 1)(n_max + 2)/2 terms. The on-board degree is
 * ADCS_IGRF_ORDER in adcs_config.h (chosen by the S3 trade study, see
 * docs/adcs_orbit_env_models.md).
 */

#ifndef ADCS_IGRF_H
#define ADCS_IGRF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "adcs_types.h"

/** Highest degree available in the coefficient table */
#define ADCS_IGRF_NMAX              13

/** IGRF reference radius [km] */
#define ADCS_IGRF_REF_RADIUS_KM     6371.2

/** IGRF return codes */
typedef enum {
    ADCS_IGRF_OK           =  0,  /**< Date inside 2025.0–2030.0 */
    ADCS_IGRF_EXTRAPOLATED =  1,  /**< Computed, but date outside 2025.0–2030.0 */
    ADCS_IGRF_ERR_INVALID  = -1   /**< NULL output, degree outside 1–13 or |r| < 1000 km */
} adcs_igrf_status_t;

/**
 * @brief Geomagnetic field vector at an Earth-fixed position.
 *
 * @param[in]  r_ecef_m      Position in an Earth-fixed frame (ECEF/PEF) [m]
 * @param[in]  decimal_year  Date as decimal year (see adcs_time_unix_to_decimal_year())
 * @param[in]  n_max         Truncation degree, 1 (tilted dipole) … 13
 * @param[out] b_ecef_t      Field in the same Earth-fixed frame [T]
 * @return ADCS_IGRF_OK, ADCS_IGRF_EXTRAPOLATED or ADCS_IGRF_ERR_INVALID
 *         (on error @p b_ecef_t is not written)
 */
int adcs_igrf_field_ecef(vec3d_t r_ecef_m, double decimal_year, int n_max,
                         vec3d_t *b_ecef_t);

#ifdef __cplusplus
}
#endif

#endif /* ADCS_IGRF_H */
