/**
 * @file adcs_tle.h
 * @brief Two-Line Element set parser.
 *
 * Fixed-column parser for the NORAD TLE format. It does not use strtod or
 * the C locale, checks both line checksums, the line numbers and that the
 * satellite numbers match. Elements are returned in TLE units; conversion
 * to SGP4 internal units is done by adcs_sgp4_init().
 *
 * Intended to be reused by the TLE upload telecommand (S5): parse first,
 * then load with adcs_orbit_load_tle() which also trial-propagates.
 */

#ifndef ADCS_TLE_H
#define ADCS_TLE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** Number of significant characters in a TLE line (columns 1–69) */
#define ADCS_TLE_LINE_LEN   69

/** TLE parser return codes */
typedef enum {
    ADCS_TLE_OK              =  0,
    ADCS_TLE_ERR_NULL        = -1, /**< NULL pointer argument */
    ADCS_TLE_ERR_LENGTH      = -2, /**< Line shorter than 69 characters */
    ADCS_TLE_ERR_LINE_NUMBER = -3, /**< Line does not start with "1 " / "2 " */
    ADCS_TLE_ERR_CHECKSUM    = -4, /**< Column-69 checksum mismatch */
    ADCS_TLE_ERR_SATNUM      = -5, /**< Satellite numbers of the two lines differ */
    ADCS_TLE_ERR_FORMAT      = -6, /**< Unexpected character in a numeric field */
    ADCS_TLE_ERR_RANGE       = -7  /**< Element outside its physical range */
} adcs_tle_status_t;

/** Mean elements of one TLE, in TLE units */
typedef struct {
    uint32_t satnum;          /**< NORAD catalogue number */
    int      epoch_yy;        /**< Two-digit epoch year */
    double   epoch_day;       /**< Fractional day of year (1.0 = Jan 1 00:00 UTC) */
    double   epoch_unix;      /**< Epoch as Unix time [s] */
    double   ndot;            /**< First derivative of mean motion / 2 [rev/day²] */
    double   nddot;           /**< Second derivative of mean motion / 6 [rev/day³] */
    double   bstar;           /**< B* drag term [1/earth radii] */
    double   incl_deg;        /**< Inclination [deg] */
    double   raan_deg;        /**< Right ascension of ascending node [deg] */
    double   ecc;             /**< Eccentricity [-] */
    double   argp_deg;        /**< Argument of perigee [deg] */
    double   mean_anom_deg;   /**< Mean anomaly [deg] */
    double   mean_motion;     /**< Mean motion (Kozai) [rev/day] */
} adcs_tle_t;

/**
 * @brief Compute the TLE checksum of a line.
 *
 * Sum of all digits in columns 1–68, with '-' counting as 1, modulo 10.
 *
 * @param[in] line  TLE line (at least 68 characters)
 * @return Checksum digit 0–9, or -1 if the line is NULL or too short
 */
int adcs_tle_checksum(const char *line);

/**
 * @brief Parse a two-line element set.
 *
 * Characters beyond column 69 are ignored, so lines may carry a trailing
 * newline or extra columns. On error @p tle is left unchanged.
 *
 * @param[in]  line1  First TLE line
 * @param[in]  line2  Second TLE line
 * @param[out] tle    Parsed elements
 * @return ADCS_TLE_OK or a negative adcs_tle_status_t code
 */
int adcs_tle_parse(const char *line1, const char *line2, adcs_tle_t *tle);

#ifdef __cplusplus
}
#endif

#endif /* ADCS_TLE_H */
