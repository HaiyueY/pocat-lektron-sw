/**
 * @file adcs_time.h
 * @brief Time-scale conversions used by the ADCS orbit/environment models.
 *
 * All functions take UTC as seconds since 1970-01-01 00:00:00 (Unix time,
 * no leap seconds) as a double, so that sub-second resolution can be added
 * later without changing the interface. UT1 is approximated by UTC
 * (|UT1 − UTC| < 0.9 s → < 0.004° Earth-rotation error).
 *
 * The ADCS does not judge whether the system time is valid; that is the
 * responsibility of OBC/OBDH. These helpers only convert.
 */

#ifndef ADCS_TIME_H
#define ADCS_TIME_H

#ifdef __cplusplus
extern "C" {
#endif

/** Seconds per day */
#define ADCS_SECONDS_PER_DAY    86400.0

/** Julian Date of the Unix epoch (1970-01-01 00:00:00 UTC) */
#define ADCS_JD_UNIX_EPOCH      2440587.5

/** Julian Date of J2000.0 (2000-01-01 12:00:00) */
#define ADCS_JD_J2000           2451545.0

/** Unix time of J2000.0 (2000-01-01 12:00:00 UTC) [s] */
#define ADCS_UNIX_J2000         946728000.0

/**
 * @brief Convert Unix time to Julian Date (UTC).
 * @param[in] unix_s  Seconds since 1970-01-01 00:00:00 UTC
 * @return Julian Date [days]
 */
double adcs_time_unix_to_jd(double unix_s);

/**
 * @brief Convert Julian Date (UTC) to Unix time.
 * @param[in] jd  Julian Date [days]
 * @return Seconds since 1970-01-01 00:00:00 UTC
 */
double adcs_time_jd_to_unix(double jd);

/**
 * @brief Convert a proleptic-Gregorian calendar date/time to Unix time.
 *
 * Valid for any year representable in an int; years before 1970 give
 * negative results.
 *
 * @param[in] year    Full year (e.g. 2027)
 * @param[in] month   1–12
 * @param[in] day     1–31
 * @param[in] hour    0–23
 * @param[in] minute  0–59
 * @param[in] sec     Seconds (may be fractional)
 * @return Seconds since 1970-01-01 00:00:00 UTC
 */
double adcs_time_from_calendar(int year, int month, int day,
                               int hour, int minute, double sec);

/**
 * @brief Convert Unix time to a decimal year (e.g. 2027.5), as used by IGRF.
 *
 * The fraction is the elapsed part of the calendar year, so 365- and
 * 366-day years are handled exactly.
 *
 * @param[in] unix_s  Seconds since 1970-01-01 00:00:00 UTC
 * @return Decimal year
 */
double adcs_time_unix_to_decimal_year(double unix_s);

/**
 * @brief Convert a TLE epoch to Unix time.
 *
 * Two-digit years follow the NORAD convention: 57–99 → 1957–1999,
 * 00–56 → 2000–2056. Day 1.0 is January 1st, 00:00 UTC.
 *
 * @param[in] yy           Two-digit epoch year (0–99)
 * @param[in] day_of_year  Fractional day of year (1.0 ≤ day < 367.0)
 * @return Seconds since 1970-01-01 00:00:00 UTC
 */
double adcs_time_tle_epoch_to_unix(int yy, double day_of_year);

/**
 * @brief Greenwich Mean Sidereal Time (IAU-82, Vallado `gstime`).
 *
 * This is the angle that rotates TEME into the pseudo-Earth-fixed frame:
 * r_ecef = R3(gmst) · r_teme (polar motion neglected).
 *
 * @param[in] unix_s  Seconds since 1970-01-01 00:00:00 UTC (≈ UT1)
 * @return GMST [rad], in [0, 2π)
 */
double adcs_time_gmst(double unix_s);

#ifdef __cplusplus
}
#endif

#endif /* ADCS_TIME_H */
