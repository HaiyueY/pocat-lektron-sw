/**
 * @file adcs_time.c
 * @brief Time-scale conversions used by the ADCS orbit/environment models.
 *
 * Calendar arithmetic uses the days-from-civil algorithm (H. Hinnant,
 * "chrono-Compatible Low-Level Date Algorithms"), which is exact for the
 * proleptic Gregorian calendar and needs no tables.
 *
 * GMST follows Vallado, "Fundamentals of Astrodynamics and Applications",
 * Algorithm 15 / `gstime` in the AIAA-2006-6753 SGP4 package.
 */

#include "adcs_time.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * @brief Days since 1970-01-01 for a proleptic-Gregorian date.
 */
static long days_from_civil(long y, int m, int d)
{
    y -= (m <= 2) ? 1 : 0;
    const long era = (y >= 0 ? y : y - 399) / 400;
    const long yoe = y - era * 400;                                  /* [0, 399]   */
    const long doy = (153L * (m > 2 ? m - 3 : m + 9) + 2) / 5 + d - 1; /* [0, 365] */
    const long doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;          /* [0, 146096] */
    return era * 146097L + doe - 719468L;
}

/**
 * @brief Calendar year containing a given day count since 1970-01-01.
 */
static long year_from_days(long z)
{
    z += 719468L;
    const long era = (z >= 0 ? z : z - 146096L) / 146097L;
    const long doe = z - era * 146097L;
    const long yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    const long doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    const long mp  = (5 * doy + 2) / 153;
    const long m   = mp < 10 ? mp + 3 : mp - 9;
    return yoe + era * 400 + (m <= 2 ? 1 : 0);
}

double adcs_time_unix_to_jd(double unix_s)
{
    return unix_s / ADCS_SECONDS_PER_DAY + ADCS_JD_UNIX_EPOCH;
}

double adcs_time_jd_to_unix(double jd)
{
    return (jd - ADCS_JD_UNIX_EPOCH) * ADCS_SECONDS_PER_DAY;
}

double adcs_time_from_calendar(int year, int month, int day,
                               int hour, int minute, double sec)
{
    long days = days_from_civil(year, month, day);
    return (double)days * ADCS_SECONDS_PER_DAY
         + (double)hour * 3600.0 + (double)minute * 60.0 + sec;
}

double adcs_time_unix_to_decimal_year(double unix_s)
{
    long year = year_from_days((long)floor(unix_s / ADCS_SECONDS_PER_DAY));
    double start = (double)days_from_civil(year, 1, 1) * ADCS_SECONDS_PER_DAY;
    double end   = (double)days_from_civil(year + 1, 1, 1) * ADCS_SECONDS_PER_DAY;
    return (double)year + (unix_s - start) / (end - start);
}

double adcs_time_tle_epoch_to_unix(int yy, double day_of_year)
{
    int year = (yy < 57) ? 2000 + yy : 1900 + yy;
    return (double)days_from_civil(year, 1, 1) * ADCS_SECONDS_PER_DAY
         + (day_of_year - 1.0) * ADCS_SECONDS_PER_DAY;
}

double adcs_time_gmst(double unix_s)
{
    const double twopi = 2.0 * M_PI;

    /* Julian centuries of UT1 since J2000, computed from the Unix offset
     * rather than from a JD to avoid losing ~1e-10 day of resolution. */
    double tut1 = (unix_s - ADCS_UNIX_J2000) / (ADCS_SECONDS_PER_DAY * 36525.0);

    /* GMST in seconds of time (IAU-82) */
    double temp = -6.2e-6 * tut1 * tut1 * tut1
                + 0.093104 * tut1 * tut1
                + (876600.0 * 3600.0 + 8640184.812866) * tut1
                + 67310.54841;

    /* 360°/86400 s = 1/240 °/s, then to rad */
    temp = fmod(temp * (M_PI / 180.0) / 240.0, twopi);
    if (temp < 0.0) {
        temp += twopi;
    }
    return temp;
}
