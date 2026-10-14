/**
 * @file adcs_tle.c
 * @brief Two-Line Element set parser.
 *
 * Column layout (1-based, see https://celestrak.org/columns/v04n03/):
 *
 *   Line 1: 1 satnum(3-7) class(8) intl-des(10-17) epoch-yy(19-20)
 *           epoch-day(21-32) ndot(34-43) nddot(45-52) bstar(54-61)
 *           eph-type(63) elset-no(65-68) checksum(69)
 *   Line 2: 2 satnum(3-7) incl(9-16) raan(18-25) ecc(27-33, implied "0.")
 *           argp(35-42) M(44-51) n(53-63) rev-no(64-68) checksum(69)
 *
 * nddot and bstar use the "±ddddd±e" implied-decimal exponent notation:
 * " 28098-4" = +0.28098e-4. Blank digits are read as zero, matching the
 * input clean-up in Vallado's twoline2rv().
 *
 * Numeric fields are accumulated as integers and scaled once by an exact
 * power of ten, which gives the correctly rounded value for the ≤ 16
 * significant digits a TLE field can hold.
 */

#include "adcs_tle.h"
#include "adcs_time.h"

/** Exact powers of ten (all representable in IEEE-754 double) */
static const double pow10_table[] = {
    1e0, 1e1, 1e2, 1e3, 1e4, 1e5, 1e6, 1e7, 1e8,
    1e9, 1e10, 1e11, 1e12, 1e13, 1e14, 1e15, 1e16
};

#define POW10_MAX   ((int)(sizeof(pow10_table) / sizeof(pow10_table[0])) - 1)

static int is_digit(char c)
{
    return c >= '0' && c <= '9';
}

/**
 * @brief Check that columns 1–69 are present (no terminator or newline).
 */
static int line_has_min_length(const char *line)
{
    for (int i = 0; i < ADCS_TLE_LINE_LEN; i++) {
        if (line[i] == '\0' || line[i] == '\n' || line[i] == '\r') {
            return 0;
        }
    }
    return 1;
}

/**
 * @brief Parse an unsigned integer field with optional leading blanks.
 */
static int parse_uint(const char *s, int len, uint32_t *out)
{
    int i = 0;
    uint32_t value = 0;
    int digits = 0;

    while (i < len && s[i] == ' ') {
        i++;
    }
    for (; i < len; i++) {
        if (!is_digit(s[i])) {
            return -1;
        }
        value = value * 10u + (uint32_t)(s[i] - '0');
        digits++;
    }
    if (digits == 0) {
        return -1;
    }
    *out = value;
    return 0;
}

/**
 * @brief Parse a signed decimal field such as "  34.2682" or "-.00000084".
 *
 * Leading and trailing blanks are allowed; the decimal point is optional.
 */
static int parse_decimal(const char *s, int len, double *out)
{
    int i = 0;
    int negative = 0;
    int seen_dot = 0;
    int digits = 0;
    int frac_digits = 0;
    double mantissa = 0.0;

    while (i < len && s[i] == ' ') {
        i++;
    }
    if (i < len && (s[i] == '-' || s[i] == '+')) {
        negative = (s[i] == '-');
        i++;
    }
    for (; i < len && s[i] != ' '; i++) {
        if (is_digit(s[i])) {
            mantissa = mantissa * 10.0 + (double)(s[i] - '0');
            digits++;
            if (seen_dot) {
                frac_digits++;
            }
        } else if (s[i] == '.' && !seen_dot) {
            seen_dot = 1;
        } else {
            return -1;
        }
    }
    for (; i < len; i++) {
        if (s[i] != ' ') {
            return -1;
        }
    }
    if (digits == 0 || frac_digits > POW10_MAX) {
        return -1;
    }

    double value = mantissa / pow10_table[frac_digits];
    *out = negative ? -value : value;
    return 0;
}

/**
 * @brief Parse an implied-decimal exponent field "±ddddd±e" (8 columns).
 */
static int parse_exp_field(const char *s, double *out)
{
    int negative;
    int exp_negative;
    double mantissa = 0.0;

    if (s[0] == '-') {
        negative = 1;
    } else if (s[0] == ' ' || s[0] == '+') {
        negative = 0;
    } else {
        return -1;
    }

    for (int i = 1; i <= 5; i++) {
        int d;
        if (s[i] == ' ') {
            d = 0;
        } else if (is_digit(s[i])) {
            d = s[i] - '0';
        } else {
            return -1;
        }
        mantissa = mantissa * 10.0 + (double)d;
    }

    if (s[6] == '-') {
        exp_negative = 1;
    } else if (s[6] == '+' || s[6] == ' ' || s[6] == '0') {
        exp_negative = 0;
    } else {
        return -1;
    }

    int e;
    if (s[7] == ' ') {
        e = 0;
    } else if (is_digit(s[7])) {
        e = s[7] - '0';
    } else {
        return -1;
    }

    /* value = 0.ddddd × 10^(±e) = ddddd × 10^(±e − 5) */
    int p = (exp_negative ? -e : e) - 5;
    double value = (p >= 0) ? mantissa * pow10_table[p]
                            : mantissa / pow10_table[-p];
    *out = negative ? -value : value;
    return 0;
}

/**
 * @brief Parse the 7-digit eccentricity field with implied leading "0.".
 */
static int parse_ecc_field(const char *s, double *out)
{
    double mantissa = 0.0;

    for (int i = 0; i < 7; i++) {
        int d;
        if (s[i] == ' ') {
            d = 0;
        } else if (is_digit(s[i])) {
            d = s[i] - '0';
        } else {
            return -1;
        }
        mantissa = mantissa * 10.0 + (double)d;
    }
    *out = mantissa / pow10_table[7];
    return 0;
}

int adcs_tle_checksum(const char *line)
{
    if (line == 0) {
        return -1;
    }

    int sum = 0;
    for (int i = 0; i < ADCS_TLE_LINE_LEN - 1; i++) {
        char c = line[i];
        if (c == '\0' || c == '\n' || c == '\r') {
            return -1;
        }
        if (is_digit(c)) {
            sum += c - '0';
        } else if (c == '-') {
            sum += 1;
        }
    }
    return sum % 10;
}

int adcs_tle_parse(const char *line1, const char *line2, adcs_tle_t *tle)
{
    adcs_tle_t t;
    uint32_t satnum2;
    uint32_t yy;

    if (line1 == 0 || line2 == 0 || tle == 0) {
        return ADCS_TLE_ERR_NULL;
    }
    if (!line_has_min_length(line1) || !line_has_min_length(line2)) {
        return ADCS_TLE_ERR_LENGTH;
    }
    if (line1[0] != '1' || line1[1] != ' ' || line2[0] != '2' || line2[1] != ' ') {
        return ADCS_TLE_ERR_LINE_NUMBER;
    }
    if (!is_digit(line1[68]) || adcs_tle_checksum(line1) != line1[68] - '0' ||
        !is_digit(line2[68]) || adcs_tle_checksum(line2) != line2[68] - '0') {
        return ADCS_TLE_ERR_CHECKSUM;
    }

    /* ---- Line 1 ---- */
    if (parse_uint(&line1[2], 5, &t.satnum) != 0 ||
        parse_uint(&line1[18], 2, &yy) != 0 ||
        parse_decimal(&line1[20], 12, &t.epoch_day) != 0 ||
        parse_decimal(&line1[33], 10, &t.ndot) != 0 ||
        parse_exp_field(&line1[44], &t.nddot) != 0 ||
        parse_exp_field(&line1[53], &t.bstar) != 0) {
        return ADCS_TLE_ERR_FORMAT;
    }

    /* ---- Line 2 ---- */
    if (parse_uint(&line2[2], 5, &satnum2) != 0 ||
        parse_decimal(&line2[8], 8, &t.incl_deg) != 0 ||
        parse_decimal(&line2[17], 8, &t.raan_deg) != 0 ||
        parse_ecc_field(&line2[26], &t.ecc) != 0 ||
        parse_decimal(&line2[34], 8, &t.argp_deg) != 0 ||
        parse_decimal(&line2[43], 8, &t.mean_anom_deg) != 0 ||
        parse_decimal(&line2[52], 11, &t.mean_motion) != 0) {
        return ADCS_TLE_ERR_FORMAT;
    }

    if (satnum2 != t.satnum) {
        return ADCS_TLE_ERR_SATNUM;
    }

    if (t.epoch_day < 1.0 || t.epoch_day >= 367.0 ||
        t.incl_deg < 0.0 || t.incl_deg > 180.0 ||
        t.raan_deg < 0.0 || t.raan_deg >= 360.0 ||
        t.argp_deg < 0.0 || t.argp_deg >= 360.0 ||
        t.mean_anom_deg < 0.0 || t.mean_anom_deg >= 360.0 ||
        t.ecc >= 1.0 || t.mean_motion <= 0.0) {
        return ADCS_TLE_ERR_RANGE;
    }

    t.epoch_yy = (int)yy;
    t.epoch_unix = adcs_time_tle_epoch_to_unix(t.epoch_yy, t.epoch_day);

    *tle = t;
    return ADCS_TLE_OK;
}
