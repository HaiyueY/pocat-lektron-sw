/**
 * @file test_ephemeris.c
 * @brief Golden-vector regression for the S3 orbit/environment models.
 *
 * Usage: test_ephemeris <data_dir>
 *
 * Checks, against independent references (see data/README.md):
 *   1. Time scales and GMST           (closed-form values, Vallado Ex. 3-5)
 *   2. TLE parser                     (field values, error paths)
 *   3. SGP4                           (Vallado SGP4-VER.TLE / tcppver.out)
 *   4. IGRF-14                        (ppigrf at degrees 13 / 8 / 1)
 *   5. Sun direction and shadow       (astropy, Vallado Ex. 5-1)
 *   6. Ephemeris layer                (status bits, TLE commit rules, sanity)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "adcs_types.h"
#include "adcs_config.h"
#include "adcs_math.h"
#include "adcs_time.h"
#include "adcs_tle.h"
#include "adcs_sgp4.h"
#include "adcs_igrf.h"
#include "adcs_sun.h"
#include "adcs_ephemeris.h"

/* ---- Acceptance thresholds ------------------------------------------- */

/** SGP4 vs Vallado C++ (S3 exit criterion: < 1 m) */
#define SGP4_POS_TOL_KM     1.0e-3
#define SGP4_VEL_TOL_KMS    1.0e-6

/** IGRF vs ppigrf: float coefficient storage only [nT] */
#define IGRF_TOL_NT         0.01

/** Sun vs astropy: Almanac model accuracy (~0.01°) + MOD/TEME (<0.005°) */
#define SUN_TOL_DEG         0.02

/** PoCat design orbit: 500 km SSO, LTDN 10:30, epoch 2027-09-15 (synthetic) */
static const char design_tle1[] =
    "1 99999U 27001A   27258.00000000  .00002000  00000-0  10000-3 0  9995";
static const char design_tle2[] =
    "2 99999  97.4000 330.0000 0010000  90.0000 270.0000 15.21937171    10";

/** Vallado verification case 00005 (TEME example) */
static const char tle5_1[] =
    "1 00005U 58002B   00179.78495062  .00000023  00000-0  28098-4 0  4753";
static const char tle5_2[] =
    "2 00005  34.2682 348.7242 1859667 331.7664  19.3264 10.82419157413667";

static int test_count = 0;
static int pass_count = 0;

static void check(const char *desc, int condition)
{
    test_count++;
    if (condition) {
        printf("  PASS: %s\n", desc);
        pass_count++;
    } else {
        printf("  FAIL: %s\n", desc);
    }
}

static int near(double a, double b, double tol)
{
    return fabs(a - b) <= tol;
}

static double angle_deg(vec3d_t a, vec3d_t b)
{
    double c = vec3d_dot(vec3d_normalize(a), vec3d_normalize(b));
    if (c > 1.0) c = 1.0;
    if (c < -1.0) c = -1.0;
    return acos(c) * RAD_TO_DEG;
}

static FILE *open_data(const char *dir, const char *name)
{
    char path[1024];
    snprintf(path, sizeof(path), "%s/%s", dir, name);
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        printf("  ERROR: cannot open %s\n", path);
    }
    return f;
}

/** Strip trailing CR/LF in place */
static void chomp(char *s)
{
    s[strcspn(s, "\r\n")] = '\0';
}

/** Rewrite column 69 with the correct checksum */
static void fix_checksum(char *line)
{
    line[68] = (char)('0' + adcs_tle_checksum(line));
}

/* ======================================================================
 * 1. Time
 * ==================================================================== */

static void test_time(void)
{
    printf("\n[1] Time scales\n");

    check("J2000.0: Unix 946728000 -> JD 2451545.0",
          adcs_time_unix_to_jd(946728000.0) == 2451545.0);
    check("JD -> Unix round trip (2027-09-15)",
          near(adcs_time_jd_to_unix(adcs_time_unix_to_jd(1820966400.25)),
               1820966400.25, 1.0e-4));

    check("calendar 1970-01-01 = 0",
          adcs_time_from_calendar(1970, 1, 1, 0, 0, 0.0) == 0.0);
    check("calendar 2000-01-01 12:00 = 946728000",
          adcs_time_from_calendar(2000, 1, 1, 12, 0, 0.0) == 946728000.0);
    check("calendar 2024-02-29 (leap day) = 1709164800",
          adcs_time_from_calendar(2024, 2, 29, 0, 0, 0.0) == 1709164800.0);
    check("calendar 1957-01-01 = -410227200",
          adcs_time_from_calendar(1957, 1, 1, 0, 0, 0.0) == -410227200.0);

    check("decimal year 2025-01-01 = 2025.0",
          adcs_time_unix_to_decimal_year(adcs_time_from_calendar(2025, 1, 1, 0, 0, 0)) == 2025.0);
    check("decimal year 2024-07-01 = 2024 + 182/366",
          near(adcs_time_unix_to_decimal_year(1719792000.0), 2024.0 + 182.0 / 366.0, 1e-12));
    check("decimal year 2028-12-31 12:00 = 2028 + 365.5/366",
          near(adcs_time_unix_to_decimal_year(1861876800.0), 2028.0 + 365.5 / 366.0, 1e-12));

    /* 00005 epoch printed by tcppver: 2000-06-27 18:50:19.733568 */
    check("TLE epoch 00179.78495062 -> 2000-06-27 18:50:19.733568",
          near(adcs_time_tle_epoch_to_unix(0, 179.78495062), 962131819.733568, 1.0e-5));
    check("TLE epoch yy=57 -> 1957, yy=56 -> 2056",
          adcs_time_tle_epoch_to_unix(57, 1.0) == -410227200.0 &&
          adcs_time_tle_epoch_to_unix(56, 1.0) == 2713910400.0);

    /* Vallado Example 3-5: 1992-08-20 12:14 UT1 -> GMST 152.578787810° */
    double gmst = adcs_time_gmst(adcs_time_from_calendar(1992, 8, 20, 12, 14, 0.0)) * RAD_TO_DEG;
    printf("    GMST Ex. 3-5: %.9f deg (ref 152.578787810)\n", gmst);
    check("GMST Vallado Ex. 3-5 within 1e-6 deg", near(gmst, 152.578787810, 1.0e-6));
}

/* ======================================================================
 * 2. TLE parser
 * ==================================================================== */

static void test_tle(void)
{
    adcs_tle_t t;
    char l1[80], l2[80];

    printf("\n[2] TLE parser\n");

    int rc = adcs_tle_parse(tle5_1, tle5_2, &t);
    check("00005 parses", rc == ADCS_TLE_OK);
    check("00005 fields (satnum, epoch, ndot, nddot, bstar)",
          t.satnum == 5 && t.epoch_yy == 0 && t.epoch_day == 179.78495062 &&
          t.ndot == 0.00000023 && t.nddot == 0.0 && near(t.bstar, 0.28098e-4, 1e-18));
    check("00005 fields (i, raan, e, argp, M, n)",
          t.incl_deg == 34.2682 && t.raan_deg == 348.7242 && t.ecc == 0.1859667 &&
          t.argp_deg == 331.7664 && t.mean_anom_deg == 19.3264 &&
          t.mean_motion == 10.82419157);

    /* Negative mantissa/exponent fields (21897: bstar -13525-3, 16925: nddot -30915-6) */
    rc = adcs_tle_parse(
        "1 21897U 92011A   06176.02341244 -.00001273  00000-0 -13525-3 0  3044",
        "2 21897  62.1749 198.0096 7421690 253.0462  20.1561  2.01269994104880", &t);
    check("21897 bstar = -0.13525e-3, ndot = -0.00001273",
          rc == ADCS_TLE_OK && near(t.bstar, -0.13525e-3, 1e-18) && t.ndot == -0.00001273);
    rc = adcs_tle_parse(
        "1 16925U 86065D   06151.67415771  .02550794 -30915-6  18784-3 0  4486",
        "2 16925  62.0906 295.0239 5596327 245.1593  47.9690  4.88511875148616", &t);
    check("16925 nddot = -0.30915e-6", rc == ADCS_TLE_OK && near(t.nddot, -0.30915e-6, 1e-20));

    /* Error paths: output must stay untouched */
    adcs_tle_t before;
    adcs_tle_parse(tle5_1, tle5_2, &before);
    t = before;

    strcpy(l1, tle5_1);
    l1[30] = (l1[30] == '9') ? '8' : '9';                     /* corrupt epoch digit */
    check("corrupted digit -> ERR_CHECKSUM", adcs_tle_parse(l1, tle5_2, &t) == ADCS_TLE_ERR_CHECKSUM);

    strcpy(l2, tle5_2);
    l2[0] = '3';
    check("wrong line number -> ERR_LINE_NUMBER", adcs_tle_parse(tle5_1, l2, &t) == ADCS_TLE_ERR_LINE_NUMBER);

    strcpy(l1, tle5_1);
    l1[60] = '\0';
    check("short line -> ERR_LENGTH", adcs_tle_parse(l1, tle5_2, &t) == ADCS_TLE_ERR_LENGTH);

    strcpy(l2, tle5_2);
    l2[6] = '6';
    fix_checksum(l2);
    check("satnum mismatch -> ERR_SATNUM", adcs_tle_parse(tle5_1, l2, &t) == ADCS_TLE_ERR_SATNUM);

    strcpy(l2, tle5_2);
    l2[10] = 'x';
    fix_checksum(l2);
    check("letter in inclination -> ERR_FORMAT", adcs_tle_parse(tle5_1, l2, &t) == ADCS_TLE_ERR_FORMAT);

    strcpy(l2, tle5_2);
    memcpy(&l2[52], " 0.00000000", 11);
    fix_checksum(l2);
    check("zero mean motion -> ERR_RANGE", adcs_tle_parse(tle5_1, l2, &t) == ADCS_TLE_ERR_RANGE);

    check("NULL -> ERR_NULL", adcs_tle_parse(NULL, tle5_2, &t) == ADCS_TLE_ERR_NULL);
    check("output untouched after errors", memcmp(&t, &before, sizeof(t)) == 0);
}

/* ======================================================================
 * 3. SGP4 vs Vallado verification output
 * ==================================================================== */

#define MAX_VER_LINES  1000

static void test_sgp4(const char *dir)
{
    static char ver[MAX_VER_LINES][320];
    int nver = 0;
    char l1[256], l2[256];
    int near_earth = 0, deep = 0, lines = 0, mismatches = 0, bad_cksum = 0;
    int cksum_sats_ok = 1;
    double max_dr = 0.0, max_dv = 0.0;

    printf("\n[3] SGP4 vs Vallado SGP4-VER.TLE / tcppver.out\n");

    FILE *fv = open_data(dir, "sgp4_tcppver.txt");
    FILE *ft = open_data(dir, "SGP4-VER.TLE");
    if (fv == NULL || ft == NULL) {
        check("SGP4 verification files readable", 0);
        if (fv) fclose(fv);
        if (ft) fclose(ft);
        return;
    }
    while (nver < MAX_VER_LINES && fgets(ver[nver], sizeof(ver[0]), fv)) {
        chomp(ver[nver]);
        nver++;
    }
    fclose(fv);

    while (fgets(l1, sizeof(l1), ft)) {
        if (l1[0] != '1') {
            continue;
        }
        if (!fgets(l2, sizeof(l2), ft)) {
            break;
        }
        chomp(l1);
        chomp(l2);

        /* Parser must flag the deliberately bad checksums (33333–33335) */
        adcs_tle_t t;
        if (adcs_tle_parse(l1, l2, &t) == ADCS_TLE_ERR_CHECKSUM) {
            bad_cksum++;
            if (strncmp(&l1[2], "3333", 4) != 0) {
                cksum_sats_ok = 0;
            }
        }
        fix_checksum(l1);
        fix_checksum(l2);
        if (adcs_tle_parse(l1, l2, &t) != ADCS_TLE_OK) {
            printf("    parse failed: %s\n", l1);
            mismatches++;
            continue;
        }

        adcs_sgp4_t rec;
        int rc = adcs_sgp4_init(&t, &rec);
        if (rc == ADCS_SGP4_ERR_DEEP_SPACE) {
            deep++;
            continue;
        }
        near_earth++;

        /* Locate this satellite's block ("<satnum> xx"); 20413 appears twice
         * but is deep space, so near-Earth headers are unique. */
        char hdr[16];
        snprintf(hdr, sizeof(hdr), "%u xx", (unsigned)t.satnum);
        int k = -1;
        for (int i = 0; i < nver; i++) {
            if (strcmp(ver[i], hdr) == 0) {
                k = i + 1;
                break;
            }
        }
        if (k < 0) {
            printf("    %u: no block in tcppver\n", (unsigned)t.satnum);
            mismatches++;
            continue;
        }

        /* Schedule exactly as Vallado's test driver: t = 0, then
         * start..stop by step, then stop itself if it was skipped. */
        double tstart, tstop, tstep;
        if (sscanf(&l2[69], "%lf %lf %lf", &tstart, &tstop, &tstep) != 3) {
            mismatches++;
            continue;
        }
        double times[512];
        int nt = 0;
        times[nt++] = 0.0;
        double ts = tstart;
        while (ts <= tstop && nt < 511) {
            if (!(ts == tstart && tstart == 0.0)) {
                times[nt++] = ts;
            }
            ts += tstep;
        }
        if (ts - tstop < tstep - 1.0e-6) {
            times[nt++] = tstop;
        }

        int stop_err = 0;
        int produced = 0;
        for (int i = 0; i < nt; i++) {
            double r[3], v[3], e[7];
            int err = adcs_sgp4_propagate(&rec, times[i], r, v);
            if (err != ADCS_SGP4_OK && i > 0) {
                stop_err = err;
                break;
            }
            if (k >= nver || strstr(ver[k], "xx") != NULL ||
                sscanf(ver[k], "%lf %lf %lf %lf %lf %lf %lf",
                       &e[0], &e[1], &e[2], &e[3], &e[4], &e[5], &e[6]) != 7 ||
                fabs(e[0] - times[i]) > 1.0e-6) {
                printf("    %u: output diverges from tcppver at t=%.4f\n",
                       (unsigned)t.satnum, times[i]);
                mismatches++;
                break;
            }
            double dr = sqrt((r[0] - e[1]) * (r[0] - e[1]) + (r[1] - e[2]) * (r[1] - e[2]) +
                             (r[2] - e[3]) * (r[2] - e[3]));
            double dv = sqrt((v[0] - e[4]) * (v[0] - e[4]) + (v[1] - e[5]) * (v[1] - e[5]) +
                             (v[2] - e[6]) * (v[2] - e[6]));
            if (dr > max_dr) max_dr = dr;
            if (dv > max_dv) max_dv = dv;
            k++;
            produced++;
            lines++;
        }
        /* The reference must stop exactly where we stop (error) or end */
        int ref_has_more = (k < nver && strstr(ver[k], "xx") == NULL && ver[k][0] != '\0');
        if (ref_has_more) {
            printf("    %u: tcppver continues after our stop (err %d)\n",
                   (unsigned)t.satnum, stop_err);
            mismatches++;
        }
        printf("    %5u: %2d epochs%s", (unsigned)t.satnum, produced,
               stop_err ? "" : "\n");
        if (stop_err) {
            printf(", stops with error %d (as reference)\n", stop_err);
        }
    }
    fclose(ft);

    printf("    near-Earth sets %d, deep-space rejected %d, epochs %d\n", near_earth, deep, lines);
    printf("    max |dr| = %.3e km, max |dv| = %.3e km/s\n", max_dr, max_dv);

    check("parser flags exactly the 3 bad-checksum sets (33333-33335)",
          bad_cksum == 3 && cksum_sats_ok);
    check("all 9 near-Earth verification sets run, 24 deep-space rejected",
          near_earth == 9 && deep == 24);
    check("every epoch and every error stop matches tcppver", mismatches == 0 && lines == 158);
    check("position within 1 m of Vallado", max_dr < SGP4_POS_TOL_KM);
    check("velocity within 1 mm/s of Vallado", max_dv < SGP4_VEL_TOL_KMS);
}

/* ======================================================================
 * 4. IGRF-14
 * ==================================================================== */

static void test_igrf(const char *dir)
{
    double worst[ADCS_IGRF_NMAX + 1] = {0};
    int rows = 0, errors = 0;
    char line[512];

    printf("\n[4] IGRF-14 vs ppigrf\n");

    FILE *f = open_data(dir, "igrf14_golden.csv");
    if (f == NULL) {
        check("IGRF golden file readable", 0);
        return;
    }
    if (!fgets(line, sizeof(line), f)) {
        fclose(f);
        check("IGRF golden file has a header", 0);
        return;
    }
    while (fgets(line, sizeof(line), f)) {
        double year, r, th, ph, br, bt, bp;
        int n_max;
        if (sscanf(line, "%lf,%d,%lf,%lf,%lf,%lf,%lf,%lf",
                   &year, &n_max, &r, &th, &ph, &br, &bt, &bp) != 8) {
            continue;
        }
        double t = th * DEG_TO_RAD, p = ph * DEG_TO_RAD;
        vec3d_t r_hat = {sin(t) * cos(p), sin(t) * sin(p), cos(t)};
        vec3d_t t_hat = {cos(t) * cos(p), cos(t) * sin(p), -sin(t)};
        vec3d_t p_hat = {-sin(p), cos(p), 0.0};
        vec3d_t b;
        if (adcs_igrf_field_ecef(vec3d_scale(r_hat, r * 1.0e3), year, n_max, &b) < 0) {
            errors++;
            continue;
        }
        b = vec3d_scale(b, 1.0e9);
        double d = fmax(fabs(vec3d_dot(b, r_hat) - br),
                   fmax(fabs(vec3d_dot(b, t_hat) - bt), fabs(vec3d_dot(b, p_hat) - bp)));
        if (n_max >= 1 && n_max <= ADCS_IGRF_NMAX && d > worst[n_max]) {
            worst[n_max] = d;
        }
        rows++;
    }
    fclose(f);

    for (int n = 1; n <= ADCS_IGRF_NMAX; n++) {
        if (worst[n] > 0.0) {
            printf("    degree %2d: worst component difference %.6f nT\n", n, worst[n]);
        }
    }
    check("3240 golden points evaluated without error", rows == 3240 && errors == 0);
    check("degree 13 within 0.01 nT", worst[13] < IGRF_TOL_NT);
    check("degree 8 truncation within 0.01 nT", worst[8] < IGRF_TOL_NT);
    check("degree 1 (tilted dipole) within 0.01 nT", worst[1] < IGRF_TOL_NT);

    /* Exact pole: finite and continuous with a point 1e-7 rad away */
    const double r_m = 6871.2e3;
    vec3d_t b_pole, b_near;
    adcs_igrf_field_ecef(vec3d_make(0.0, 0.0, r_m), 2027.5, 13, &b_pole);
    adcs_igrf_field_ecef(vec3d_make(r_m * 1.0e-7, 0.0, r_m), 2027.5, 13, &b_near);
    check("exact north pole finite and continuous (< 0.1 nT)",
          isfinite(b_pole.x) && isfinite(b_pole.y) && isfinite(b_pole.z) &&
          vec3d_norm(vec3d_sub(b_pole, b_near)) * 1.0e9 < 0.1);

    vec3d_t b;
    vec3d_t r = vec3d_make(r_m, 0.0, 0.0);
    check("2027.5 -> ADCS_IGRF_OK", adcs_igrf_field_ecef(r, 2027.5, 13, &b) == ADCS_IGRF_OK);
    check("2024.9 and 2030.1 -> ADCS_IGRF_EXTRAPOLATED",
          adcs_igrf_field_ecef(r, 2024.9, 13, &b) == ADCS_IGRF_EXTRAPOLATED &&
          adcs_igrf_field_ecef(r, 2030.1, 13, &b) == ADCS_IGRF_EXTRAPOLATED);
    check("degree 0 / 14, r = 0, NULL -> ADCS_IGRF_ERR_INVALID",
          adcs_igrf_field_ecef(r, 2027.5, 0, &b) == ADCS_IGRF_ERR_INVALID &&
          adcs_igrf_field_ecef(r, 2027.5, 14, &b) == ADCS_IGRF_ERR_INVALID &&
          adcs_igrf_field_ecef(vec3d_zero(), 2027.5, 13, &b) == ADCS_IGRF_ERR_INVALID &&
          adcs_igrf_field_ecef(r, 2027.5, 13, NULL) == ADCS_IGRF_ERR_INVALID);
}

/* ======================================================================
 * 5. Sun and shadow
 * ==================================================================== */

static void test_sun(const char *dir)
{
    double worst = 0.0;
    int rows = 0;
    char line[256];

    printf("\n[5] Sun direction and shadow\n");

    FILE *f = open_data(dir, "sun_golden.csv");
    if (f == NULL) {
        check("Sun golden file readable", 0);
        return;
    }
    if (!fgets(line, sizeof(line), f)) {
        fclose(f);
        check("Sun golden file has a header", 0);
        return;
    }
    while (fgets(line, sizeof(line), f)) {
        double t;
        vec3d_t ref;
        if (sscanf(line, "%lf,%lf,%lf,%lf", &t, &ref.x, &ref.y, &ref.z) != 4) {
            continue;
        }
        double a = angle_deg(adcs_sun_direction(t), ref);
        if (a > worst) worst = a;
        rows++;
    }
    fclose(f);
    printf("    %d points 2025-2029, worst %.5f deg\n", rows, worst);
    check("Sun within 0.02 deg of astropy (TEME)", rows == 183 && worst < SUN_TOL_DEG);

    /* Vallado Example 5-1: 2006-04-02 00:00 UTC, r_sun = (0.9771945, 0.1924424, 0.0834308) AU */
    vec3d_t s = adcs_sun_direction(adcs_time_from_calendar(2006, 4, 2, 0, 0, 0.0));
    double a = angle_deg(s, vec3d_make(0.9771945, 0.1924424, 0.0834308));
    printf("    Vallado Ex. 5-1: %.6f deg\n", a);
    check("Sun Vallado Ex. 5-1 within 0.001 deg", a < 0.001);
    check("Sun direction is a unit vector", near(vec3d_norm(s), 1.0, 1e-12));

    /* Cylindrical shadow geometry */
    vec3d_t u = vec3d_make(1.0, 0.0, 0.0);
    check("night side on the Earth-Sun line -> shadow",
          adcs_eclipse_cylindrical(vec3d_make(-7.0e6, 0.0, 0.0), u) == 1);
    check("day side -> sunlit",
          adcs_eclipse_cylindrical(vec3d_make(7.0e6, 0.0, 0.0), u) == 0);
    check("night side, 6000 km off the line -> shadow",
          adcs_eclipse_cylindrical(vec3d_make(-3.0e6, 6.0e6, 0.0), u) == 1);
    check("night side, 6500 km off the line -> sunlit",
          adcs_eclipse_cylindrical(vec3d_make(-3.0e6, 0.0, 6.5e6), u) == 0);
}

/* ======================================================================
 * 6. Ephemeris layer
 * ==================================================================== */

static void test_ephemeris(void)
{
    adcs_orbit_t orbit;
    adcs_ephem_t eph;
    adcs_state_t state = {0};

    printf("\n[6] Ephemeris layer\n");

    adcs_orbit_init(&orbit);
    check("no TLE -> ADCS_EPH_NO_TLE",
          adcs_ephem_update(&orbit, 1.8e9, ADCS_IGRF_ORDER, &eph) == ADCS_EPH_NO_TLE);
    state.pos_eci = vec3d_make(1.0, 2.0, 3.0);
    check("invalid ephemeris is not copied to the state",
          adcs_ephem_to_state(&eph, &state) == -1 && state.pos_eci.x == 1.0);

    check("bad checksum -> ADCS_ORBIT_ERR_PARSE, nothing loaded",
          adcs_orbit_load_tle(&orbit, "1 00005U 58002B   00179.78495062  .00000023  00000-0  28098-4 0  4750",
                              tle5_2) == ADCS_ORBIT_ERR_PARSE && !orbit.loaded);
    check("deep-space TLE (08195) -> ADCS_ORBIT_ERR_INIT",
          adcs_orbit_load_tle(&orbit,
              "1 08195U 75081A   06176.33215444  .00000099  00000-0  11873-3 0   813",
              "2 08195  64.1586 279.0717 6877146 264.7651  20.2257  2.00491383225656")
          == ADCS_ORBIT_ERR_INIT && !orbit.loaded);
    check("design TLE loads", adcs_orbit_load_tle(&orbit, design_tle1, design_tle2) == ADCS_ORBIT_OK &&
                              orbit.loaded && orbit.tle.satnum == 99999);
    check("design TLE epoch = 2027-09-15 00:00 UTC", orbit.tle.epoch_unix == 1820966400.0);
    check("a rejected TLE keeps the loaded one",
          adcs_orbit_load_tle(&orbit, tle5_1, "2 00005  garbage") == ADCS_ORBIT_ERR_PARSE &&
          orbit.tle.satnum == 99999);

    /* One orbit along the design orbit */
    const double t0 = orbit.tle.epoch_unix;
    const double period = 86400.0 / orbit.tle.mean_motion;
    int all_ok = 1, nadir_ok = 1, alt_ok = 1, b_ok = 1, sun_ok = 1;
    int shadow = 0, samples = 0;
    double b_min = 1.0, b_max = 0.0;
    for (double dt = 0.0; dt < period; dt += 10.0) {
        uint8_t st = adcs_ephem_update(&orbit, t0 + 86400.0 + dt, ADCS_IGRF_ORDER, &eph);
        if (st != ADCS_EPH_OK) all_ok = 0;
        double r = vec3d_norm(eph.pos_eci);
        double alt = (r - 6378137.0) * 1.0e-3;
        if (alt < 470.0 || alt > 540.0) alt_ok = 0;
        if (!near(vec3d_dot(eph.nadir_eci, vec3d_scale(eph.pos_eci, 1.0 / r)), -1.0, 1e-12)) nadir_ok = 0;
        double bn = vec3d_norm(eph.b_field_eci);
        if (bn < b_min) b_min = bn;
        if (bn > b_max) b_max = bn;
        if (bn < 15.0e-6 || bn > 60.0e-6) b_ok = 0;
        if (!near(vec3d_norm(eph.sun_eci), 1.0, 1e-12)) sun_ok = 0;
        shadow += eph.eclipse;
        samples++;
    }
    printf("    period %.1f min, |B| %.1f-%.1f uT, shadow %.1f%% of the orbit\n",
           period / 60.0, b_min * 1e6, b_max * 1e6, 100.0 * shadow / samples);
    check("status OK along the orbit (2027, TLE age 1 day)", all_ok);
    check("altitude 470-540 km", alt_ok);
    check("nadir = -r_hat", nadir_ok);
    check("|B| within 15-60 uT", b_ok);
    check("Sun direction unit length", sun_ok);
    check("shadow fraction 25-42% (LTDN 10:30 in September)",
          shadow > 0.25 * samples && shadow < 0.42 * samples);

    /* Frame round trip: B in TEME equals R3(-gmst) * IGRF(R3(gmst) r) */
    double t1 = t0 + 3600.0;
    adcs_ephem_update(&orbit, t1, 13, &eph);
    double g = adcs_time_gmst(t1);
    vec3d_t b_ecef;
    adcs_igrf_field_ecef(vec3d_rot_z(eph.pos_eci, g), adcs_time_unix_to_decimal_year(t1), 13, &b_ecef);
    check("B_teme = R3(-GMST) IGRF(R3(GMST) r_teme)",
          vec3d_norm(vec3d_sub(vec3d_rot_z(b_ecef, -g), eph.b_field_eci)) < 1.0e-15);

    /* TLE age reporting */
    adcs_ephem_update(&orbit, t0 + 15.0 * 86400.0, ADCS_IGRF_ORDER, &eph);
    check("15 d after epoch -> TLE_STALE (outputs still valid)",
          eph.status == ADCS_EPH_TLE_STALE && near(eph.tle_age_s, 15.0 * 86400.0, 1e-6));
    adcs_ephem_update(&orbit, t0 - 15.0 * 86400.0, ADCS_IGRF_ORDER, &eph);
    check("15 d before epoch -> TLE_STALE", eph.status == ADCS_EPH_TLE_STALE && eph.tle_age_s < 0.0);

    /* Old element set: IGRF outside its window, TLE stale */
    adcs_orbit_t old;
    adcs_orbit_init(&old);
    adcs_orbit_load_tle(&old, tle5_1, tle5_2);
    adcs_ephem_update(&old, old.tle.epoch_unix + 360.0 * 60.0, ADCS_IGRF_ORDER, &eph);
    check("year-2000 TLE -> IGRF_EXTRAPOLATED only",
          eph.status == ADCS_EPH_IGRF_EXTRAPOLATED);
    check("00005 at 360 min matches tcppver (-7154.031, -3783.177, -3536.194 km)",
          near(eph.pos_eci.x, -7154031.20202, 1e-3) && near(eph.pos_eci.y, -3783176.82504, 1e-3) &&
          near(eph.pos_eci.z, -3536194.12294, 1e-3));

    /* Copy to state */
    adcs_ephem_update(&orbit, t0, ADCS_IGRF_ORDER, &eph);
    check("valid ephemeris copied to the state",
          adcs_ephem_to_state(&eph, &state) == 0 &&
          state.pos_eci.x == eph.pos_eci.x && state.b_field_eci.z == eph.b_field_eci.z &&
          state.sun_eci.y == eph.sun_eci.y && state.nadir_eci.x == eph.nadir_eci.x &&
          state.vel_eci.z == eph.vel_eci.z);
}

int main(int argc, char **argv)
{
    if (argc != 2) {
        fprintf(stderr, "usage: %s <data_dir>\n", argv[0]);
        return 2;
    }

    printf("=== ADCS Orbit/Environment Model Test ===\n");

    test_time();
    test_tle();
    test_sgp4(argv[1]);
    test_igrf(argv[1]);
    test_sun(argv[1]);
    test_ephemeris();

    printf("\n=== Results: %d/%d passed ===\n", pass_count, test_count);
    return (pass_count == test_count) ? 0 : 1;
}
