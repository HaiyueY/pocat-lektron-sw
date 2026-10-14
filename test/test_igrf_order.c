/**
 * @file test_igrf_order.c
 * @brief IGRF truncation-degree trade study, open loop.
 *
 * Usage: ./test_igrf_order [--csv <path>]
 *
 * Flies the design SSO elements at 450, 500 and 550 km through four 24 h
 * arcs spread over a year (TLE epoch + 0/91/182/273 days, 10 s steps) and
 * compares the IGRF-14 field truncated at degree N = 1…12 with the full
 * degree-13 field at the same point. Per degree it reports:
 *   - direction error mean / 95th percentile / max [deg]  (what TRIAD sees)
 *   - |B| error 95th percentile [%]
 *   - number of (n, m) terms and host time per call relative to N = 13
 *
 * The closed-loop half of the study is sweep_igrf_order.sh, which runs
 * test_nadir --ephem for a set of degrees, seeds and start times.
 * Not a pass/fail test (not registered with ctest).
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <sys/stat.h>
#include "adcs_types.h"
#include "adcs_config.h"
#include "adcs_math.h"
#include "adcs_time.h"
#include "adcs_tle.h"
#include "adcs_sgp4.h"
#include "adcs_igrf.h"
#include "sim_environment.h"

#define N_ALT       3
#define N_ARC       4
#define ARC_S       86400.0
#define STEP_S      10.0
#define SAMPLES_PER_ARC  ((int)(ARC_S / STEP_S))
#define N_SAMPLES   (N_ALT * N_ARC * SAMPLES_PER_ARC)

/** Host timing: calls per degree */
#define TIMING_CALLS  200000

static const double altitudes_km[N_ALT] = {450.0, 500.0, 550.0};
static const double arc_days[N_ARC] = {0.0, 91.0, 182.0, 273.0};

typedef struct {
    vec3d_t r_ecef;
    double year;
} sample_t;

static int cmp_double(const void *a, const void *b)
{
    double x = *(const double *)a, y = *(const double *)b;
    return (x > y) - (x < y);
}

static double angle_deg(vec3d_t a, vec3d_t b)
{
    double c = vec3d_dot(a, b) / (vec3d_norm(a) * vec3d_norm(b));
    if (c > 1.0) c = 1.0;
    if (c < -1.0) c = -1.0;
    return acos(c) * RAD_TO_DEG;
}

static double now_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + 1.0e-9 * (double)ts.tv_nsec;
}

/** Mean motion [rev/day] of a circular orbit at the given altitude (WGS-72 mu) */
static double mean_motion_rev_day(double alt_km)
{
    double a = 6378.137 + alt_km;
    return sqrt(398600.8 / (a * a * a)) * 86400.0 / (2.0 * M_PI);
}

int main(int argc, char **argv)
{
    const char *csv_path = "results/igrf_order_open_loop.csv";
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--csv") == 0 && i + 1 < argc) {
            csv_path = argv[++i];
        } else {
            fprintf(stderr, "usage: %s [--csv <path>]\n", argv[0]);
            return 2;
        }
    }

    adcs_tle_t design;
    if (adcs_tle_parse(SIM_DESIGN_TLE1, SIM_DESIGN_TLE2, &design) != ADCS_TLE_OK) {
        fprintf(stderr, "design TLE rejected\n");
        return 1;
    }

    sample_t *samples = malloc(sizeof(sample_t) * N_SAMPLES);
    double *dir_err[ADCS_IGRF_NMAX + 1] = {0};
    double *mag_err[ADCS_IGRF_NMAX + 1] = {0};
    if (samples == NULL) {
        return 1;
    }
    for (int n = 1; n < ADCS_IGRF_NMAX; n++) {
        dir_err[n] = malloc(sizeof(double) * N_SAMPLES);
        mag_err[n] = malloc(sizeof(double) * N_SAMPLES);
        if (dir_err[n] == NULL || mag_err[n] == NULL) {
            return 1;
        }
    }

    /* ---- Sample points along the orbits ---- */
    int ns = 0;
    for (int a = 0; a < N_ALT; a++) {
        adcs_tle_t tle = design;
        adcs_sgp4_t rec;
        tle.mean_motion = mean_motion_rev_day(altitudes_km[a]);
        if (adcs_sgp4_init(&tle, &rec) != ADCS_SGP4_OK) {
            fprintf(stderr, "SGP4 init failed at %.0f km\n", altitudes_km[a]);
            return 1;
        }
        for (int k = 0; k < N_ARC; k++) {
            for (int i = 0; i < SAMPLES_PER_ARC; i++) {
                double dt = arc_days[k] * 86400.0 + STEP_S * i;
                double unix_s = tle.epoch_unix + dt;
                double r[3], v[3];
                if (adcs_sgp4_propagate(&rec, dt / 60.0, r, v) != ADCS_SGP4_OK) {
                    fprintf(stderr, "SGP4 failed\n");
                    return 1;
                }
                vec3d_t r_teme = vec3d_make(r[0] * 1e3, r[1] * 1e3, r[2] * 1e3);
                samples[ns].r_ecef = vec3d_rot_z(r_teme, adcs_time_gmst(unix_s));
                samples[ns].year = adcs_time_unix_to_decimal_year(unix_s);
                ns++;
            }
        }
    }

    /* ---- Truncation errors vs degree 13 ---- */
    for (int i = 0; i < ns; i++) {
        vec3d_t b13;
        adcs_igrf_field_ecef(samples[i].r_ecef, samples[i].year, 13, &b13);
        double b13n = vec3d_norm(b13);
        for (int n = 1; n < ADCS_IGRF_NMAX; n++) {
            vec3d_t bn;
            adcs_igrf_field_ecef(samples[i].r_ecef, samples[i].year, n, &bn);
            dir_err[n][i] = angle_deg(bn, b13);
            mag_err[n][i] = 100.0 * fabs(vec3d_norm(bn) - b13n) / b13n;
        }
    }

    /* ---- Host time per call ---- */
    double ns_per_call[ADCS_IGRF_NMAX + 1];
    volatile double sink = 0.0;
    for (int n = 1; n <= ADCS_IGRF_NMAX; n++) {
        double t0 = now_s();
        for (int c = 0; c < TIMING_CALLS; c++) {
            vec3d_t b;
            const sample_t *s = &samples[c % ns];
            adcs_igrf_field_ecef(s->r_ecef, s->year, n, &b);
            sink += b.x;
        }
        ns_per_call[n] = (now_s() - t0) / TIMING_CALLS * 1.0e9;
    }
    (void)sink;

    /* ---- Report ---- */
    mkdir("results", 0755);
    FILE *csv = fopen(csv_path, "w");
    if (csv) {
        fprintf(csv, "n_max,terms,dir_mean_deg,dir_p95_deg,dir_max_deg,mag_p95_pct,"
                     "host_ns_per_call,cost_rel_n13\n");
    }

    printf("=== IGRF-14 truncation study (open loop) ===\n");
    printf("%d samples: %d altitudes (450/500/550 km) x %d day-long arcs over a year, %.0f s step\n\n",
           ns, N_ALT, N_ARC, STEP_S);
    printf(" N  terms  dir mean   dir p95   dir max   |B| p95   host ns  cost/N13\n");
    printf("--  -----  --------  --------  --------  --------  -------  --------\n");
    for (int n = 1; n <= ADCS_IGRF_NMAX; n++) {
        int terms = (n + 1) * (n + 2) / 2 - 1;
        double mean = 0.0, p95 = 0.0, max = 0.0, mp95 = 0.0;
        if (n < ADCS_IGRF_NMAX) {
            for (int i = 0; i < ns; i++) {
                mean += dir_err[n][i];
            }
            mean /= ns;
            qsort(dir_err[n], ns, sizeof(double), cmp_double);
            qsort(mag_err[n], ns, sizeof(double), cmp_double);
            p95 = dir_err[n][(int)(0.95 * (ns - 1))];
            max = dir_err[n][ns - 1];
            mp95 = mag_err[n][(int)(0.95 * (ns - 1))];
        }
        double rel = ns_per_call[n] / ns_per_call[ADCS_IGRF_NMAX];
        printf("%2d  %5d  %7.4f°  %7.4f°  %7.4f°  %7.3f%%  %7.0f  %8.2f\n",
               n, terms, mean, p95, max, mp95, ns_per_call[n], rel);
        if (csv) {
            fprintf(csv, "%d,%d,%.6f,%.6f,%.6f,%.6f,%.1f,%.4f\n",
                    n, terms, mean, p95, max, mp95, ns_per_call[n], rel);
        }
    }
    if (csv) {
        fclose(csv);
        printf("\nCSV: %s\n", csv_path);
    }

    for (int n = 1; n < ADCS_IGRF_NMAX; n++) {
        free(dir_err[n]);
        free(mag_err[n]);
    }
    free(samples);
    return 0;
}
