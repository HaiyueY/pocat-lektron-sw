/**
 * @file adcs_bench.c
 * @brief One-shot on-target timing of the ADCS reference models.
 *
 * Output format (one line per measurement, UART4 debug console):
 *
 *   [ADCS_BENCH] <what> n=<degree> cycles min <a> avg <b> max <c>
 *
 * Each measurement evaluates BENCH_SAMPLES epochs spaced BENCH_STEP_S apart
 * along the PoCat design orbit, so the IGRF/SGP4 inputs vary like in
 * flight. Divide cycles by SystemCoreClock (printed first) for seconds.
 */

#include "adcs_bench.h"

#ifdef ADCS_BENCH

#include <stdint.h>
#include <stdio.h>
#include "stm32l4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "health.h"
#include "adcs_ephemeris.h"
#include "adcs_igrf.h"
#include "adcs_math.h"
#include "adcs_sun.h"
#include "adcs_time.h"

#define BENCH_SAMPLES   16
#define BENCH_STEP_S    600.0

/** PoCat design orbit: 500 km SSO, LTDN 10:30, epoch 2027-09-15 (synthetic) */
static const char bench_tle1[] =
    "1 99999U 27001A   27258.00000000  .00002000  00000-0  10000-3 0  9995";
static const char bench_tle2[] =
    "2 99999  97.4000 330.0000 0010000  90.0000 270.0000 15.21937171    10";

static const int bench_orders[] = {1, 4, 8, 13};

/** Static so that the benchmark does not distort the task stack watermark */
static adcs_orbit_t bench_orbit;

typedef struct {
    uint32_t min;
    uint32_t max;
    uint32_t sum;
} bench_stat_t;

static void stat_reset(bench_stat_t *s)
{
    s->min = UINT32_MAX;
    s->max = 0;
    s->sum = 0;
}

static void stat_add(bench_stat_t *s, uint32_t cycles)
{
    if (cycles < s->min) {
        s->min = cycles;
    }
    if (cycles > s->max) {
        s->max = cycles;
    }
    s->sum += cycles;
}

static void stat_print(const char *what, int order, const bench_stat_t *s)
{
    printf("[ADCS_BENCH] %-6s n=%2d cycles min %lu avg %lu max %lu\r\n",
           what, order, (unsigned long)s->min,
           (unsigned long)(s->sum / BENCH_SAMPLES), (unsigned long)s->max);
}

static double sample_time(int i)
{
    return bench_orbit.tle.epoch_unix + BENCH_STEP_S * (double)i;
}

void adcs_bench_run(void)
{
    bench_stat_t st;
    adcs_ephem_t eph;
    uint32_t t0;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    printf("[ADCS_BENCH] SystemCoreClock %lu Hz, %d samples\r\n",
           (unsigned long)SystemCoreClock, BENCH_SAMPLES);

    adcs_orbit_init(&bench_orbit);
    int rc = adcs_orbit_load_tle(&bench_orbit, bench_tle1, bench_tle2);
    if (rc != ADCS_ORBIT_OK) {
        printf("[ADCS_BENCH] TLE load failed (%d)\r\n", rc);
        return;
    }

    /* SGP4 alone */
    stat_reset(&st);
    for (int i = 0; i < BENCH_SAMPLES; i++) {
        double r[3], v[3];
        double tsince = (sample_time(i) - bench_orbit.tle.epoch_unix) / 60.0;
        t0 = DWT->CYCCNT;
        (void)adcs_sgp4_propagate(&bench_orbit.sgp4, tsince, r, v);
        stat_add(&st, DWT->CYCCNT - t0);
    }
    stat_print("sgp4", 0, &st);

    /* Sun direction + shadow */
    stat_reset(&st);
    for (int i = 0; i < BENCH_SAMPLES; i++) {
        vec3d_t r_eci = {6878137.0, 0.0, 0.0};
        t0 = DWT->CYCCNT;
        vec3d_t s = adcs_sun_direction(sample_time(i));
        (void)adcs_eclipse_cylindrical(r_eci, s);
        stat_add(&st, DWT->CYCCNT - t0);
    }
    stat_print("sun", 0, &st);
    health_kick(HEALTH_BIT_ADCS);

    for (unsigned k = 0; k < sizeof(bench_orders) / sizeof(bench_orders[0]); k++) {
        int order = bench_orders[k];

        /* IGRF alone, at the orbit positions */
        stat_reset(&st);
        for (int i = 0; i < BENCH_SAMPLES; i++) {
            vec3d_t b;
            (void)adcs_ephem_update(&bench_orbit, sample_time(i), 1, &eph);
            vec3d_t r_ecef = vec3d_rot_z(eph.pos_eci, adcs_time_gmst(sample_time(i)));
            double year = adcs_time_unix_to_decimal_year(sample_time(i));
            t0 = DWT->CYCCNT;
            (void)adcs_igrf_field_ecef(r_ecef, year, order, &b);
            stat_add(&st, DWT->CYCCNT - t0);
        }
        stat_print("igrf", order, &st);

        /* Complete reference-vector update */
        stat_reset(&st);
        for (int i = 0; i < BENCH_SAMPLES; i++) {
            t0 = DWT->CYCCNT;
            (void)adcs_ephem_update(&bench_orbit, sample_time(i), order, &eph);
            stat_add(&st, DWT->CYCCNT - t0);
        }
        stat_print("ephem", order, &st);
        health_kick(HEALTH_BIT_ADCS);
    }

    printf("[ADCS_BENCH] ADCS stack high-water mark %lu bytes free\r\n",
           (unsigned long)(uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t)));
}

#else

void adcs_bench_run(void)
{
}

#endif /* ADCS_BENCH */
