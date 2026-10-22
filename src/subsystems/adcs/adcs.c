/**
 * @file adcs.c
 * @brief Implementation of the ADCS task.
 * @date 2026-01-20
 *
 * Scheduling shell around adcs_ctrl_cycle(): absolute-deadline control
 * periods, notifications, the pause/resume protocol and the health kick.
 * The magnetorquers are put to sleep before the task can be paused or
 * after it is (re)created, so no coil is left powered by the OBC's task
 * management.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

#include "adcs.h"
#include "adcs_bench.h"
#include "adcs_config.h"
#include "adcs_ctrl.h"
#include "adcs_hal.h"
#include "adcs_magnetorquer.h"
#include "adcs_tle.h"
#include "flash.h"
#include "health.h"
#include "notifications.h"
#include "task_management.h"
#include "time.h"

/** Longest wait between health kicks, independent of the control period [ms] */
#define ADCS_KICK_PERIOD_MS     1000u

/** TLE record in flash at TLE_ADDR: line 1 then line 2, ADCS_TLE_LINE_LEN
 *  characters each, no terminators (format TBC with the TC upload in S5) */
#define ADCS_TLE_FLASH_LEN      (2u * ADCS_TLE_LINE_LEN)

/** @brief Notification bits deferred while the ADCS task is paused. */
static uint32_t deferred_notifications;

/** Controller state; static so it does not load the 4 KB task stack */
static adcs_ctrl_t ctrl;

static const adcs_io_t board_io = {
    .read_mag   = adcs_hal_read_mag,
    .read_gyro  = adcs_hal_read_gyro,
    .read_css   = adcs_hal_read_css,
    .mtq_enable = adcs_hal_mtq_enable,
    .mtq_apply  = mtq_apply_command,
};

/** Next control deadline and the tick of the previous samples */
static TickType_t next_wake;
static TickType_t last_sample;
static bool have_sample;
static bool paused;

static adcs_task_stats_t stats;

static void setup_adcs(void);
static void process_adcs(void);
static void apply_desired_mode(uint32_t notification);
static void load_tle_from_flash(void);
static void run_cycle(void);

void adcs_task(void *pv_parameters) {
    (void)pv_parameters;
    setup_adcs();

    for (;;) {
        process_adcs();
        health_kick(HEALTH_BIT_ADCS);
    }
}

void adcs_get_task_stats(adcs_task_stats_t *out) {
    taskENTER_CRITICAL();
    *out = stats;
    taskEXIT_CRITICAL();
}

/**
 * @brief Initialize ADCS task state.
 *
 * Also runs after a health-check reset, which deletes the task wherever it
 * was: the coils go to sleep first.
 */
static void setup_adcs(void) {
    (void)adcs_hal_mtq_enable(false);

    deferred_notifications = 0;
    paused = false;
    have_sample = false;
    memset(&stats, 0, sizeof(stats));
    stats.stack_hwm_words = UINT32_MAX;

    if (adcs_hal_init() != 0) {
        printf("ADCS: sensor init failed\r\n");
    }
    adcs_ctrl_init(&ctrl);
    load_tle_from_flash();

#ifdef ADCS_BENCH
    adcs_bench_run();
#endif

    next_wake = xTaskGetTickCount();
}

/**
 * @brief Execute one ADCS task processing cycle.
 *
 * Waits for a notification until the next control deadline (at most
 * ADCS_KICK_PERIOD_MS), handles the pause/resume protocol and pending
 * notifications, and runs the control cycle once the deadline has passed.
 */
static void process_adcs(void) {

    TickType_t now = xTaskGetTickCount();
    TickType_t timeout = pdMS_TO_TICKS(ADCS_KICK_PERIOD_MS);
    TickType_t until_deadline = next_wake - now;

    if ((int32_t)until_deadline <= 0) {
        timeout = 0;
    } else if (until_deadline < timeout) {
        timeout = until_deadline;
    }

    uint32_t notificationValue = wait_for_notification(timeout);

    /* tm_check_pause() ACKs a pause at once: the coils must be off first */
    if (notificationValue & N_TASK_PAUSE) {
        adcs_ctrl_quiesce(&ctrl, &board_io);
    }

    if (tm_check_pause(notificationValue, &deferred_notifications)) {
        paused = true;
        return;
    }

    if (paused) {
        /* Estimates and timing are stale after a pause of unknown length */
        paused = false;
        adcs_ctrl_restart(&ctrl);
        have_sample = false;
        next_wake = xTaskGetTickCount();
    }

    notificationValue |= deferred_notifications;
    deferred_notifications = 0;

    if (notificationValue & N_ADCS_NEW_CALIBRATION) {
        // @todo S5: load the new calibration block from flash
    }

    if (notificationValue & N_ADCS_NEW_TLE) {
        load_tle_from_flash();
    }

    apply_desired_mode(notificationValue);

    if ((int32_t)(xTaskGetTickCount() - next_wake) >= 0) {
        run_cycle();
    }
}

/**
 * @brief Set the desired mode from the N_ADCS_DESIRED_STATE_* bits.
 *
 * If several are pending the safest wins: IDLE, then DETUMBLING, then NADIR.
 */
static void apply_desired_mode(uint32_t notification) {

    if (notification & N_ADCS_DESIRED_STATE_IDLE) {
        (void)adcs_ctrl_set_desired(&ctrl, ADCS_MODE_IDLE);
    } else if (notification & N_ADCS_DESIRED_STATE_DETUMBLING) {
        (void)adcs_ctrl_set_desired(&ctrl, ADCS_MODE_DETUMBLING);
    } else if (notification & N_ADCS_DESIRED_STATE_NADIR) {
        (void)adcs_ctrl_set_desired(&ctrl, ADCS_MODE_NADIR_POINTING);
    } else {
        return;
    }

    /* Act on the new mode now rather than at the end of an idle period */
    next_wake = xTaskGetTickCount();
}

/**
 * @brief Load the TLE stored at TLE_ADDR; the current one stays on failure.
 */
static void load_tle_from_flash(void) {

    uint8_t record[ADCS_TLE_FLASH_LEN];
    char line1[ADCS_TLE_LINE_LEN + 1];
    char line2[ADCS_TLE_LINE_LEN + 1];

    if (OBDH_Read_Request(TLE_ADDR, record, sizeof(record)) != HAL_OK) {
        printf("ADCS: TLE flash read failed\r\n");
        return;
    }

    memcpy(line1, record, ADCS_TLE_LINE_LEN);
    line1[ADCS_TLE_LINE_LEN] = '\0';
    memcpy(line2, record + ADCS_TLE_LINE_LEN, ADCS_TLE_LINE_LEN);
    line2[ADCS_TLE_LINE_LEN] = '\0';

    int rc = adcs_ctrl_load_tle(&ctrl, line1, line2);
    if (rc == ADCS_ORBIT_OK) {
        printf("ADCS: TLE loaded\r\n");
    } else {
        printf("ADCS: TLE rejected (%d)\r\n", rc);
    }
}

/**
 * @brief Run one control cycle and schedule the next deadline.
 */
static void run_cycle(void) {

    TickType_t start = xTaskGetTickCount();
    double dt = have_sample ? (double)(start - last_sample) / (double)configTICK_RATE_HZ
                            : ADCS_CONTROL_DT;
    last_sample = start;
    have_sample = true;

    double period_s = adcs_ctrl_cycle(&ctrl, &board_io, (double)time_get_unix(), dt);

    TickType_t end = xTaskGetTickCount();
    TickType_t period = pdMS_TO_TICKS((uint32_t)(period_s * 1000.0));

    stats.cycles++;
    if ((uint32_t)(end - start) > stats.max_cycle_ticks) {
        stats.max_cycle_ticks = (uint32_t)(end - start);
    }
    UBaseType_t hwm = uxTaskGetStackHighWaterMark(NULL);
    if (hwm < stats.stack_hwm_words) {
        stats.stack_hwm_words = hwm;
    }

    /* Absolute deadlines keep the period free of processing jitter; an
     * overrun restarts the schedule from now */
    next_wake += period;
    if ((int32_t)(end - next_wake) >= 0) {
        stats.overruns++;
        next_wake = end;
    }
}
