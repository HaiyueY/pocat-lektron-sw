/**
 * @file adcs.c
 * @brief ADCS FreeRTOS task — processes sensor data, executes attitude
 *        determination and control algorithms, and commands magnetorquers.
 *
 * The task integrates the modular ADCS algorithm library with the
 * FreeRTOS notification-driven event model used by pocat-lektron-sw.
 *
 * Mode transitions are commanded by OBC via task notifications defined
 * in notifications.h (N_ADCS_DESIRED_STATE_*).  The internal mode
 * state-machine (adcs_mode_manager) handles transition validation and
 * auto-exit (e.g. detumble → IDLE when angular velocity stabilises).
 *
 * @version 1.0
 * @date 2026-03-02
 */

/* ---- Includes ---- */
#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "adcs.h"
#include "adcs_types.h"
#include "adcs_config.h"
#include "adcs_mode_manager.h"
#include "adcs_detumble.h"
#include "adcs_nadir.h"
#include "adcs_magnetorquer.h"
#include "health.h"
#include "notifications.h"
#include "log.h"

/* ---- Macros and constants ---- */
#define ADCS_LOOP_PERIOD_MS  1000u  /**< Control loop period [ms] */

/* ---- Module-level variables ---- */
static adcs_state_t adcs_state;  /**< Runtime state shared by all modules */

/* ---- Private function prototypes ---- */
static void setup_adcs(void);
static void process_adcs(void);
static void handle_notifications(uint32_t bits);
static void read_sensors(void);

/* ---- Public function definitions ---- */

void adcs_task(void *pv_parameters)
{
    (void)pv_parameters;
    setup_adcs();

    for (;;) {
        process_adcs();
        health_kick(HEALTH_BIT_ADCS);
        vTaskDelay(pdMS_TO_TICKS(ADCS_LOOP_PERIOD_MS));
    }
}

/* ---- Private function definitions ---- */

/**
 * @brief One-time initialisation of ADCS state and mode manager.
 */
static void setup_adcs(void)
{
    memset(&adcs_state, 0, sizeof(adcs_state));
    adcs_state.dt = ADCS_CONTROL_DT;
    adcs_mode_init(&adcs_state);
    printf("ADCS: initialised (mode=IDLE)\r\n");
}

/**
 * @brief Main processing routine called once per loop iteration.
 *
 * 1. Check for OBC notifications → request mode transitions.
 * 2. Read sensors (placeholder — real I2C reads in future).
 * 3. Step the current mode algorithm.
 */
static void process_adcs(void)
{
    /* Non-blocking notification check (timeout = 0) */
    uint32_t bits = 0;
    if (xTaskNotifyWait(0, 0xFFFFFFFF, &bits, 0) == pdTRUE) {
        handle_notifications(bits);
    }

    read_sensors();

    int rc = adcs_mode_step(&adcs_state);
    if (rc == 1) {
        printf("ADCS: auto-transition -> IDLE\r\n");
    }
}

/**
 * @brief Map notification bits to mode-manager requests.
 */
static void handle_notifications(uint32_t bits)
{
    if (bits & N_ADCS_DESIRED_STATE_IDLE) {
        adcs_mode_request(&adcs_state, ADCS_MODE_IDLE);
        printf("ADCS: OBC requested IDLE\r\n");
    }
    if (bits & N_ADCS_DESIRED_STATE_DETUMBLING) {
        adcs_mode_request(&adcs_state, ADCS_MODE_DETUMBLING);
        printf("ADCS: OBC requested DETUMBLING\r\n");
    }
    if (bits & N_ADCS_DESIRED_STATE_NADIR) {
        adcs_mode_request(&adcs_state, ADCS_MODE_NADIR_POINTING);
        printf("ADCS: OBC requested NADIR_POINTING\r\n");
    }
}

/**
 * @brief Populate adcs_state sensor fields from hardware.
 *
 * Currently a placeholder.  When I2C drivers are integrated, this
 * function will read MMC5983MA (magnetometer), IIM-42652 (gyroscope),
 * and the 6-face photodiode array via ADC+MUX.
 */
static void read_sensors(void)
{
    /* TODO: implement real I2C / ADC sensor reads */
}
