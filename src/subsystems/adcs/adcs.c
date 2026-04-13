/**
 * @file adcs.c
 * @author your name (you@domain.com)
 * @brief The ADCS software processes sensor data, executes algorithms to determine the satellite's 
    position, and issues commands to control and stabilize its orientation.
 * @version 0.1
 * @date 2026-01-20
 * 
 * @copyright Copyright (c) 2026
 * 
 */

/* ---- Includes ---- */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "adcs.h"
#include "health.h"
#include "notifications.h"
#include "events.h"

/* ---- Macros and constants ---- */
#define ADCS_DETUMBLING_MODE (1 << 0)
#define ADCS_NADIR_POINTING_MODE (1 << 1)

/* ---- Type definitions ---- */
// ..

/* ---- Module-level variables ---- */
static bool paused;
static uint32_t deferred_notifications;

/* ---- Private function prototypes ---- */
static void setup_adcs(void);
static void process_adcs(void);
static uint32_t wait_for_notification(void);
static void detumble(void);
static void point_to_nadir(void);

/* ---- Public function definitions ---- */

void adcs_task(void *pv_parameters) {
    setup_adcs();
    for (;;) {
        process_adcs();
        health_kick(HEALTH_BIT_ADCS);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ---- Private function definitions ---- */

static void setup_adcs(void) {
    // Apply the default configuration
    paused = false;
    deferred_notifications = 0;
}

static void process_adcs(void) {

    uint32_t notificationValue = wait_for_notification();

    if (paused) { 
        deferred_notifications |= notificationValue & ~(N_TASK_PAUSE | N_TASK_RESUME);
        if (notificationValue & N_TASK_RESUME) {
            paused = false;
            notificationValue = deferred_notifications;
            deferred_notifications = 0;
        }
        else return; 
    }

    if (notificationValue & N_TASK_PAUSE) {
        deferred_notifications |= notificationValue & ~(N_TASK_PAUSE | N_TASK_RESUME);
        paused = true;
        xEventGroupSetBits(task_events_handle, EV_TASK_ACK_ADCS);
        return;
    }

    if (notificationValue & ADCS_DETUMBLING_MODE) {
        detumble();
    }

    if (notificationValue & ADCS_NADIR_POINTING_MODE) {
        point_to_nadir();
    }

}


static void detumble(void) {

    // Don't exit function until finished 
    return;

}

static void point_to_nadir(void) {

    // Don't exit function until finished 

}

static uint32_t wait_for_notification(void) {

    uint32_t notificationValue = 0;
    xTaskNotifyWait( 0,          // don’t clear on entry
                    0xFFFFFFFF,  // clear all bits on exit
                    &notificationValue,
                    0 );
    return notificationValue;

}
