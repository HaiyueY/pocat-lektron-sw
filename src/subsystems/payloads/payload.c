/**
 * @file payload.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2026-01-20
 * 
 * @copyright Copyright (c) 2026
 * 
 */


/* ---- Includes ---- */
#include <stdbool.h>
#include "FreeRTOS.h"
#include "main.h"
#include "health.h"
#include "notifications.h"
#include "task_management.h"
#include "log.h"

/* ---- Macros and constants ---- */
// ..

/* ---- Type definitions ---- */
// ..

/* ---- Module-level variables ---- */
// ..
static uint32_t deferred_notifications;

/* ---- Private function prototypes ---- */
static void setup_payload(void);
static void process_payload(void);
static uint32_t wait_for_notification(void);

/* ---- Public function definitions ---- */

void payload_task(void *pv_parameters) {
    (void)pv_parameters;
    setup_payload();

    for (;;) {
        process_payload();
        health_kick(HEALTH_BIT_PAYLOAD);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
}

/* ---- Private function definitions ---- */

static void setup_payload(void) {

    // Apply default configuration
    // ...
    deferred_notifications = 0;

}

static void process_payload(void) {

    uint32_t notificationValue = wait_for_notification();

    if (tm_check_pause(notificationValue, &deferred_notifications))
        return;

    notificationValue |= deferred_notifications;
    deferred_notifications = 0;
    // if (notificationValue & PAYLOAD_PHOTO_CAPTURE) {
    //     capture_photo();
    // }
    // if ... (not else if!!)

}

static uint32_t wait_for_notification(void) {

    uint32_t notificationValue = 0;
    xTaskNotifyWait( 0,          // don't clear on entry
                    0xFFFFFFFF,  // clear all bits on exit
                    &notificationValue,
                    pdMS_TO_TICKS(500) );  // timeout to allow periodic health kicks
    return notificationValue;

}
