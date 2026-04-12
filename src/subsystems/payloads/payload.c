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
#include "events.h"

/* ---- Macros and constants ---- */
// ..

/* ---- Type definitions ---- */
// ..

/* ---- Module-level variables ---- */
// ..
static bool paused;
static uint32_t deferred_notifications;

/* ---- Private function prototypes ---- */
static void setup_payload(void);
static void process_payload(void);
static uint32_t wait_for_notification(void);
static void capture_photo(void);

/* ---- Public function definitions ---- */

void payload_task(void *pv_parameters) {

    setup_payload();

    for (;;) {
        process_payload();
        health_kick(HEALTH_BIT_PAYLOAD);
        vTaskDelay(pdMS_TO_TICKS(1000));
        //printf("PAYLOAD loop\r\n");
    }
    
}

/* ---- Private function definitions ---- */

static void setup_payload(void) {

    // Apply default configuration
    // ...
    paused = false;
    deferred_notifications = 0;
    printf("Setting up PAYLOAD...\r\n");
    
}

static void process_payload(void) {

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
        xEventGroupSetBits(task_events_handle, EV_TASK_ACK_PAYLOAD);
        return;
    }

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

static void capture_photo(void) {

    //     /* 1. Initialize camera with current settings */
    // initCam(huart4, resolution, compressibility, info);

    // /* 2. Grab a photo into ‘info’ buffer */
    // getPhoto(huart4, info);

    printf("Capturing photo...\n");

}
