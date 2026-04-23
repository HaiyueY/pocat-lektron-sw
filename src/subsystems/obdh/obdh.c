/**
 * @file obdh.c
 * @author Medir Segura medir.segura@estudiantat.upc.edu
 * @brief oversees the management of internal data within the spacecraft. Its primary focus includes 
    housekeeping data, scientific data, and configurations, as well as managing access to flash 
    memory. (primary focus now is saving and retrieving data from flash)
 * @version 0.1
 * @date 2026-01-20
 * 
 * @copyright Copyright (c) 2026
 * 
 */


/* ---- Includes ---- */
#include "obdh.h"
#include <stdbool.h>
#include <stdio.h>
#include "health.h"
#include <stdint.h>
#include <string.h>
#include "main.h"
#include "queue.h"
#include "flash.h"
#include "notifications.h"
#include "events.h"

/* ---- Macros and constants ---- */
// ..

/* ---- Type definitions ---- */
// ..

/* ---- Module-level variables ---- */
// ..
QueueHandle_t obdh_queue_handle;
static bool paused;
static uint32_t deferred_notifications;
/* ---- Private function prototypes ---- */
void setup_obdh(void);
void process_obdh(void);
static uint32_t wait_for_notification(void);


/* ---- Public function definitions ---- */

void obdh_task(void *pv_parameters) {

    setup_obdh();

    for (;;) {
        process_obdh(); 
        health_kick(HEALTH_BIT_OBDH);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

}

/* ---- Private function definitions ---- */

void setup_obdh(void) {
    paused = false;
    deferred_notifications = 0;
    // Apply the default configuration

}

/**
 * @brief This process waits for an element of the queue to be recived,
 * a request. The request can be to read flash or to write flash.
 * When operations are done, then a notification(with flags) is
 * given to the OBC with an event. 
 */
void process_obdh(void) {

    uint32_t notifications = 0;
    notifications = wait_for_notification();

    if (paused) {
        deferred_notifications |= notifications & ~(N_TASK_PAUSE | N_TASK_RESUME);
        if (notifications & N_TASK_RESUME) {
            paused = false;
            notifications = deferred_notifications;
            deferred_notifications = 0;
            xEventGroupSetBits(task_events_handle, EV_TASK_ACK_OBDH);
        }
        else return;
    }

    if (notifications & N_TASK_PAUSE) {
        deferred_notifications |= notifications & ~(N_TASK_PAUSE | N_TASK_RESUME);
        paused = true;
        xEventGroupSetBits(task_events_handle, EV_TASK_ACK_OBDH);
        return;
    }
    
    obdh_request request;
    HAL_StatusTypeDef status=HAL_OK;

    BaseType_t result_queue= xQueueReceive(obdh_queue_handle,&request,pdMS_TO_TICKS(1000));
    if (result_queue== pdPASS)
    {
        if(request.op==FLASH_READ)
        {
            if(request.buf.dst!=NULL)
            {
                Read_Flash(request.addr, request.buf.dst, request.len);
            }
            status=HAL_OK;
        }
        else if(request.op == FLASH_WRITE)
        {
            if(request.buf.src != NULL)
            {
                Write_Flash(request.addr, request.buf.src, request.len);
                status=HAL_OK;
            }
            else
            {
                status=HAL_ERROR;
            }
        }

        if (request.res != NULL)
        {
            *(request.res)=status;
        }
        if (request.client!=NULL)
        {
            xTaskNotify(request.client,N_FLASH_OPERATION_COMPLETE,eSetBits);
        }
        
    }
}

static uint32_t wait_for_notification(void)
{
    uint32_t notificationValue = 0;
    xTaskNotifyWait(0, 0xFFFFFFFF, &notificationValue, 0);
    return notificationValue;
}
