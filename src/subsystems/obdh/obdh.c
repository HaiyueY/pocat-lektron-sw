/**
 * @file obdh.c
 * @author Medir Segura medir.segura@estudiantat.upc.edu
 * @brief Implementation of the OBDH task.
 * 
 */


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
#include "task_management.h"

QueueHandle_t obdh_queue_handle;
static uint32_t deferred_notifications;

static void setup_obdh(void);
static void process_obdh(void);


void obdh_task(void *pv_parameters) {
    (void)pv_parameters;
    setup_obdh();

    for (;;) {
        process_obdh(); 
        health_kick(HEALTH_BIT_OBDH);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

}

/**
 * @brief Initialize OBDH task state.
 */
static void setup_obdh(void) {
    deferred_notifications = 0;
    // Apply the default configuration

}

/**
 * @brief Execute one OBDH task processing cycle.
 *
 * Handles pause/resume notifications, receives one pending flash request from
 * the OBDH queue, performs the requested read or write operation, stores the
 * operation status in the request result pointer, and notifies the requesting
 * task when the operation is complete.
 */
static void process_obdh(void) {

    uint32_t notifications = 0;
    notifications = wait_for_notification();

    if (tm_check_pause(notifications, &deferred_notifications))
        return;

    notifications |= deferred_notifications;
    deferred_notifications = 0;
    
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
