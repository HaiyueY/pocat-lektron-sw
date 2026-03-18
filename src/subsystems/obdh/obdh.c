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
#include <stdio.h>
#include "health.h"
#include <stdint.h>
#include <string.h>
#include "main.h"
#include "queue.h"
#include "flash.h"
#include "notifications.h"

/* ---- Macros and constants ---- */
// ..

/* ---- Type definitions ---- */
// ..

/* ---- Module-level variables ---- */
// ..
QueueHandle_t obdh_queue_handle;
/* ---- Private function prototypes ---- */
void setup_obdh(void);
void process_obdh(void);


/* ---- Public function definitions ---- */

void obdh_task(void *pv_parameters) {

    setup_obdh();

    for (;;) {
        process_obdh(); // Blocks for 1 second, waiting for requests from the OBC task. If a request is received, it processes it and notifies the OBC task when done.
        health_kick(HEALTH_BIT_OBDH);
        
    }

}


/* ---- Private function definitions ---- */

void setup_obdh(void) {

    printf("Setting up OBDH...\r\n");
    // Apply the default configuration

}
/**
 * @brief This process waits for an element of the queue to be recived,
 * a request. The request can be to read flash or to write flash.
 * When operations are done, then a notification(with flags) is
 * given to the OBC with an event. 
 */
void process_obdh(void) {
    obdh_request request;
    HAL_StatusTypeDef status=HAL_OK;
    //printf("Processing OBDH...\n");

    BaseType_t result_queue= xQueueReceive(obdh_queue_handle,&request,pdMS_TO_TICKS(1000));
    if (result_queue== pdPASS)
    {
        if(request.op==FLASH_READ)
        {
            if(request.buf!=NULL)
            {
                Read_Flash(request.addr, request.buf, request.len);
            }
            status=HAL_OK;
        }
        else if(request.op == FLASH_WRITE)
        {
            if(request.buf != NULL)
            {
                Write_Flash(request.addr, request.buf, request.len);
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