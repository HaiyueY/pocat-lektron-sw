/**
 * @file obc.c
 * @author guillermo.o.tuama@estudiantat.upc.edu
 * @brief OBC Task serves as the central scheduler, coordinating the operation of all other tasks. 
    It is responsible for managing transitions between different operational modes, task scheduling, 
    power control, and essential satellite checkups.
 * @version 0.1
 * @date 2026-01-20
 * 
 * @copyright Copyright (c) 2026
 * 
 */

/* ---- Includes ---- */
#include <stdint.h> //mirar
#include "obc.h"
#include "task_management.h"
#include "state_machine.h"
#include "FreeRTOS.h" // mirar
#include "task.h" // mirar
#include "main.h"
#include "eps.h"
#include "comms.h"
#include "obdh.h"
#include "payload.h"
#include "health.h"
#include "log.h"
#include "flash.h"
#include "notifications.h"

/* ---- Macros and constants ---- */
//Variables que vaig fer servir per a la simulació, no verificats
#define OBDH_QUEUE_LEN 10
#define OBDH_ITEM_SIZE sizeof(obdh_request)


/* ---- Private function prototypes ---- */
static void setup_obc(void);
static void process_obc(ObcState_t *currentState);
// static void create_queues(void);  // TODO: implement this function
static void suspend_and_resume_tasks_depending_on_state(ObcState_t *currentState);
static uint32_t waitForNotification(void);

static void process_obc_notifications(void);

static void handlePayloadCapture(void);
static void handle_health_faults(EventBits_t faults);

// a considerar/eliminar:
static ObcState_t currentState;

ObcState_t obc_get_current_state(void) { return currentState; }

/* ---- Public function definitions ---- */

void obc_task(void *pv_parameters) {
    
    setup_obc();

    for (;;) {
       process_obc(&currentState);
       EventBits_t faults = health_check();
       if (faults != 0)
       {
           handle_health_faults(faults);
       }
       vTaskDelay(pdMS_TO_TICKS(2000)); // Delay to prevent busy looping, adjust as needed  
    }

}

/* ---- Private function defisnitions ---- */

static void setup_obc(void) {

    // 1. Create queues
    // create_queues();  // TODO: implement this function (small version)
    obdh_queue_handle = xQueueCreate(OBDH_QUEUE_LEN, OBDH_ITEM_SIZE);

    if (obdh_queue_handle == NULL) {
        printf("ERROR: Could not create OBDH Queue\n");
        // This has to be implemented
        while(1);
    }
    // 2. Create tasks
    BaseType_t ok = tm_create_all_tasks();
    if (ok != pdPASS)
    {
        printf("Error creating subsystem tasks\r\n");
    }
    health_init();
    health_register_iwdg(&hiwdg);
    health_set_expected(HEALTH_BIT_PAYLOAD | HEALTH_BIT_OBDH |
                        HEALTH_BIT_EPS | HEALTH_BIT_COMMS);
    health_config(pdMS_TO_TICKS(5000));
}


static void process_obc(ObcState_t *currentState) {

    // Process notifications:
    process_obc_notifications();

    //printf("Processing OBC...\r\n");
    currentState = check_next_state(currentState);

    suspend_and_resume_tasks_depending_on_state(currentState);

    vTaskDelay(pdMS_TO_TICKS(100)); // Delay to prevent busy looping, adjust as needed

}

static void process_obc_notifications(void) {

    uint32_t notificationValue;
    xTaskNotifyWait( 0,          // don't clear on entry
                    0xFFFFFFFF, // clear all bits on exit
                    &notificationValue,
                    0);         // don't block, just check if there's a notification);

    // Process the notification value and take appropriate actions
    // For example:
    if (notificationValue & N_OBC_EXIT_STATE_TO_NOMINAL) {
        printf("Transitioning to NOMINAL state\r\n");
        // Handle transition to NOMINAL state
    }
    if (notificationValue & N_OBC_EXIT_STATE_TO_CONTINGENCY) {
        printf("Transitioning to CONTINGENCY state\r\n");
        // Handle transition to CONTINGENCY state
    }
    if (notificationValue & N_OBC_EXIT_STATE_TO_SUNSAFE) {
        printf("Transitioning to SUNSAFE state\r\n");
        // Handle transition to SUNSAFE state
    }
    if (notificationValue & N_OBC_EXIT_STATE_TO_SURVIVAL) {
        printf("Transitioning to SURVIVAL state\r\n");
        // Handle transition to SURVIVAL state
    }
    if (notificationValue & N_OBC_UPDATE_TIME) {
        printf("Updating system time\r\n");
        // Handle time update, e.g., read new time from OBDH or TC and set RTC
    }
    // ... handle other notifications as needed
}

static void suspend_and_resume_tasks_depending_on_state(ObcState_t *currentState) {

    switch (*currentState) {

        case NOMINAL:
            // Suspend or resume tasks as needed for the NOMINAL state
            // vTaskSuspend(task_handle) ....
            // vTaskResume(task_handle) ....
            break;

        default:
            //printf("Unknown state\r\n");
            break;

    }

}

static void handlePayloadCapture(void) {
    // xTaskNotify(payload_task_handle,  // Fixed: was xPayloadTaskHandle
    //         PAYLOAD_PHOTO_CAPTURE,
    //         eSetBits);
    // ...
}

// TODO: implement or remove this function
// static void obc_does_nominal(void) {
//     // 
// }

// implemented until here at the moment:

ObcState_t Nominal(void) {
    // to do:
    //  frequency tratment for state
    for(;;)
	{
        uint32_t notificationValue = waitForNotification();

        //  if ( notificationValue & OBC_PHOTO_CAPTURE ) handlePayloadCapture();
        // ... handle other events
	}
}

/**
 * @brief Handle health faults by resetting unresponsive tasks.
 * @param faults Bitmask of faulty subsystems from health_check().
 */
static void handle_health_faults(EventBits_t faults)
{
    if (faults & HEALTH_BIT_EPS) {
        tm_reset_eps_task();
        printf("EPS task reset due to health check\r\n");
    }
    if (faults & HEALTH_BIT_COMMS) {
        tm_reset_comms_task();
        printf("COMMS task reset due to health check\r\n");
    }
    if (faults & HEALTH_BIT_PAYLOAD) {
        tm_reset_payload_task();
        printf("PAYLOAD task reset due to health check\r\n");
    }
    if (faults & HEALTH_BIT_OBDH) {
        tm_reset_obdh_task();
        printf("OBDH task reset due to health check\r\n");
    }
}



// REVISAR!!
    // The obc task / manager task is the only one that is in charge of changing satellite modes
    // 1. suspends or resumes the other tasks
    // 2. checks the manager queue and looks for notifications/events for changing 
    //    the mode of the satellite or reloading the default configuration
    // 3. evaluates the variable CURRENT SATELLITE STATUS and checks the flags that
    //    are set to 1, it analyses them and decides whether or not needs to perform a transit of mode
    // Can't 2 not be merged into three? Or the other way around?