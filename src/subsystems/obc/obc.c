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

/* ---- Macros and constants ---- */
//Variables que vaig fer servir per a la simulació, no verificats
#define OBDH_QUEUE_LEN 10
#define OBDH_ITEM_SIZE sizeof(obdh_request)

/* ---- Module-level variables ---- */
static TaskHandle_t payload_task_handle;
static TaskHandle_t eps_task_handle;
static TaskHandle_t comms_task_handle;
static TaskHandle_t obdh_task_handle;


/* ---- Public getters ---- */

TaskHandle_t obc_get_comms_handle(void)   { return comms_task_handle; }
TaskHandle_t obc_get_eps_handle(void)     { return eps_task_handle; }
TaskHandle_t obc_get_obdh_handle(void)    { return obdh_task_handle; }
TaskHandle_t obc_get_payload_handle(void) { return payload_task_handle; }

/* ---- Private function prototypes ---- */
static void setup_obc(void);
static void process_obc(ObcState_t *currentState);
// static void create_queues(void);  // TODO: implement this function
static void suspend_and_resume_tasks_depending_on_state(ObcState_t *currentState);
static uint32_t waitForNotification(void);
static void handlePayloadCapture(void);
static void handle_health_faults(EventBits_t faults);
static BaseType_t create_payload_task(void);
static BaseType_t create_eps_task(void);
static BaseType_t create_comms_task(void);
static BaseType_t create_obdh_task(void);

void reset_payload_task(void);
void reset_eps_task(void);
void reset_comms_task(void);
void reset_obdh_task(void);

// a considerar/eliminar:
static ObcState_t currentState;

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
    BaseType_t ok = create_payload_task();
    if (ok != pdPASS)
    {
        printf("Error creating payload task\r\n");
    }
    ok = create_eps_task();
    if (ok != pdPASS)
    {
        printf("Error creating eps task\r\n");
    }
    ok = create_comms_task();
    if (ok != pdPASS)
    {
        printf("Error creating comms task\r\n");
    }
    ok = create_obdh_task();
    if (ok != pdPASS)
    {
        printf("Error creating obdh task\r\n");
    }
    health_init();
    health_register_iwdg(&hiwdg);
    health_set_expected(HEALTH_BIT_PAYLOAD | HEALTH_BIT_OBDH |
                        HEALTH_BIT_EPS | HEALTH_BIT_COMMS);
    health_config(pdMS_TO_TICKS(5000));
}


static void process_obc(ObcState_t *currentState) {

    //printf("Processing OBC...\r\n");
    currentState = check_next_state(currentState);

    suspend_and_resume_tasks_depending_on_state(currentState);

    vTaskDelay(pdMS_TO_TICKS(100)); // Delay to prevent busy looping, adjust as needed

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

static uint32_t waitForNotification(void) {
    uint32_t notificationValue;
    xTaskNotifyWait( 0,          // don't clear on entry
                    0xFFFFFFFF, // clear all bits on exit
                    &notificationValue,
                    portMAX_DELAY );
    return notificationValue;
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
        reset_eps_task();
        printf("EPS task reset due to health check\r\n");
    }
    if (faults & HEALTH_BIT_COMMS) {
        reset_comms_task();
        printf("COMMS task reset due to health check\r\n");
    }
    if (faults & HEALTH_BIT_PAYLOAD) {
        reset_payload_task();
        printf("PAYLOAD task reset due to health check\r\n");
    }
    if (faults & HEALTH_BIT_OBDH) {
        reset_obdh_task();
        printf("OBDH task reset due to health check\r\n");
    }
}


static BaseType_t create_payload_task(void)
{
    return xTaskCreate(payload_task, "PAYLOAD", PAYLOAD_STACK_SIZE, NULL, PAYLOAD_PRIORITY, &payload_task_handle);
}

static BaseType_t create_eps_task(void)
{
    return xTaskCreate(eps_task, "EPS", EPS_STACK_SIZE, NULL, EPS_PRIORITY, &eps_task_handle);
}

static BaseType_t create_comms_task(void)
{
    return xTaskCreate(comms_task, "COMMS", COMMS_STACK_SIZE, NULL, COMMS_PRIORITY, &comms_task_handle);
}

static BaseType_t create_obdh_task(void)
{
    return xTaskCreate(obdh_task, "OBDH", OBDH_STACK_SIZE, NULL, OBDH_PRIORITY, &obdh_task_handle);
}

void reset_payload_task(void)
{
    if (payload_task_handle == NULL)
        return;

    taskENTER_CRITICAL();

    vTaskSuspend(payload_task_handle);
    vTaskDelete(payload_task_handle);
    payload_task_handle = NULL;

    taskEXIT_CRITICAL();

    BaseType_t ok = create_payload_task();
    if (ok != pdPASS)
    {
        // error
    }
}

void reset_eps_task(void)
{
    if (eps_task_handle == NULL)
        return;

    taskENTER_CRITICAL();

    vTaskSuspend(eps_task_handle);
    vTaskDelete(eps_task_handle);
    eps_task_handle = NULL;

    taskEXIT_CRITICAL();

    BaseType_t ok = create_eps_task();
    if (ok != pdPASS)
    {
        // error
    }
}

void reset_comms_task(void)
{
    if (comms_task_handle == NULL)
        return;

    taskENTER_CRITICAL();

    vTaskSuspend(comms_task_handle);
    vTaskDelete(comms_task_handle);
    comms_task_handle = NULL;

    taskEXIT_CRITICAL();

    BaseType_t ok = create_comms_task();

    if (ok != pdPASS)
    {
        // error
    }
}

void reset_obdh_task(void)
{
    if (obdh_task_handle == NULL)
        return;

    taskENTER_CRITICAL();

    vTaskSuspend(obdh_task_handle);
    vTaskDelete(obdh_task_handle);
    obdh_task_handle = NULL;

    taskEXIT_CRITICAL();

    BaseType_t ok = create_obdh_task();

    if (ok != pdPASS)
    {
        // error
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