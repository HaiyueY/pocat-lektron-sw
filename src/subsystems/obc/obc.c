/**
 * @file obc.c
 * @author guillermo.o.tuama@estudiantat.upc.edu
 * @brief Implementation of the OBC task.
 * @details 
 * OBC Task serves as the central scheduler, coordinating the operation of all other tasks. 
 * It is responsible for managing transitions between different operational modes, task scheduling, 
 * power control, and essential satellite checkups.
 * @date 2026-01-20
 * 
 */

#include <stdint.h>
#include "obc.h"
#include "task_management.h"
#include "state_machine.h"
#include "FreeRTOS.h" 
#include "task.h" 
#include "main.h"
#include "eps.h"
#include "comms.h"
#include "obdh.h"
#include "payload.h"
#include "adcs.h"
#include "health.h"
#include "log.h"
#include "flash.h"
#include "notifications.h"

//Variables que vaig fer servir per a la simulació, no verificats
#define OBDH_QUEUE_LEN 10
#define OBDH_ITEM_SIZE sizeof(obdh_request)

static void setup_obc(ObcState_t currentState);
static void process_obc(ObcState_t *currentState);

void obc_task(void *pv_parameters) {

    ObcState_t currentState = (ObcState_t)(uint32_t)pv_parameters;
    setup_obc(currentState);

    for (;;) {
       process_obc(&currentState);
       EventBits_t faults = health_check();
       if (faults != 0)
       {
           tm_handle_health_faults(faults);
       }
    }

}

/**
 * @brief Initialize OBC task state.
 *
 * Creates the OBDH request queue, initializes the health monitoring module,
 * registers the independent watchdog handle, configures the health check
 * period, and creates each subsystem tasks in resumed or paused state according 
 * to the current satellite operational mode.
 *
 * @param currentState Satellite state restored at boot.
 */
static void setup_obc(ObcState_t currentState) {

    // 1. Create queues
    obdh_queue_handle = xQueueCreate(OBDH_QUEUE_LEN, OBDH_ITEM_SIZE);

    if (obdh_queue_handle == NULL) {
        printf("ERROR: Could not create OBDH Queue\n");
        while(1);
    }
<<<<<<< HEAD
    ok = create_adcs_task();
    if (ok != pdPASS)
    {
        printf("Error creating adcs task\r\n");
    }
    health_init();
    health_register_iwdg(&hiwdg);
    health_set_expected(HEALTH_BIT_PAYLOAD | HEALTH_BIT_OBDH |
                        HEALTH_BIT_EPS | HEALTH_BIT_COMMS |
                        HEALTH_BIT_ADCS);
=======

    health_init();
    health_register_iwdg(&hiwdg);
>>>>>>> fcf80c65ec4d92772610f99d8318259001da8dca
    health_config(pdMS_TO_TICKS(5000));

    // 2. Create subsystem tasks
    if (!state_machine_boot(currentState)) {
        printf("Error creating subsystem tasks\r\n");
    }
}


/**
 * @brief Execute one OBC task processing cycle.
 *
 * Reads pending OBC task notifications, processes them, and passes them to 
 * the state machine so it can evaluate possible state transitions.
 *
 * @param currentState Pointer to the current OBC state.
 */
static void process_obc(ObcState_t *currentState) {

    // Process notifications:
    uint32_t notificationValue = wait_for_notification(pdMS_TO_TICKS(2000));

    if (notificationValue == N_OBC_EXIT_STATE_GROUP_MASK) {
        // Notification to change state, but we will check the exact state in the state machine function
    }
    if (notificationValue & N_OBC_UPDATE_TIME) {
        // printf("Updating system time\r\n");
        // Handle time update, e.g., read new time from OBDH or TC and set RTC
    }
    if (notificationValue & N_OBC_HARD_REBOOT) {
        // Handle hard reboot, e.g., trigger a watchdog reset or perform necessary cleanup before rebooting
    }
    if (notificationValue & N_OBC_SOFT_REBOOT) {
        // Handle soft reboot, e.g., reset tasks and reinitialize subsystems without clearing flash
    }
<<<<<<< HEAD
    if (faults & HEALTH_BIT_ADCS) {
        reset_adcs_task();
        printf("ADCS task reset due to health check\r\n");
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

static BaseType_t create_adcs_task(void)
{
    return xTaskCreate(adcs_task, "ADCS", ADCS_STACK_SIZE, NULL, ADCS_PRIORITY, &adcs_task_handle);
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

void reset_adcs_task(void)
{
    if (adcs_task_handle == NULL)
        return;

    taskENTER_CRITICAL();

    vTaskSuspend(adcs_task_handle);
    vTaskDelete(adcs_task_handle);
    adcs_task_handle = NULL;

    taskEXIT_CRITICAL();

    BaseType_t ok = create_adcs_task();
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
=======

    check_next_state(currentState, notificationValue);
}

>>>>>>> fcf80c65ec4d92772610f99d8318259001da8dca
