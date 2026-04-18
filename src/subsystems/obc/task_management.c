/**
 * @file task_management.c
 * @brief Task creation and reset management for OBC subsystem tasks.
 * @version 0.1
 * @date 2026-03-30
 *
 * @copyright Copyright (c) 2026
 */

/* ---- Includes ---- */
#include "task_management.h"
#include "obc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "eps.h"
#include "comms.h"
#include "obdh.h"
#include "payload.h"
#include "adcs.h"
#include "transceiver.h"
#include "beacon.h"
#include "health.h"
#include "flash.h"
#include "notifications.h"
#include "events.h"
#include <stdio.h>

#define TM_PAUSE_ACK_TIMEOUT_MS 10000u // definir

/* ---- Module-level variables ---- */
static TaskHandle_t payload_task_handle;
static TaskHandle_t eps_task_handle;
static TaskHandle_t comms_task_handle;
static TaskHandle_t adcs_task_handle;
static TaskHandle_t obdh_task_handle;
static TaskHandle_t transceiver_task_handle;
static TaskHandle_t beacon_task_handle;

EventGroupHandle_t task_events_handle = NULL;

/* ---- Public getters ---- */

TaskHandle_t obc_get_comms_handle(void)       { return comms_task_handle; }
TaskHandle_t obc_get_eps_handle(void)         { return eps_task_handle; }
TaskHandle_t obc_get_obdh_handle(void)        { return obdh_task_handle; }
TaskHandle_t obc_get_adcs_handle(void)        { return adcs_task_handle; }
TaskHandle_t obc_get_payload_handle(void)     { return payload_task_handle; }
TaskHandle_t obc_get_transceiver_handle(void) { return transceiver_task_handle; }
TaskHandle_t obc_get_beacon_handle(void)      { return beacon_task_handle; }

/* ---- Private function definitions ---- */

static BaseType_t create_task_events(void)
{
    if (task_events_handle == NULL)
    {
        task_events_handle = xEventGroupCreate();
        if (task_events_handle == NULL)
        {
            printf("Error creating task event group\r\n");
            return pdFAIL;
        }
    }
    return pdTRUE;
}

static BaseType_t create_payload_task(void)
{
    BaseType_t ok = xTaskCreate(payload_task, "PAYLOAD", PAYLOAD_STACK_SIZE, NULL, PAYLOAD_PRIORITY, &payload_task_handle);
    
    if (ok == pdPASS)
    {
        health_set_expected(health_get_expected() | HEALTH_BIT_PAYLOAD);
    }
      
    return ok;
}

static BaseType_t create_eps_task(void)
{
    BaseType_t ok = xTaskCreate(eps_task, "EPS", EPS_STACK_SIZE, NULL, EPS_PRIORITY, &eps_task_handle);
    if (ok == pdPASS)
    {
        health_set_expected(health_get_expected() | HEALTH_BIT_EPS);
    }
    return ok;
}

static BaseType_t create_comms_task(void)
{
    BaseType_t ok = xTaskCreate(comms_task, "COMMS", COMMS_STACK_SIZE, NULL, COMMS_PRIORITY, &comms_task_handle);
    if (ok == pdPASS)
    {
        health_set_expected(health_get_expected() | HEALTH_BIT_COMMS);
    }
    return ok;
}

static BaseType_t create_adcs_task(void)
{
    BaseType_t ok = xTaskCreate(adcs_task, "ADCS", ADCS_STACK_SIZE, NULL, ADCS_PRIORITY, &adcs_task_handle);
    if (ok == pdPASS)
    {
        health_set_expected(health_get_expected() | HEALTH_BIT_ADCS);
    }
    return ok;
}

static BaseType_t create_obdh_task(void)
{
    BaseType_t ok = xTaskCreate(obdh_task, "OBDH", OBDH_STACK_SIZE, NULL, OBDH_PRIORITY, &obdh_task_handle);
    if (ok == pdPASS)
    {
        health_set_expected(health_get_expected() | HEALTH_BIT_OBDH);
    }
    return ok;
}

static BaseType_t create_transceiver_task(void)
{
    BaseType_t ok = xTaskCreate(transceiver_task, "TRANSCEIVER", TRANSCEIVER_STACK_SIZE, NULL, TRANSCEIVER_PRIORITY, &transceiver_task_handle);
    if (ok == pdPASS)
    {
        health_set_expected(health_get_expected() | HEALTH_BIT_TRANSCEIVER);
    }
    return ok;
}

static BaseType_t create_beacon_task(void)
{
    BaseType_t ok = xTaskCreate(beacon_task, "BEACON", BEACON_STACK_SIZE, NULL, BEACON_PRIORITY, &beacon_task_handle);
    if (ok == pdPASS)
    {
        health_set_expected(health_get_expected() | HEALTH_BIT_BEACON);
    }
    return ok;
}

/* ---- Public function definitions ---- */

void tm_reset_payload_task(void)
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
        printf("Error recreating payload task\r\n");
    }
}

void tm_reset_eps_task(void)
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
        printf("Error recreating eps task\r\n");
    }
}

void tm_reset_comms_task(void)
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
        printf("Error recreating comms task\r\n");
    }
}

void tm_reset_adcs_task(void)
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
        printf("Error recreating adcs task\r\n");
    }
}

void tm_reset_obdh_task(void)
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
        printf("Error recreating obdh task\r\n");
    }
}

void tm_reset_transceiver_task(void)
{
    if (transceiver_task_handle == NULL)
        return;

    taskENTER_CRITICAL();

    vTaskSuspend(transceiver_task_handle);
    vTaskDelete(transceiver_task_handle);
    transceiver_task_handle = NULL;

    taskEXIT_CRITICAL();

    BaseType_t ok = create_transceiver_task();

    if (ok != pdPASS)
    {
        printf("Error recreating transceiver task\r\n");
    }
}

void tm_reset_beacon_task(void)
{
    if (beacon_task_handle == NULL)
        return;

    taskENTER_CRITICAL();

    vTaskSuspend(beacon_task_handle);
    vTaskDelete(beacon_task_handle);
    beacon_task_handle = NULL;

    taskEXIT_CRITICAL();

    BaseType_t ok = create_beacon_task();

    if (ok != pdPASS)
    {
        printf("Error recreating beacon task\r\n");
    }
}

BaseType_t tm_create_all_tasks(void)
{
    BaseType_t ok = pdPASS;

    ok = create_task_events(); // create event group for task acks

    if (ok != pdPASS)
    {
        printf("Error creating task event group\r\n");
        return ok;
    }

    ok = create_payload_task();
    if (ok != pdPASS)
    {
        printf("Error creating payload task\r\n");
        return ok;
    }

    ok = create_eps_task();
    if (ok != pdPASS)
    {
        printf("Error creating eps task\r\n");
        return ok;
    }

    ok = create_comms_task();
    if (ok != pdPASS)
    {
        printf("Error creating comms task\r\n");
        return ok;
    }

    ok = create_adcs_task();
    if (ok != pdPASS)    {
        printf("Error creating adcs task\r\n");
        return ok;
    }

    ok = create_obdh_task();
    if (ok != pdPASS)
    {
        printf("Error creating obdh task\r\n");
        return ok;
    }

    ok = create_transceiver_task();
    if (ok != pdPASS)
    {
        printf("Error creating transceiver task\r\n");
        return ok;
    }

    ok = create_beacon_task();
    if (ok != pdPASS)
    {
        printf("Error creating beacon task\r\n");
        return ok;
    }

    return pdPASS;
}

void tm_pause_nominal_tasks(void)
{
    xEventGroupClearBits(task_events_handle, EV_TASK_ACK_NOMINAL_MASK);

    xTaskNotify(payload_task_handle, N_TASK_PAUSE, eSetBits);
    xTaskNotify(eps_task_handle, N_TASK_PAUSE, eSetBits);
    xTaskNotify(comms_task_handle, N_TASK_PAUSE, eSetBits);
    xTaskNotify(adcs_task_handle, N_TASK_PAUSE, eSetBits);
    xTaskNotify(obdh_task_handle, N_TASK_PAUSE, eSetBits);
    xTaskNotify(transceiver_task_handle, N_TASK_PAUSE, eSetBits);
    xTaskNotify(beacon_task_handle, N_TASK_PAUSE, eSetBits);

    
    EventBits_t acks = xEventGroupWaitBits(task_events_handle,
                                           EV_TASK_ACK_NOMINAL_MASK,
                                           pdTRUE,
                                           pdTRUE,
                                           pdMS_TO_TICKS(TM_PAUSE_ACK_TIMEOUT_MS));

    EventBits_t missing_acks = EV_TASK_ACK_NOMINAL_MASK & ~acks;
    if (missing_acks != 0)
    {
        printf("tm: PAUSE ACK timeout, missing: 0x%08lX\r\n",
            (unsigned long)missing_acks);
    }
}

void tm_pause_non_nominal_tasks(void)
{
    xEventGroupClearBits(task_events_handle, EV_TASK_ACK_NON_NOMINAL_MASK);

    xTaskNotify(eps_task_handle, N_TASK_PAUSE, eSetBits);
    xTaskNotify(comms_task_handle, N_TASK_PAUSE, eSetBits);
    xTaskNotify(adcs_task_handle, N_TASK_PAUSE, eSetBits);
    xTaskNotify(obdh_task_handle, N_TASK_PAUSE, eSetBits);
    xTaskNotify(transceiver_task_handle, N_TASK_PAUSE, eSetBits);
    xTaskNotify(beacon_task_handle, N_TASK_PAUSE, eSetBits);

    EventBits_t acks = xEventGroupWaitBits(task_events_handle,
                                           EV_TASK_ACK_NON_NOMINAL_MASK,
                                           pdTRUE,
                                           pdTRUE,
                                           pdMS_TO_TICKS(TM_PAUSE_ACK_TIMEOUT_MS));

    EventBits_t missing_acks = EV_TASK_ACK_NON_NOMINAL_MASK & ~acks;
    if (missing_acks != 0)
    {
        printf("tm: PAUSE ACK timeout for non-nominal tasks, missing: 0x%08lX\r\n",
            (unsigned long)missing_acks);
    }
}

void tm_resume_nominal_tasks(void)
{
    xEventGroupClearBits(task_events_handle, EV_TASK_ACK_NOMINAL_MASK);

    xTaskNotify(payload_task_handle, N_TASK_RESUME, eSetBits);
    xTaskNotify(eps_task_handle, N_TASK_RESUME, eSetBits);
    xTaskNotify(comms_task_handle, N_TASK_RESUME, eSetBits);
    xTaskNotify(adcs_task_handle, N_TASK_RESUME, eSetBits);
    xTaskNotify(obdh_task_handle, N_TASK_RESUME, eSetBits);
    xTaskNotify(transceiver_task_handle, N_TASK_RESUME, eSetBits);
    xTaskNotify(beacon_task_handle, N_TASK_RESUME, eSetBits);

    EventBits_t acks = xEventGroupWaitBits(task_events_handle,
                                           EV_TASK_ACK_NOMINAL_MASK,
                                           pdTRUE,
                                           pdTRUE,
                                           pdMS_TO_TICKS(TM_PAUSE_ACK_TIMEOUT_MS));

    EventBits_t missing_acks = EV_TASK_ACK_NOMINAL_MASK & ~acks;
    if (missing_acks != 0)
    {
        printf("tm: RESUME ACK timeout, missing: 0x%08lX\r\n",
            (unsigned long)missing_acks);
    }
}

void tm_resume_non_nominal_tasks(void)
{
    xEventGroupClearBits(task_events_handle, EV_TASK_ACK_NON_NOMINAL_MASK);

    xTaskNotify(eps_task_handle, N_TASK_RESUME, eSetBits);
    xTaskNotify(comms_task_handle, N_TASK_RESUME, eSetBits);
    xTaskNotify(adcs_task_handle, N_TASK_RESUME, eSetBits);
    xTaskNotify(obdh_task_handle, N_TASK_RESUME, eSetBits);
    xTaskNotify(transceiver_task_handle, N_TASK_RESUME, eSetBits);
    xTaskNotify(beacon_task_handle, N_TASK_RESUME, eSetBits);

    EventBits_t acks = xEventGroupWaitBits(task_events_handle,
                                           EV_TASK_ACK_NON_NOMINAL_MASK,
                                           pdTRUE,
                                           pdTRUE,
                                           pdMS_TO_TICKS(TM_PAUSE_ACK_TIMEOUT_MS));

    EventBits_t missing_acks = EV_TASK_ACK_NON_NOMINAL_MASK & ~acks;
    if (missing_acks != 0)
    {
        printf("tm: RESUME ACK timeout for non-nominal tasks, missing: 0x%08lX\r\n",
            (unsigned long)missing_acks);
    }
}
