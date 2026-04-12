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
#include "health.h"
#include "flash.h"
#include "notifications.h"
#include "events.h"
#include <stdio.h>

#define TM_PAUSE_ACK_TIMEOUT_MS 10000u

/* ---- Module-level variables ---- */
static TaskHandle_t payload_task_handle;
static TaskHandle_t eps_task_handle;
static TaskHandle_t comms_task_handle;
static TaskHandle_t adcs_task_handle;
static TaskHandle_t obdh_task_handle;

EventGroupHandle_t task_events_handle = NULL;

/* ---- Public getters ---- */

TaskHandle_t obc_get_comms_handle(void)   { return comms_task_handle; }
TaskHandle_t obc_get_eps_handle(void)     { return eps_task_handle; }
TaskHandle_t obc_get_obdh_handle(void)    { return obdh_task_handle; }
TaskHandle_t obc_get_adcs_handle(void)    { return adcs_task_handle; }
TaskHandle_t obc_get_payload_handle(void) { return payload_task_handle; }

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

    return pdPASS;
}

void tm_pause_all_tasks(void)
{
    xEventGroupClearBits(task_events_handle, EV_TASK_ACK_ALL_MASK);

    xTaskNotify(payload_task_handle, N_TASK_PAUSE, eSetBits);
    xTaskNotify(eps_task_handle, N_TASK_PAUSE, eSetBits);
    xTaskNotify(comms_task_handle, N_TASK_PAUSE, eSetBits);
    xTaskNotify(adcs_task_handle, N_TASK_PAUSE, eSetBits);
    xTaskNotify(obdh_task_handle, N_TASK_PAUSE, eSetBits);

    EventBits_t acks = xEventGroupWaitBits(task_events_handle,
                                           EV_TASK_ACK_ALL_MASK,
                                           pdTRUE,
                                           pdTRUE,
                                           pdMS_TO_TICKS(TM_PAUSE_ACK_TIMEOUT_MS));

    EventBits_t missing_acks = EV_TASK_ACK_ALL_MASK & ~acks;
    if (missing_acks != 0)
    {
        printf("tm: PAUSE ACK timeout, missing: 0x%08lX\r\n",
            (unsigned long)missing_acks);
    }
}

void tm_resume_payload_task(void)
{
    xTaskNotify(payload_task_handle, N_TASK_RESUME, eSetBits);
}

void tm_resume_eps_task(void)
{
    xTaskNotify(eps_task_handle, N_TASK_RESUME, eSetBits);
}

void tm_resume_comms_task(void)
{
    xTaskNotify(comms_task_handle, N_TASK_RESUME, eSetBits);
}

void tm_resume_adcs_task(void)
{
    xTaskNotify(adcs_task_handle, N_TASK_RESUME, eSetBits);
}

void tm_resume_obdh_task(void)
{
    xTaskNotify(obdh_task_handle, N_TASK_RESUME, eSetBits);
}

void tm_resume_all_tasks(void)
{
    tm_resume_payload_task();
    tm_resume_eps_task();
    tm_resume_comms_task();
    tm_resume_adcs_task();
    tm_resume_obdh_task();
}
