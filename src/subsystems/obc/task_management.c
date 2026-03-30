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
#include <stdio.h>

/* ---- Module-level variables ---- */
static TaskHandle_t payload_task_handle;
static TaskHandle_t eps_task_handle;
static TaskHandle_t comms_task_handle;
static TaskHandle_t adcs_task_handle;
static TaskHandle_t obdh_task_handle;

/* ---- Public getters ---- */

TaskHandle_t obc_get_comms_handle(void)   { return comms_task_handle; }
TaskHandle_t obc_get_eps_handle(void)     { return eps_task_handle; }
TaskHandle_t obc_get_obdh_handle(void)    { return obdh_task_handle; }
TaskHandle_t obc_get_adcs_handle(void)    { return adcs_task_handle; }
TaskHandle_t obc_get_payload_handle(void) { return payload_task_handle; }

/* ---- Private function definitions ---- */

static BaseType_t create_payload_task(void)
{
    return xTaskCreate(payload_task, "PAYLOAD", PAYLOAD_STACK_SIZE, NULL, PAYLOAD_PRIORITY, &payload_task_handle);
}

static BaseType_t suspend_payload_task(void)
{
    if (payload_task_handle == NULL)
        return pdFAIL;

    vTaskSuspend(payload_task_handle);
    return pdPASS;
}

static BaseType_t resume_payload_task(void)
{
    if (payload_task_handle == NULL)
        return pdFAIL;

    vTaskResume(payload_task_handle);
    return pdPASS;
}

static BaseType_t create_eps_task(void)
{
    return xTaskCreate(eps_task, "EPS", EPS_STACK_SIZE, NULL, EPS_PRIORITY, &eps_task_handle);
}

static BaseType_t create_comms_task(void)
{
    return xTaskCreate(comms_task, "COMMS", COMMS_STACK_SIZE, NULL, COMMS_PRIORITY, &comms_task_handle);
}

static BaseType_t create_adcs_task(void)
{
    return xTaskCreate(adcs_task, "ADCS", ADCS_STACK_SIZE, NULL, ADCS_PRIORITY, &adcs_task_handle);
}

static BaseType_t suspend_adcs_task(void)
{
    if (adcs_task_handle == NULL)
        return pdFAIL;

    vTaskSuspend(adcs_task_handle);
    return pdPASS;
}

static BaseType_t resume_adcs_task(void)
{
    if (adcs_task_handle == NULL)
        return pdFAIL;

    vTaskResume(adcs_task_handle);
    return pdPASS;
}

static BaseType_t create_obdh_task(void)
{
    return xTaskCreate(obdh_task, "OBDH", OBDH_STACK_SIZE, NULL, OBDH_PRIORITY, &obdh_task_handle);
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

void tm_change_state_to_nominal(void) {
    resume_payload_task();
    resume_adcs_task();
    health_set_expected(HEALTH_BIT_PAYLOAD | HEALTH_BIT_OBDH | HEALTH_BIT_EPS | HEALTH_BIT_COMMS | HEALTH_BIT_ADCS);
}

void tm_change_state_to_contingency(void) {
    // Example: suspend payload task, keep others running
    suspend_payload_task();
    suspend_adcs_task();
    health_set_expected(HEALTH_BIT_OBDH | HEALTH_BIT_EPS | HEALTH_BIT_COMMS);

}

void tm_change_state_to_sunsafe(void) {
    // Example: suspend payload and comms tasks, keep others running
    suspend_payload_task();
    suspend_adcs_task();
    health_set_expected(HEALTH_BIT_OBDH | HEALTH_BIT_EPS | HEALTH_BIT_COMMS);
    // TODO disable EPS Heating
    // TODO Disable comms transmission & reception
}
