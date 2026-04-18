/**
 * @file obc.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2026-01-20
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef INC_OBC_H_
#define INC_OBC_H_

#include "FreeRTOS.h"
#include "task.h"
#include "state_machine.h"

// TODO: revisar stack sizes y prioridades!

// Task stack sizes
#define OBC_STACK_SIZE       1024
#define PAYLOAD_STACK_SIZE   512
#define EPS_STACK_SIZE       512
#define COMMS_STACK_SIZE     1024
#define ADCS_STACK_SIZE      1024
#define OBDH_STACK_SIZE      1024
#define TRANSCEIVER_STACK_SIZE 1024
#define BEACON_STACK_SIZE    512

// Task priorities
#define OBC_PRIORITY        6
#define OBDH_PRIORITY       5
#define COMMS_PRIORITY      4
#define TRANSCEIVER_PRIORITY 4
#define BEACON_PRIORITY     4
#define EPS_PRIORITY        3
#define ADCS_PRIORITY       2
#define PAYLOAD_PRIORITY    1


/**
 * @brief OBC task entry point — runs the OBC state machine.
 */
void obc_task(void *pv_parameters);

/** @name Subsystem task handle getters
 *  Used by the TC handler and other tasks to send notifications.
 *  Handles are valid once obc_task has finished setup_obc().
 * @{ */
TaskHandle_t obc_get_comms_handle(void);
TaskHandle_t obc_get_eps_handle(void);
TaskHandle_t obc_get_obdh_handle(void);
TaskHandle_t obc_get_adcs_handle(void);
TaskHandle_t obc_get_payload_handle(void);
TaskHandle_t obc_get_transceiver_handle(void);
TaskHandle_t obc_get_beacon_handle(void);
/** @} */

#endif /* INC_OBC_H_ */