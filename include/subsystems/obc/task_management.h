/**
 * @file task_management.h
 * @brief Task creation and reset management for OBC subsystem tasks.
 * @version 0.1
 * @date 2026-03-30
 *
 * @copyright Copyright (c) 2026
 */

#ifndef INC_TASK_MANAGEMENT_H_
#define INC_TASK_MANAGEMENT_H_

#include "FreeRTOS.h"
#include "task.h"
#include "obc.h"

/** @name Task creation and reset functions
 * @{ */

/**
 * @brief Create all subsystem tasks (payload, eps, comms, obdh).
 * @return pdPASS on success, pdFAIL otherwise.
 */
BaseType_t tm_create_all_tasks(void);

/**
 * @brief Reset the payload task by suspending, deleting, and recreating it.
 */
void tm_reset_payload_task(void);


/**
 * @brief Reset the EPS task by suspending, deleting, and recreating it.
 */
void tm_reset_eps_task(void);

/**
 * @brief Reset the comms task by suspending, deleting, and recreating it.
 */
void tm_reset_comms_task(void);

/**
 * @brief Reset the ADCS task by suspending, deleting, and recreating it.
 */
void tm_reset_adcs_task(void);

/**
 * @brief Reset the OBDH task by suspending, deleting, and recreating it.
 */
void tm_reset_obdh_task(void);

/**
 * @brief Reset the transceiver task by suspending, deleting, and recreating it.
 */
void tm_reset_transceiver_task(void);

/** @} */

/** @name Task handle getters
 * @{ */

/**
 * @brief Get the comms task handle.
 * @return TaskHandle_t for comms task, or NULL if not created.
 */
TaskHandle_t obc_get_comms_handle(void);

/**
 * @brief Get the EPS task handle.
 * @return TaskHandle_t for EPS task, or NULL if not created.
 */
TaskHandle_t obc_get_eps_handle(void);

/**
 * @brief Get the OBDH task handle.
 * @return TaskHandle_t for OBDH task, or NULL if not created.
 */
TaskHandle_t obc_get_obdh_handle(void);

/**
 * @brief Get the ADCS task handle.
 * @return TaskHandle_t for ADCS task, or NULL if not created.
 */
TaskHandle_t obc_get_adcs_handle(void);

/**
 * @brief Get the payload task handle.
 * @return TaskHandle_t for payload task, or NULL if not created.
 */
TaskHandle_t obc_get_payload_handle(void);

/**
 * @brief Get the transceiver task handle.
 * @return TaskHandle_t for transceiver task, or NULL if not created.
 */
TaskHandle_t obc_get_transceiver_handle(void);

/** @} */

#endif /* INC_TASK_MANAGEMENT_H_ */
