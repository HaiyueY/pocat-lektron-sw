/**
 * @file events.h
 * @brief FreeRTOS event-group bit definitions.
 */

#pragma once

#include "FreeRTOS.h"
#include "event_groups.h"

extern EventGroupHandle_t task_events_handle;

/* ---- Task-management Events ---- */

#define EV_TASK_ACK_PAYLOAD    (1u << 0)  /**< PAYLOAD acknowledged N_TASK_PAUSE */
#define EV_TASK_ACK_EPS        (1u << 1)  /**< EPS acknowledged N_TASK_PAUSE */
#define EV_TASK_ACK_COMMS      (1u << 2)  /**< COMMS acknowledged N_TASK_PAUSE */
#define EV_TASK_ACK_ADCS       (1u << 3)  /**< ADCS acknowledged N_TASK_PAUSE */
#define EV_TASK_ACK_OBDH          (1u << 4)  /**< OBDH acknowledged N_TASK_PAUSE */
#define EV_TASK_ACK_TRANSCEIVER   (1u << 5)  /**< TRANSCEIVER acknowledged N_TASK_PAUSE */
#define EV_TASK_ACK_BEACON        (1u << 6)  /**< BEACON acknowledged N_TASK_PAUSE */
#define EV_TASK_ACK_NOMINAL_MASK  (EV_TASK_ACK_PAYLOAD | EV_TASK_ACK_EPS | EV_TASK_ACK_COMMS | EV_TASK_ACK_ADCS | EV_TASK_ACK_OBDH | EV_TASK_ACK_TRANSCEIVER | EV_TASK_ACK_BEACON)
#define EV_TASK_ACK_NON_NOMINAL_MASK  (EV_TASK_ACK_EPS | EV_TASK_ACK_COMMS | EV_TASK_ACK_ADCS | EV_TASK_ACK_OBDH | EV_TASK_ACK_TRANSCEIVER | EV_TASK_ACK_BEACON)