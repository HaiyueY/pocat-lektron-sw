/**
 * @file beacon.h
 * @brief Periodic beacon transmission using a FreeRTOS software timer.
 */

#pragma once

#define BEACON_PERIOD_MS  5000  /**< Beacon interval: 1 minute */

/**
 * @brief Create and start the beacon software timer.
 *
 * Must be called after the scheduler is running (i.e. from within a task).
 */
void beacon_init(void);

void send_beacon(void);
