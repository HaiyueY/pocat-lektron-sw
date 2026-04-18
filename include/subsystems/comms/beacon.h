/**
 * @file beacon.h
 * @brief Periodic beacon transmission task.
 */

#pragma once

#define BEACON_PERIOD_MS  5000  /**< Beacon interval in ms */

/**
 * @brief Beacon task: enqueues a beacon packet every BEACON_PERIOD_MS.
 *
 * Gets the TX queue handle from comms and notifies transceiver_task.
 * Must be created after the scheduler is running.
 */
void beacon_task(void *pv_parameters);
