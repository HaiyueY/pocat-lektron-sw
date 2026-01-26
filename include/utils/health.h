/**
 * @file health.h
 * @brief Software watchdog for subsystem health monitoring.
 *
 * This module implements a software watchdog using FreeRTOS event groups.
 * Each subsystem task must periodically call health_kick() to signal it is
 * alive. The OBC task calls system_health() to check which subsystems have
 * failed to kick within the configured period.
 */

#pragma once

#include "FreeRTOS.h"
#include "event_groups.h"
#include <stdint.h>

/**
 * @brief Health monitoring bit flags for each subsystem.
 *
 * Each subsystem has a unique bit that it uses when calling health_kick().
 * These bits are also used with health_set_expected() to define which
 * subsystems should be monitored.
 */
typedef enum {
    HEALTH_BIT_PAYLOAD = (1u << 0),  /**< Payload subsystem */
    HEALTH_BIT_OBDH    = (1u << 1),  /**< On-Board Data Handling subsystem */
    HEALTH_BIT_EPS     = (1u << 2),  /**< Electrical Power System subsystem */
    HEALTH_BIT_COMMS   = (1u << 3),  /**< Communications subsystem */
    HEALTH_BIT_ADCS    = (1u << 4),  /**< Attitude Determination and Control subsystem */
} health_bit_t;

/**
 * @brief Initialize the health monitoring system.
 *
 * Creates the FreeRTOS event group used for tracking health kicks.
 * Must be called once during system startup before any other health functions.
 */
void health_init(void);

/**
 * @brief Signal that a subsystem is alive.
 *
 * Each subsystem task should call this function periodically (at least once
 * per health check period) to indicate it is operating normally.
 *
 * @param bit The health bit for the calling subsystem (e.g., HEALTH_BIT_EPS).
 */
void health_kick(EventBits_t bit);

/**
 * @brief Configure the health check period.
 *
 * Sets the time window within which all expected subsystems must kick.
 * After this period, system_health() will report any subsystems that
 * failed to kick.
 *
 * @param period Health check period in FreeRTOS ticks. Use pdMS_TO_TICKS()
 *               to convert from milliseconds.
 */
void health_config(TickType_t period);

/**
 * @brief Set which subsystems are expected to report health.
 *
 * Only subsystems whose bits are set here will be monitored.
 * Call this when satellite mode changes to adjust monitoring expectations.
 *
 * @param expected_bits Bitmask of subsystems to monitor (OR of health_bit_t values).
 *
 * @note This restarts the health check window.
 */
void health_set_expected(EventBits_t expected_bits);

/**
 * @brief Check system health and get faulty subsystems.
 *
 * Should be called periodically by the OBC task. When the health check
 * period has elapsed, returns which expected subsystems failed to kick.
 *
 * @return Bitmask of faulty subsystems (bits set for subsystems that failed
 *         to kick). Returns 0 if the check period has not elapsed yet or
 *         if all expected subsystems have kicked.
 */
EventBits_t system_health(void);
