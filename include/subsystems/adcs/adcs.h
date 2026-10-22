/**
 * @file adcs.h
 * @brief Processes sensor data, executes algorithms to determine the satellite's
    position, and issues commands to control and stabilize its orientation.
 * @date 2026-01-20
 *
 */

#ifndef INC_ADCS_H_
#define INC_ADCS_H_

#include <stdint.h>

/** ADCS task timing statistics (for S4b measurements and S5 telemetry) */
typedef struct {
    uint32_t cycles;            /**< Control cycles run since the task started */
    uint32_t overruns;          /**< Cycles that ended after the next deadline */
    uint32_t max_cycle_ticks;   /**< Longest cycle (sensors to MTQ output) [ticks] */
    uint32_t stack_hwm_words;   /**< Smallest free stack seen [words] */
} adcs_task_stats_t;

/**
 * @brief ADCS FreeRTOS task entry point.
 * @param pv_parameters Task parameter provided by xTaskCreate(); currently unused.
 */
void adcs_task(void *pv_parameters);

/**
 * @brief Copy the ADCS task statistics.
 * @param[out] stats  Destination
 */
void adcs_get_task_stats(adcs_task_stats_t *stats);

#endif /* INC_ADCS_H_ */
