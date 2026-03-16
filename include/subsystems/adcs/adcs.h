/**
 * @file adcs.h
 * @brief Public interface for the ADCS FreeRTOS task.
 *
 * The ADCS task runs attitude determination and control algorithms
 * at 1 Hz.  Mode transitions are driven by OBC task notifications
 * (N_ADCS_DESIRED_STATE_*).
 *
 * @version 1.0
 * @date 2026-03-02
 */

#ifndef INC_ADCS_H_
#define INC_ADCS_H_

/**
 * @brief ADCS FreeRTOS task entry point.
 *
 * Created by OBC during system startup.  Runs an infinite loop:
 *   1. Poll for OBC notifications (non-blocking).
 *   2. Read sensors.
 *   3. Execute current mode algorithm (IDLE/DETUMBLING/NADIR_POINTING/SAFE).
 *   4. Kick health watchdog.
 *   5. Delay 1 second.
 *
 * @param pv_parameters Unused (NULL).
 */
void adcs_task(void *pv_parameters);

#endif /* INC_ADCS_H_ */
