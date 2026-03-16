/**
 * @file adcs_mode_manager.h
 * @brief ADCS mode state machine manager.
 *
 * Manages transitions between ADCS operating modes (IDLE, DETUMBLING,
 * NADIR_POINTING, SAFE) based on OBC commands and autonomous conditions.
 *
 * Mode transitions:
 *   IDLE → DETUMBLING    (OBC command or auto after deploy)
 *   DETUMBLING → IDLE    (ω < threshold for sustained period)
 *   IDLE → NADIR_POINTING (OBC command)
 *   NADIR_POINTING → IDLE (OBC command)
 *   Any → SAFE           (anomaly detected)
 *   SAFE → IDLE          (OBC command)
 */

#ifndef ADCS_MODE_MANAGER_H
#define ADCS_MODE_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "adcs_types.h"

/**
 * @brief Initialize the ADCS mode manager.
 *
 * Sets the initial mode to IDLE and resets all state.
 *
 * @param[in,out] state  ADCS state
 */
void adcs_mode_init(adcs_state_t *state);

/**
 * @brief Request a mode transition.
 *
 * Validates and applies the requested mode change.
 *
 * @param[in,out] state       ADCS state
 * @param[in]     new_mode    Requested new mode
 * @return 0 on success, -1 if transition is invalid
 */
int adcs_mode_request(adcs_state_t *state, adcs_mode_t new_mode);

/**
 * @brief Execute one ADCS control cycle.
 *
 * Dispatches to the appropriate mode handler (detumble_step, nadir_step)
 * and handles autonomous mode transitions.
 *
 * @param[in,out] state  ADCS state
 * @return 1 if mode auto-transitioned (e.g., detumble converged), 0 otherwise, -1 on error
 */
int adcs_mode_step(adcs_state_t *state);

#ifdef __cplusplus
}
#endif

#endif /* ADCS_MODE_MANAGER_H */
