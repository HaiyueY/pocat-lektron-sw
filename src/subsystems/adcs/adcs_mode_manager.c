/**
 * @file adcs_mode_manager.c
 * @brief ADCS mode state machine implementation.
 *
 * Manages transitions between ADCS operating modes:
 *   IDLE → DETUMBLING   (on OBC command or after deployment)
 *   DETUMBLING → IDLE   (when ω < threshold sustained)
 *   IDLE → NADIR_POINTING (on OBC command)
 *   NADIR_POINTING → IDLE (on OBC command or anomaly)
 *   Any → SAFE          (on critical anomaly)
 *
 * The step function dispatches to the active mode controller and
 * handles auto-transitions (e.g., detumble completion).
 *
 * MATLAB reference:
 *   The MATLAB simulations run each mode independently; the state
 *   machine is a firmware-only construct. Transition conditions
 *   are derived from:
 *     - Detumbling.m: exit when |ω| < 0.017 rad/s for 30 steps
 *     - cat-structure-subsystems-dmc.md L1454: mode descriptions
 */

#include "adcs_mode_manager.h"
#include "adcs_detumble.h"
#include "adcs_nadir.h"
#include "adcs_config.h"

void adcs_mode_init(adcs_state_t *state)
{
    state->mode = ADCS_MODE_IDLE;
    state->detumble_stable_count = 0;
    state->step_count = 0;
    state->dt = ADCS_CONTROL_DT;
}

int adcs_mode_request(adcs_state_t *state, adcs_mode_t requested)
{
    adcs_mode_t current = state->mode;

    /* Transition validation */
    switch (requested) {
    case ADCS_MODE_DETUMBLING:
        if (current != ADCS_MODE_IDLE) {
            return -1;
        }
        detumble_init(state);
        state->mode = ADCS_MODE_DETUMBLING;
        return 0;

    case ADCS_MODE_NADIR_POINTING:
        if (current != ADCS_MODE_IDLE) {
            return -1;
        }
        nadir_init(state);
        state->mode = ADCS_MODE_NADIR_POINTING;
        return 0;

    case ADCS_MODE_IDLE:
        if (current == ADCS_MODE_SAFE) {
            return -1;
        }
        state->mode = ADCS_MODE_IDLE;
        return 0;

    case ADCS_MODE_SAFE:
        /* Safe mode is always reachable */
        state->mode = ADCS_MODE_SAFE;
        /* Zero actuator commands for safety */
        state->mtq_cmd.dipole.x = 0.0;
        state->mtq_cmd.dipole.y = 0.0;
        state->mtq_cmd.dipole.z = 0.0;
        state->mtq_cmd.intensity_ma.x = 0.0;
        state->mtq_cmd.intensity_ma.y = 0.0;
        state->mtq_cmd.intensity_ma.z = 0.0;
        return 0;

    default:
        return -1;
    }
}

int adcs_mode_step(adcs_state_t *state)
{
    switch (state->mode) {
    case ADCS_MODE_DETUMBLING: {
        int done = detumble_step(state);
        if (done) {
            /* Auto-transition to IDLE when detumbling completes */
            state->mode = ADCS_MODE_IDLE;
            return 1;
        }
        return 0;
    }

    case ADCS_MODE_NADIR_POINTING:
        nadir_step(state);
        return 0;

    case ADCS_MODE_IDLE:
        /* No actuation in IDLE */
        state->mtq_cmd.dipole.x = 0.0;
        state->mtq_cmd.dipole.y = 0.0;
        state->mtq_cmd.dipole.z = 0.0;
        state->mtq_cmd.intensity_ma.x = 0.0;
        state->mtq_cmd.intensity_ma.y = 0.0;
        state->mtq_cmd.intensity_ma.z = 0.0;
        return 0;

    case ADCS_MODE_SAFE:
        /* No actuation in SAFE */
        state->mtq_cmd.dipole.x = 0.0;
        state->mtq_cmd.dipole.y = 0.0;
        state->mtq_cmd.dipole.z = 0.0;
        state->mtq_cmd.intensity_ma.x = 0.0;
        state->mtq_cmd.intensity_ma.y = 0.0;
        state->mtq_cmd.intensity_ma.z = 0.0;
        return 0;

    default:
        return -1;
    }
}
