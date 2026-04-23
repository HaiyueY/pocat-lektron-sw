/**
 * @file state_machine.c
 * @brief OBC operational state machine.
 *
 * Evaluates pending task notifications (N_OBC_EXIT_STATE_TO_*) and returns
 * the next operational state for the satellite.
 */

#include "state_machine.h"
#include "notifications.h"
#include "FreeRTOS.h"
#include "task.h"
#include "flash.h"
#include "task_management.h"
#include "clock_profile.h"

#include <stdio.h>

static void state_operations_at_beginning(ObcState_t *currentState);

void state_machine_init(ObcState_t *currentState) {
    // Initialize state machine, e.g., read current state from flash
    *currentState = NOMINAL; // Default state
    //OBDH_Read_Request(CURRENT_STATE_ADDR, (uint8_t*)currentState, sizeof(ObcState_t));
    state_operations_at_beginning(currentState);
}

static void change_state(ObcState_t *currentState, ObcState_t newState)
{
    OBDH_Write_Request(PREVIOUS_STATE_ADDR, (uint8_t*)currentState, sizeof(ObcState_t)); // Store the current state in flash

    if (*currentState == NOMINAL) {
        tm_pause_nominal_tasks();
    }
    else {
        tm_pause_non_nominal_tasks();
    }

    vTaskSuspendAll(); // revisar
    clock_profile_transition_to_state(newState);
    xTaskResumeAll(); // revisar

    if (newState == NOMINAL) {
        tm_resume_nominal_tasks();
        // return COMMS to NOMINAL mode
        // return ADCS to NOMINAL mode
    }
    else {
        // resume non nominal tasks
        tm_resume_non_nominal_tasks();
        if (newState == CONTINGENCY) {
            // COMMS beacon only
            // ADCS detumbling only
        }
        else if (newState == SUNSAFE) {
            // COMMS beacon only
            // ADCS idle
        }
        else if (newState == SURVIVAL) {
            // COMMS RX only
            // ADCS idle
        }
    }

    *currentState = newState;
    OBDH_Write_Request(CURRENT_STATE_ADDR, (uint8_t*)currentState, sizeof(ObcState_t)); // Update the current state in flash
    state_operations_at_beginning(currentState);
}

static void state_operations_at_beginning(ObcState_t *currentState) {

    switch (*currentState) {

        case NOMINAL:
            // TODO

            break;

        case CONTINGENCY:
            // TODO
            break;

        case SUNSAFE:
            // TODO
            break;

        default:
            //printf("Unknown state\r\n");
            break;
    }
}

void check_next_state(ObcState_t *currentState, uint32_t notificationValue)
{
    // ADD EPS LOGIC 
    if (notificationValue & N_OBC_EXIT_STATE_TO_NOMINAL) {
        change_state(currentState, NOMINAL);
    }
    else if (notificationValue & N_OBC_EXIT_STATE_TO_CONTINGENCY) {
        change_state(currentState, CONTINGENCY);
    }
    else if (notificationValue & N_OBC_EXIT_STATE_TO_SUNSAFE) {
        change_state(currentState, SUNSAFE);
    }
    else if (notificationValue & N_OBC_EXIT_STATE_TO_SURVIVAL) {
        change_state(currentState, SURVIVAL);
    }
    // ... handle other notifications as needed
    return;
}
