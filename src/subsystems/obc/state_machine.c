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

#include <stdio.h>

void state_machine_init(ObcState_t *currentState) {
    // Initialize state machine, e.g., read current state from flash
    OBDH_Read_Request(CURRENT_STATE_ADDR, (uint8_t*)currentState, sizeof(ObcState_t));
}

static void change_state(ObcState_t *currentState, ObcState_t newState)
{
    OBDH_Write_Request(PREVIOUS_STATE_ADDR, (uint8_t*)currentState, sizeof(ObcState_t)); // Store the current state in flash
    *currentState = newState;  
    OBDH_Write_Request(CURRENT_STATE_ADDR, (uint8_t*)currentState, sizeof(ObcState_t)); // Update the current state in flash
}

ObcState_t *check_next_state(ObcState_t *currentState, uint32_t notificationValue, uint8_t *stateChange)
{
    // ADD EPS LOGIC 
    if (notificationValue & N_OBC_EXIT_STATE_TO_NOMINAL) {
        change_state(currentState, NOMINAL);
        *stateChange = 1;
    }
    else if (notificationValue & N_OBC_EXIT_STATE_TO_CONTINGENCY) {
        change_state(currentState, CONTINGENCY);
        *stateChange = 1;
    }
    else if (notificationValue & N_OBC_EXIT_STATE_TO_SUNSAFE) {
        change_state(currentState, SUNSAFE);
        *stateChange = 1;
    }
    else if (notificationValue & N_OBC_EXIT_STATE_TO_SURVIVAL) {
        change_state(currentState, SURVIVAL);
        *stateChange = 1;
    }
    // ... handle other notifications as needed
    return currentState;
}
