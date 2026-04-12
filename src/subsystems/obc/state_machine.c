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
    
    tm_pause_all_tasks();
    
    vTaskSuspendAll(); // revisar
    clock_switch_for_state(newState);
    xTaskResumeAll(); // revisar

    if (*currentState == CONTINGENCY && newState == NOMINAL) {
        tm_resume_all_tasks();
        // return COMMS to NOMINAL mode
        // return ADCS to NOMINAL mode
    }
    else {
        tm_resume_eps_task();
        tm_resume_comms_task();
        tm_resume_adcs_task();
        tm_resume_obdh_task();
        if (*currentState == NOMINAL && newState == CONTINGENCY) {
            // COMMS beacon only
            // ADCS detumbling only
        }
        else if (*currentState == CONTINGENCY && newState == SUNSAFE) {
            // ADCS only idle
        }
        else if (*currentState == SUNSAFE && newState == SURVIVAL) {
            // COMMS RX only
        }
        else if (*currentState == SURVIVAL && newState == SUNSAFE) {
            // COMMS beacon only
        }
        else if (*currentState == SUNSAFE && newState == CONTINGENCY) {
            // ADCS detumbling only
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
