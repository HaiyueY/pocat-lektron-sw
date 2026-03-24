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

#include <stdio.h>

ObcState_t *check_next_state(ObcState_t *currentState)
{
    uint32_t notif = 0;
    xTaskNotifyWait(0, 0xFFFFFFFF, &notif, 0);

    if (notif & N_OBC_EXIT_STATE_TO_NOMINAL)
    { 
        *currentState = NOMINAL;  
        printf("Transitioning to NOMINAL state\r\n");   
    }
    else if (notif & N_OBC_EXIT_STATE_TO_CONTINGENCY) 
    { 
        *currentState = CONTINGENCY; 
        printf("Transitioning to CONTINGENCY state\r\n");
    }
    else if (notif & N_OBC_EXIT_STATE_TO_SUNSAFE)     
    {
        *currentState = SUNSAFE;     
        printf("Transitioning to SUNSAFE state\r\n");
    }
    else if (notif & N_OBC_EXIT_STATE_TO_SURVIVAL)    
    {
        *currentState = SURVIVAL;    
        printf("Transitioning to SURVIVAL state\r\n");
    }

    return currentState;
}
