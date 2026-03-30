/**
 * @file state_machine.h
 * @brief OBC operational state machine.
 */

#ifndef INC_STATE_MACHINE_H_
#define INC_STATE_MACHINE_H_

#include <stdint.h>

typedef enum {
    NOMINAL,
    CONTINGENCY,
    SUNSAFE,
    SURVIVAL
} ObcState_t;

/** @brief Initialize the state machine.
 *  @param currentState Pointer to the variable where the current state will be stored.
 */
void state_machine_init(ObcState_t *currentState);

/**
 * @brief Evaluate pending notifications and determine the next OBC state.
 * @param currentState Pointer to the current operational state.
 * @return Pointer to the (possibly updated) state.
 */
ObcState_t *check_next_state(ObcState_t *currentState, uint32_t notificationValue, uint8_t *stateChange);

#endif /* INC_STATE_MACHINE_H_ */
