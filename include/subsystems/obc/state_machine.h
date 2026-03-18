/**
 * @file state_machine.h
 * @brief OBC operational state machine.
 */

#ifndef INC_STATE_MACHINE_H_
#define INC_STATE_MACHINE_H_

typedef enum {
    NOMINAL,
    CONTINGENCY,
    SUNSAFE,
    SURVIVAL
} ObcState_t;

/**
 * @brief Evaluate pending notifications and determine the next OBC state.
 * @param currentState Pointer to the current operational state.
 * @return Pointer to the (possibly updated) state.
 */
ObcState_t *check_next_state(ObcState_t *currentState);

#endif /* INC_STATE_MACHINE_H_ */
