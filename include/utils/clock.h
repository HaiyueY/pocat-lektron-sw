/**
 * @file clock.h
 * @brief Dynamic system clock frequency switching for power management.
 *
 * Provides runtime clock frequency changes based on the OBC operational state.
 * NOMINAL/CONTINGENCY run at 80 MHz (HSI+PLL), SUNSAFE at 8 MHz (MSI),
 * and SURVIVAL at 2 MHz (MSI) to reduce power consumption.
 */

#ifndef INC_CLOCK_H_
#define INC_CLOCK_H_

#include <stdbool.h>
#include "state_machine.h"

typedef enum {
    CLK_FREQ_80MHZ,
    CLK_FREQ_8MHZ,
    CLK_FREQ_2MHZ
} ClockFreq_t;

/**
 * @brief Configure the system clock for the given OBC state during startup.
 * @param state Current OBC operational state.
 * @return true on success, false if a HAL call failed.
 */
bool systemclock_config_for_state(ObcState_t state);

/**
 * @brief Switch system clock frequency for the given OBC state.
 * @param state Current OBC operational state.
 * @return true on success, false if a HAL call failed.
 */
bool clock_switch_for_state(ObcState_t state);

/**
 * @brief Get the current clock frequency setting.
 * @return Current ClockFreq_t value.
 */
ClockFreq_t clock_get_current(void);



#endif /* INC_CLOCK_H_ */
