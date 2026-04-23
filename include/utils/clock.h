/**
 * @file clock.h
 * @brief Dynamic system clock frequency switching for power management.
 *
 * Initializes and switches the system clock based on a requested frequency.
 */

#ifndef INC_CLOCK_H_
#define INC_CLOCK_H_

#include <stdbool.h>

typedef enum {
    CLK_FREQ_80MHZ,
    CLK_FREQ_8MHZ,
    CLK_FREQ_2MHZ
} ClockFreq_t;

/**
 * @brief Initialize the system clock for a given frequency during startup.
 * @param freq Target system clock frequency.
 * @return true on success, false if a HAL call failed.
 */
bool systemclock_init_for_freq(ClockFreq_t freq);

/**
 * @brief Switch system clock to a given frequency at runtime.
 * @param freq Target system clock frequency.
 * @return true on success, false if a HAL call failed.
 */
bool clock_switch_to_freq(ClockFreq_t freq);

/**
 * @brief Get the current clock frequency setting.
 * @return Current ClockFreq_t value.
 */
ClockFreq_t clock_get_current(void);



#endif /* INC_CLOCK_H_ */
