/**
 * @file adcs_bus.h
 * @brief Board I/O used by the ADCS device drivers.
 *
 * The drivers in src/subsystems/adcs/drivers/ touch the hardware only
 * through these functions, so they carry no STM32 dependency. The target
 * implementation lives with the board HAL (adcs_hal_board.c) on top of
 * the I2C3 bus layer (i2c_bus.h) and ADC2; a host build can substitute
 * register-level fakes.
 */

#ifndef ADCS_BUS_H
#define ADCS_BUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief Read consecutive registers from an I2C device.
 * @param addr7  7-bit device address
 * @param reg    First register address
 * @param buf    Destination
 * @param len    Number of bytes
 * @return 0 on success, -1 on bus error, NACK or timeout
 */
int adcs_bus_read(uint8_t addr7, uint8_t reg, uint8_t *buf, uint16_t len);

/** @brief Write consecutive registers of an I2C device; same contract as adcs_bus_read(). */
int adcs_bus_write(uint8_t addr7, uint8_t reg, const uint8_t *buf, uint16_t len);

/** @brief Block the calling task for at least ms milliseconds. */
void adcs_bus_delay_ms(uint32_t ms);

/** @brief Busy-wait for at least us microseconds (short settling delays). */
void adcs_bus_delay_us(uint32_t us);

/** @brief Drive the TMUX1108 select lines (SEL_PH2..0) to channel 0–7 (S1–S8). */
void adcs_bus_css_select(uint8_t channel);

/**
 * @brief Sample ADC_PH once.
 * @param[out] volts  Pin voltage [V]
 * @return 0 on success, -1 on ADC error/timeout
 */
int adcs_bus_css_sample(double *volts);

/** @brief Drive PoL_ADCS_POWER_EN (DRV8214 nSLEEP): 1 = awake, 0 = sleep. */
void adcs_bus_mtq_power(int on);

#ifdef __cplusplus
}
#endif

#endif /* ADCS_BUS_H */
