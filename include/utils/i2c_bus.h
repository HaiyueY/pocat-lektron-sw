/**
 * @file i2c_bus.h
 * @brief Shared I2C3 bus access: locking, timeouts, error accounting, recovery.
 * @details
 * I2C3 carries the ADCS devices and the face-board temperature sensors.
 * Every transfer takes the bus mutex, runs with a bounded timeout and is
 * counted per device address. When a transfer fails with a bus-level
 * error (BUSY, timeout, bus error, arbitration loss) the bus is
 * recovered: the peripheral is released, nine SCL clocks free a slave
 * holding SDA low, a STOP is generated and I2C3 is reinitialized.
 *
 * Nothing on the bus can be power-cycled from the OBC (the ADCS sensors
 * are on 3V3_PERM), so this recovery is the only remedy for a stuck bus.
 */

#ifndef INC_UTILS_I2C_BUS_H_
#define INC_UTILS_I2C_BUS_H_

#include <stdint.h>

/** @brief Timeout of one transfer [ms]. */
#define I2C_BUS_TIMEOUT_MS      10u

/** @brief Number of device addresses tracked by the error counters. */
#define I2C_BUS_MAX_DEVICES     8u

/** @brief Transfer statistics of one device address. */
typedef struct {
    uint8_t  addr7;       /**< 7-bit address (0 = unused slot) */
    uint32_t transfers;   /**< Transfers attempted */
    uint32_t errors;      /**< Transfers that failed (NACK, timeout, bus error) */
} i2c_bus_stats_t;

/**
 * @brief Create the bus mutex.
 * @details Must be called once before the scheduler starts. Transfers made
 * before the scheduler runs are not locked.
 */
void i2c_bus_init(void);

/**
 * @brief Read consecutive registers (8-bit register address).
 * @return 0 on success, -1 on failure (the bus is recovered on bus-level errors)
 */
int i2c_bus_read(uint8_t addr7, uint8_t reg, uint8_t *buf, uint16_t len);

/**
 * @brief Write consecutive registers (8-bit register address).
 * @return 0 on success, -1 on failure (the bus is recovered on bus-level errors)
 */
int i2c_bus_write(uint8_t addr7, uint8_t reg, const uint8_t *buf, uint16_t len);

/**
 * @brief Force a bus recovery (9 SCL clocks + STOP + reinit).
 * @return 0 if SDA is released afterwards, -1 if it is still held low
 */
int i2c_bus_recover(void);

/** @brief Number of recoveries performed since boot. */
uint32_t i2c_bus_recovery_count(void);

/**
 * @brief Copy the per-address statistics.
 * @param out Array of I2C_BUS_MAX_DEVICES entries.
 */
void i2c_bus_get_stats(i2c_bus_stats_t out[I2C_BUS_MAX_DEVICES]);

#endif /* INC_UTILS_I2C_BUS_H_ */
