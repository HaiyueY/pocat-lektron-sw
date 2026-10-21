/**
 * @file adcs_hal.h
 * @brief ADCS hardware abstraction: calibrated sensors and MTQ power.
 *
 * The ADCS task talks to the hardware only through this interface and
 * mtq_apply_command() (adcs_magnetorquer.h). The target implementation
 * (adcs_hal_board.c) drives the PQ_ADCS_HBridge board: MMC5983MA,
 * IIM-42652, the TMUX1108 sun-sensor mux and three DRV8214 H-bridges,
 * all on I2C3 except the sun sensors (ADC). Host simulations provide the
 * same readings from test/sim/.
 *
 * All outputs are in the body frame, corrected with the active
 * calibration (adcs_calib.h). A failed read returns -1 and clears the
 * reading's valid flag; the caller decides what to do with it.
 */

#ifndef ADCS_HAL_H
#define ADCS_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "adcs_types.h"
#include "adcs_calib.h"

/** Magnetorquer driver health snapshot (one entry per axis) */
typedef struct {
    uint8_t present;        /**< 1 if the driver answered on I2C */
    uint8_t fault;          /**< DRV8214 FAULT register (0x00) */
    double  voltage_v;      /**< Coil terminal voltage readback (VMTR) [V] */
    double  current_ma;     /**< Coil current readback (IMTR) [mA] */
} adcs_mtq_status_t;

/**
 * @brief Configure the ADCS sensors (after boot or after a bus recovery).
 *
 * Leaves the magnetorquers asleep. Uses the default calibration until
 * adcs_hal_set_calib() is called.
 *
 * @return 0 if every sensor answered, -1 if any did not
 */
int adcs_hal_init(void);

/**
 * @brief Select the calibration applied to all subsequent readings.
 * @param cal  Calibration record; copied. Rejected if adcs_calib_check() fails.
 * @return 0 on success, -1 if rejected (the previous calibration stays)
 */
int adcs_hal_set_calib(const adcs_calib_t *cal);

/** @brief Read the magnetic field (SET/RESET offset-cancelled) [T]. */
int adcs_hal_read_mag(mag_data_t *mag);

/** @brief Read the angular rate [rad/s]; also refreshes the gyro temperature. */
int adcs_hal_read_gyro(gyro_data_t *gyro);

/**
 * @brief Read the six sun-sensor voltages (body-face order +X, −X, +Y, −Y, +Z, −Z).
 *
 * Fills voltage[] and valid only; the sun vector and visibility are
 * derived by attitude determination.
 */
int adcs_hal_read_css(sun_sensor_data_t *sun);

/** @brief Last sensor temperatures [°C] (NaN until first read). */
void adcs_hal_get_temps(double *gyro_c, double *mag_c);

/**
 * @brief Switch the magnetorquer drivers on or off.
 *
 * Off drives PoL_ADCS_POWER_EN low, which puts all three DRV8214 to sleep
 * (outputs Hi-Z) regardless of the I2C state — the hardware guarantee
 * used by the pause/reset protocol. On wakes them and reprograms their
 * registers (they reset in sleep), outputs coasting.
 *
 * @return 0 on success, -1 if a present driver failed to configure
 */
int adcs_hal_mtq_enable(bool on);

/** @brief Read the health of the three magnetorquer drivers. */
int adcs_hal_mtq_status(adcs_mtq_status_t status[3]);

#ifdef __cplusplus
}
#endif

#endif /* ADCS_HAL_H */
