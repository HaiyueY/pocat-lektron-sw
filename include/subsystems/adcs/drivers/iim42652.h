/**
 * @file iim42652.h
 * @brief TDK InvenSense IIM-42652 6-axis IMU driver, gyroscope only (I2C, polled).
 *
 * The accelerometer is left off. The gyro runs in low-noise mode at
 * ±250 dps (covers the 90 °/s detumble design case with margin), ODR
 * 100 Hz and a 10 Hz UI low-pass, and the task polls the sense registers
 * (INT1/INT2 are not wired on PQ_ADCS_HBridge).
 *
 * Output is in the sensor frame [rad/s]; rotation to body is applied by
 * the board HAL from adcs_calib_t.
 */

#ifndef IIM42652_H
#define IIM42652_H

#ifdef __cplusplus
extern "C" {
#endif

/** 7-bit I2C address (AP_AD0 tied to GND on PQ_ADCS_HBridge) */
#define IIM42652_I2C_ADDR       0x68u

/** WHO_AM_I register value */
#define IIM42652_WHO_AM_I       0x6Fu

/**
 * @brief Soft-reset, check WHO_AM_I, configure and start the gyroscope.
 *
 * Blocks for the 45 ms minimum gyro on-time, so the first read is valid.
 *
 * @return 0 on success, -1 on bus error or wrong ID
 */
int iim42652_init(void);

/**
 * @brief Read the latest angular rate and die temperature.
 * @param[out] rate_rad_s  Angular rate in the sensor frame [rad/s]
 * @param[out] temp_c      Die temperature [°C] (may be NULL)
 * @return 0 on success, -1 on bus error or if the gyro reports no data
 */
int iim42652_read_gyro(double rate_rad_s[3], double *temp_c);

#ifdef __cplusplus
}
#endif

#endif /* IIM42652_H */
