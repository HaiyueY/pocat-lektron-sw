/**
 * @file mmc5983ma.h
 * @brief MEMSIC MMC5983MA 3-axis magnetometer driver (I2C, polled).
 *
 * Single-shot measurements, 18-bit output (16384 counts/G, null at
 * 131072). Each field reading is a SET/RESET pair: after SET the output
 * is +H + offset, after RESET it is −H + offset, so H = (SET − RESET)/2
 * cancels the bridge offset and its temperature drift (datasheet Rev A,
 * "SET and RESET"). The INT pin is not connected on PQ_ADCS_HBridge.
 *
 * Output is in the sensor frame [T]; rotation to body is applied by the
 * board HAL from adcs_calib_t.
 */

#ifndef MMC5983MA_H
#define MMC5983MA_H

#ifdef __cplusplus
extern "C" {
#endif

/** 7-bit I2C address (fixed) */
#define MMC5983MA_I2C_ADDR      0x30u

/** Product ID 1 register value */
#define MMC5983MA_PRODUCT_ID    0x30u

/**
 * Decimation filter: BW[1:0] in Internal Control 1.
 * 0: 8 ms / 100 Hz (lowest noise), 1: 4 ms, 2: 2 ms, 3: 0.5 ms.
 * One field reading takes two measurements.
 */
#define MMC5983MA_BW            0u

/**
 * @brief Soft-reset the device, check its product ID and set the bandwidth.
 * @return 0 on success, -1 on bus error or wrong ID
 */
int mmc5983ma_init(void);

/**
 * @brief Take one offset-cancelled field measurement.
 * @param[out] field_t  Field in the sensor frame [T]
 * @return 0 on success, -1 on bus error or measurement timeout
 */
int mmc5983ma_read_field(double field_t[3]);

/**
 * @brief Take one temperature measurement (≈0.8 °C/LSB).
 * @param[out] temp_c  Die temperature [°C]
 * @return 0 on success, -1 on bus error or timeout
 */
int mmc5983ma_read_temp(double *temp_c);

#ifdef __cplusplus
}
#endif

#endif /* MMC5983MA_H */
