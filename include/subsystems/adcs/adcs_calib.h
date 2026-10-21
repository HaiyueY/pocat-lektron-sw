/**
 * @file adcs_calib.h
 * @brief ADCS sensor/actuator calibration block and its application.
 *
 * adcs_calib_t is the single calibration record of the ADCS: the format
 * TC_UPLOAD_ADCS_CAL will carry and the flash configuration block will
 * hold (S5). Until then the firmware uses the compiled-in defaults from
 * adcs_calib_defaults(), which are identity/zero corrections plus the
 * nominal coil resistances.
 *
 * Sensor models (raw = sensor frame, SI units):
 *
 *   magnetometer  b_body = R_mag · S · (b_raw − h)      [T]
 *   gyroscope     ω_body = R_gyro · (ω_raw − b0 − k·(T − T0))   [rad/s]
 *   sun sensors   v_i    = g_i · (v_raw_i − o_i)        [V], i in body-face
 *                 order +X, −X, +Y, −Y, +Z, −Z (as sun_sensor_data_t)
 *
 * The sensor-to-body rotations are TBC: they depend on the package axis
 * conventions, the footprint rotations on PQ_ADCS_HBridge and the board
 * orientation in the satellite, and are verified by the S2b polarity table.
 */

#ifndef ADCS_CALIB_H
#define ADCS_CALIB_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "adcs_types.h"

/** Layout version of adcs_calib_t; bump on any field change */
#define ADCS_CALIB_VERSION  1u

/** Number of coarse sun sensor channels */
#define ADCS_CSS_COUNT      6

/** ADCS calibration record (float to keep the flash/TC image small) */
typedef struct {
    uint16_t version;               /**< ADCS_CALIB_VERSION */
    uint16_t size;                  /**< sizeof(adcs_calib_t) */

    /* Magnetometer (MMC5983MA) */
    float mag_hard_iron[3];         /**< Hard-iron offset h, sensor frame [T] */
    float mag_soft_iron[3][3];      /**< Soft-iron correction S (row-major) */
    float mag_to_body[3][3];        /**< Rotation sensor → body (row-major) */

    /* Gyroscope (IIM-42652) */
    float gyro_bias[3];             /**< Bias b0 at gyro_bias_t0_c [rad/s] */
    float gyro_bias_tc[3];          /**< Bias temperature slope k [rad/s/K] */
    float gyro_bias_t0_c;           /**< Reference temperature T0 [°C] */
    float gyro_to_body[3][3];       /**< Rotation sensor → body (row-major) */

    /* Coarse sun sensors, body-face order +X, −X, +Y, −Y, +Z, −Z */
    float css_offset[ADCS_CSS_COUNT];   /**< Dark offset o_i [V] */
    float css_gain[ADCS_CSS_COUNT];     /**< Relative gain g_i [-] */

    /* Magnetorquers */
    float mtq_coil_r_ohm[3];        /**< Coil resistance per axis at 20 °C [Ω] */
    float mtq_coil_tc;              /**< Coil resistance temperature coefficient [1/K] */

    uint32_t crc32;                 /**< CRC over the bytes before this field (S5) */
} adcs_calib_t;

/**
 * @brief Fill a calibration record with the compiled-in defaults.
 *
 * Identity soft-iron and rotations, zero offsets and biases, unit CSS
 * gains, MTQ_COIL_R_OHM_{X,Y,Z} and the copper temperature coefficient.
 * The CRC field is left 0 (computed by the S5 storage layer).
 */
void adcs_calib_defaults(adcs_calib_t *cal);

/**
 * @brief Check that a calibration record has the expected layout.
 * @return 0 if version and size match, -1 otherwise
 * @todo S5: verify crc32 once the flash/TC path defines the CRC.
 */
int adcs_calib_check(const adcs_calib_t *cal);

/** @brief Calibrated magnetic field in body frame [T] from sensor-frame [T]. */
vec3d_t adcs_calib_mag(const adcs_calib_t *cal, const double raw_t[3]);

/** @brief Calibrated angular rate in body frame [rad/s] from sensor-frame [rad/s]. */
vec3d_t adcs_calib_gyro(const adcs_calib_t *cal, const double raw_rad_s[3],
                        double temp_c);

/**
 * @brief Calibrated sun-sensor voltages, body-face order.
 * @param[in]  raw_v  Raw voltages in body-face order [V]
 * @param[out] out_v  Calibrated voltages, clamped at 0 [V]
 */
void adcs_calib_css(const adcs_calib_t *cal, const double raw_v[ADCS_CSS_COUNT],
                    double out_v[ADCS_CSS_COUNT]);

/** @brief Coil resistance of one axis at a given temperature [Ω]. */
double adcs_calib_coil_r(const adcs_calib_t *cal, int axis, double temp_c);

#ifdef __cplusplus
}
#endif

#endif /* ADCS_CALIB_H */
