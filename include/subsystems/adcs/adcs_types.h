/**
 * @file adcs_types.h
 * @brief Common types, enumerations and structures for the ADCS subsystem.
 *
 * Defines the ADCS operating modes, state machine states, and shared data
 * structures used across all ADCS modules.
 */

#ifndef ADCS_TYPES_H
#define ADCS_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* -------------------------------------------------------------------------
 * ADCS Operating Modes
 * ---------------------------------------------------------------------- */

/** ADCS mode state machine states */
typedef enum {
    ADCS_MODE_IDLE              = 0, /**< Default: no control active */
    ADCS_MODE_DETUMBLING        = 1, /**< B-DOT detumbling controller */
    ADCS_MODE_NADIR_POINTING    = 2, /**< Nadir pointing magnetic control */
    ADCS_MODE_SAFE              = 3  /**< Safe mode: sensors reset, no actuation */
} adcs_mode_t;

/* -------------------------------------------------------------------------
 * 3-axis vector type
 * ---------------------------------------------------------------------- */

/** 3-element vector (x, y, z) */
typedef struct {
    double x;
    double y;
    double z;
} vec3d_t;

/** Quaternion: scalar-first convention [w, x, y, z] */
typedef struct {
    double w;
    double x;
    double y;
    double z;
} quat_t;

/** 3x3 rotation matrix (row-major) */
typedef struct {
    double m[3][3];
} mat3d_t;

/* -------------------------------------------------------------------------
 * Sensor data structures
 * ---------------------------------------------------------------------- */

/** Magnetometer reading in body frame [Tesla] */
typedef struct {
    vec3d_t field;           /**< Magnetic field vector (body frame) [T] */
    uint8_t valid;           /**< 1 if data is fresh and valid */
} mag_data_t;

/** Gyroscope reading in body frame [rad/s] */
typedef struct {
    vec3d_t angular_vel;     /**< Angular velocity (body frame) [rad/s] */
    uint8_t valid;           /**< 1 if data is fresh and valid */
} gyro_data_t;

/** Photodiode sun sensor readings */
typedef struct {
    double voltage[6];       /**< Voltage from 6 photodiodes [V] */
    vec3d_t sun_body;        /**< Estimated sun direction in body frame */
    uint8_t sun_visible;     /**< 1 if sun is detected */
    uint8_t valid;
} sun_sensor_data_t;

/* -------------------------------------------------------------------------
 * Actuator command structures
 * ---------------------------------------------------------------------- */

/** Magnetorquer command */
typedef struct {
    vec3d_t dipole;          /**< Desired magnetic dipole [A·m²] */
    vec3d_t intensity_ma;    /**< Quantized current per axis [mA] */
} mtq_command_t;

/* -------------------------------------------------------------------------
 * ADCS state structure
 * ---------------------------------------------------------------------- */

/** Complete ADCS runtime state */
typedef struct {
    adcs_mode_t mode;                /**< Current operating mode */

    /* Sensor data (latest readings) */
    mag_data_t  mag;
    gyro_data_t gyro;
    sun_sensor_data_t sun;

    /* Previous magnetometer reading (for B-DOT derivative) */
    vec3d_t mag_field_prev;

    /* Estimated attitude */
    quat_t  q_eci_body;             /**< Quaternion ECI → Body */
    quat_t  q_triad_prev;           /**< Previous TRIAD estimate (for bias est.) */
    vec3d_t omega_body;             /**< Angular velocity in body frame [rad/s] */
    vec3d_t omega_filtered;         /**< Low-pass filtered gyro (for rate damping) */
    vec3d_t gyro_bias_est;          /**< Estimated gyro bias [rad/s] */

    /* Reference vectors in ECI frame */
    vec3d_t b_field_eci;            /**< Magnetic field in ECI [T] */
    vec3d_t b_field_eci_prev;       /**< Previous B-field in ECI [T] */
    vec3d_t sun_eci;                /**< Sun direction in ECI */
    vec3d_t nadir_eci;              /**< Nadir direction in ECI */
    vec3d_t nadir_eci_prev;         /**< Previous nadir ECI (for orbit rate est.) */

    /* Actuator output */
    mtq_command_t mtq_cmd;

    /* Mode manager */
    uint32_t detumble_stable_count; /**< Consecutive steps below ω threshold */
    uint32_t step_count;            /**< Total control steps executed */

    /* Time */
    double dt;                      /**< Control loop timestep [s] */
} adcs_state_t;

#ifdef __cplusplus
}
#endif

#endif /* ADCS_TYPES_H */
