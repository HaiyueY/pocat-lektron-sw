/**
 * @file adcs_config.h
 * @brief ADCS configuration parameters derived from satellite physical model.
 *
 * Parameters are sourced from the MATLAB simulation configuration files:
 *   - Sim_data_structure.m  (dipole limits, inertia)
 *   - Sim_pq_model.m        (satellite geometry)
 *   - Sim_PID_controller.m  (control gains)
 *   - Sim_sat_sensors.m     (sensor noise models)
 *
 * All values use SI units unless otherwise noted.
 */

#ifndef ADCS_CONFIG_H
#define ADCS_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>

/* =========================================================================
 * Physical Constants
 * ====================================================================== */

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEG_TO_RAD  (M_PI / 180.0)
#define RAD_TO_DEG  (180.0 / M_PI)

/* =========================================================================
 * Satellite Physical Parameters
 * ====================================================================== */

/** Satellite mass [kg] */
#define SAT_MASS            0.234

/** PocketQube face dimension [m] */
#define SAT_FACE_DIM        0.05

/** Inertia tensor diagonal elements [kg·m²]
 *  Source: Sim_data_structure.m, CAD measurements */
#define SAT_INERTIA_XX      131657.12e-9
#define SAT_INERTIA_YY      114011.54e-9
#define SAT_INERTIA_ZZ      125414.51e-9

/* =========================================================================
 * Magnetorquer Parameters
 * ====================================================================== */

/** Maximum magnetic dipole per axis [A·m²]
 *  Source: Sim_data_structure.m → d.maxmoment (coil geometry calculation)
 *  Lateral (X,Z): IoA * Sx = 32mA × 0.05316 m² = 17.01e-4 A·m²
 *  Top     (Y):   IoA * Sy = 32mA × 0.02867 m² =  9.17e-4 A·m² */
#define MTQ_MAX_DIPOLE_X    17.01e-4
#define MTQ_MAX_DIPOLE_Y     9.17e-4
#define MTQ_MAX_DIPOLE_Z    17.01e-4

/** Magnetorquer coil parameters for dipole↔intensity conversion
 *  dipole = intensity * N * S  →  intensity = dipole / (N * S)
 *  Source: Sim_data_structure.m (Sx, Sy, Sz) */
#define MTQ_COIL_FACTOR_X   (168.0 * 0.00018)  /**< N_x * S_x [turns·m²] */
#define MTQ_COIL_FACTOR_Y   (152.0 * 0.00022)  /**< N_y * S_y [turns·m²] */
#define MTQ_COIL_FACTOR_Z   (152.0 * 0.00022)  /**< N_z * S_z [turns·m²] */

/** BD2606MVV driver current limits [mA] */
#define MTQ_MIN_INTENSITY_MA    0.5
#define MTQ_MAX_INTENSITY_MA    32.0
#define MTQ_INTENSITY_STEP_MA   0.5

/* =========================================================================
 * Detumbling Mode Parameters
 * ====================================================================== */

/** Angular velocity threshold for exit [rad/s]
 *  Satellite is considered detumbled when |ω| < threshold
 *  Source: Detumbling.m → w_threshold = 0.017 rad/s ≈ 1°/s */
#define DETUMBLE_OMEGA_THRESHOLD    0.017

/** Number of consecutive stable steps required to exit detumbling */
#define DETUMBLE_STABLE_COUNT       30

/* =========================================================================
 * Nadir Pointing Mode Parameters
 * ====================================================================== */

/** Proportional gain kP for magnetic control law [A·m²]
 *  Source: Sim_PID_controller.m → PIDMIMO(I, 1, 0.00125, 300, 0.1, 1)
 *  Recomputed: kP = 4.373202e-5 */
#define NADIR_KP    4.373202e-5

/** Rate damping gain kR for magnetic control law [A·m²·s/rad]
 *  Source: Sim_PID_controller.m → PIDMIMO(I, 1, 0.00125, 300, 0.1, 1)
 *  Recomputed: kR = 1.907413e-2 */
#define NADIR_KR    1.907413e-2

/** Body axis to align with nadir direction
 *  +Z body axis points to Earth
 *  Source: Sim_PID_controller.m → p.body_vector = [0;0;1] */
#define NADIR_BODY_AXIS_X   0.0
#define NADIR_BODY_AXIS_Y   0.0
#define NADIR_BODY_AXIS_Z   1.0

/* =========================================================================
 * Control Loop Timing
 * ====================================================================== */

/** Default control loop timestep [s]
 *  Source: Sim_time_parameters.m → dT = 1 */
#define ADCS_CONTROL_DT     1.0

/* =========================================================================
 * Sensor Noise Parameters (for simulation)
 * ====================================================================== */

/** Magnetometer RMS noise [T]
 *  Source: Sim_sat_sensors.m → 0.4 mG = 40 nT */
#define MAG_NOISE_RMS       40.0e-9

/** Gyroscope Angle Random Walk [rad/s/√Hz]
 *  Source: Sim_sat_sensors.m → 0.038 °/s = 6.63e-4 rad/s */
#define GYRO_ARW            (0.038 * DEG_TO_RAD)

/** Gyroscope bias instability [rad/s]
 *  Source: Sim_sat_sensors.m */
#define GYRO_BIAS_STD       (0.1 * DEG_TO_RAD)

/** Photodiode noise standard deviation [V]
 *  Source: Sim_sat_sensors.m → 20 mV */
#define PHOTODIODE_NOISE_STD    0.020

#ifdef __cplusplus
}
#endif

#endif /* ADCS_CONFIG_H */
