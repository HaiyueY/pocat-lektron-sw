/**
 * @file adcs_config.h
 * @brief ADCS configuration parameters derived from satellite physical model.
 *
 * Parameters are sourced from:
 *   - docs/Max_mag_moment_calc.m (coil geometry, dipole limits)
 *   - Sim_data_structure.m       (inertia)
 *   - Sim_pq_model.m             (satellite geometry)
 *   - Sim_PID_controller.m       (control gains)
 *   - Sim_sat_sensors.m          (sensor noise models)
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
 *  Source: docs/Max_mag_moment_calc.m (spiral coil geometry + dual coils)
 *  Each axis has 2 coils (opposite faces), wired to produce additive moment.
 *  Lateral (X,Z): IoA * Sx = 150mA × 0.106306 m² = 15.946e-3 A·m²
 *  Top     (Y):   IoA * Sy = 150mA × 0.056065 m² =  8.410e-3 A·m² */
#define MTQ_MAX_DIPOLE_X    15.946e-3
#define MTQ_MAX_DIPOLE_Y     8.410e-3
#define MTQ_MAX_DIPOLE_Z    15.946e-3

/** Magnetorquer coil parameters for dipole↔intensity conversion
 *  dipole = intensity * coil_factor  →  intensity = dipole / coil_factor
 *  Constraint: MTQ_MAX_DIPOLE = MTQ_MAX_INTENSITY_A × MTQ_COIL_FACTOR
 *
 *  Source: docs/Max_mag_moment_calc.m
 *    coil_factor = Σ(Nlayers × side_i²) × N_coils_per_axis
 *    Lateral: 35 turns, L=32.3mm, d=0.22mm, w=0.22mm, 4 layers, ×2 coils
 *    Top:     29 turns, L=26.2mm, d=0.20mm, w=0.25mm, 4 layers, ×2 coils */
#define MTQ_COIL_FACTOR_X   0.106306  /**< 15.946e-3 / 0.150 = Sx×2 [turns·m²] */
#define MTQ_COIL_FACTOR_Y   0.056065  /**<  8.410e-3 / 0.150 = Sy×2 [turns·m²] */
#define MTQ_COIL_FACTOR_Z   0.106306  /**< 15.946e-3 / 0.150 = Sz×2 [turns·m²] */

/** Driver current limits [mA]
 *  Source: docs/Max_mag_moment_calc.m → IomA = 150 mA */
#define MTQ_MIN_INTENSITY_MA    0.5
#define MTQ_MAX_INTENSITY_MA    150.0
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

/** Adaptive control period for detumbling (continuous optimal scheme)
 *  See docs/adcs_detumble_high_rate_analysis.md §7.6 (Eq. 32)
 *
 *  ΔT(ω) = clamp( SNR_MIN / (SNR_COEFF × |ω|),  DT_FLOOR,  DT_CEIL )
 *
 *  SNR_COEFF = B₀ / (√2 × σ_mag)   — Eq. 24
 *    B₀:     nominal geomagnetic field strength at orbital altitude
 *    σ_mag:  magnetometer RMS noise (MAG_NOISE_RMS, defined below)
 *    √2:     noise amplification from two-sample finite difference
 *
 *  This keeps the finite-difference dB/dt SNR ≥ SNR_MIN at all ω,
 *  while the resulting phase lag δ = ωΔT/2 stays negligibly small
 *  (see Eq. 31: ωΔT = SNR_MIN/SNR_COEFF ≈ 5.66e-3 rad ≈ 0.3°).
 */
#define DETUMBLE_B0_NOMINAL     3.0e-5  /**< Geomagnetic field at 400 km polar orbit [T] */
#define DETUMBLE_SNR_MIN        3.0     /**< Minimum dB/dt SNR for reliable sign detection */
#define DETUMBLE_DT_FLOOR       0.1     /**< Minimum control period [s] (10 Hz, §7.10.3) */
#define DETUMBLE_DT_CEIL        1.0     /**< Safety upper bound near exit threshold [s] */

/** Dead-time per control cycle [s]: MTQ off + I2C mag/gyro reads + compute.
 *  During this window the magnetorquer is not generating torque.
 *  See docs/adcs_detumble_high_rate_analysis.md §7.10.1              */
#define DETUMBLE_DEAD_TIME_S    0.0035  /**< ~3.5 ms (MTQ settle 20µs + I2C 2.5ms + margin) */

/** SNR scaling coefficient: SNR = SNR_COEFF × ω × ΔT   (Eq. 24)
 *  Derived from B₀ and σ_mag — NOT a magic number.
 *  = DETUMBLE_B0_NOMINAL / (√2 × MAG_NOISE_RMS)
 *  = 3.0e-5 / (1.41421356 × 4.0e-8) ≈ 530.3  */
#define DETUMBLE_SNR_COEFF      (DETUMBLE_B0_NOMINAL / (1.41421356 * MAG_NOISE_RMS))

/** Proportional B-DOT gain with per-axis saturation
 *  See docs/adcs_detumble_high_rate_analysis.md §9
 *
 *  Control law:  m_i = clamp( −k × dB_i/dt,  −m_max_i,  +m_max_i )
 *
 *  The gain k determines the saturation crossover angular velocity ω_sat:
 *    k = m_max_ref / (ω_sat × B₀)
 *
 *  Physical interpretation:
 *    ω > ω_sat: proportional output exceeds m_max → saturates → bang-bang
 *    ω < ω_sat: proportional output < m_max → smooth torque ∝ ω
 *
 *  Derivation of ω_sat:
 *    At ω_sat, the proportional command equals hardware limit:
 *      k × |dB/dt| = m_max,  where |dB/dt| ≈ ω × B₀
 *    Solving: ω_sat = m_max / (k × B₀), or equivalently k = m_max / (ω_sat × B₀)
 *
 *  Why proportional helps at low ω:
 *    The B-DOT magnetic torque τ = m × B can only damp ω_perp (perpendicular
 *    to B). The ω_parallel component is uncontrollable. With bang-bang (full
 *    m_max), the cross-axis coupling in Euler's equations redistributes
 *    energy chaotically between axes. With proportional (m ∝ ω), the torque
 *    naturally reduces when damping is ineffective, avoiding energy redistribution.
 *
 *  At high ω: |k·dB/dt| > m_max → saturates → equivalent to bang-bang
 *  At low ω:  |k·dB/dt| < m_max → proportional → smooth convergence
 *
 *  Orbit-averaged stability check (proportional regime):
 *    r = (2/3)·k·B₀²·ΔT/I → at ΔT=0.1s, k=1711: r=0.08% per step ≪ 1 ✓
 */
#define BDOT_SAT_OMEGA  (20.0 * DEG_TO_RAD)  /**< Saturation crossover [rad/s] (tunable) */

/** Reference maximum dipole — average of 3 axes [A·m²] */
#define BDOT_M_MAX_REF  ((MTQ_MAX_DIPOLE_X + MTQ_MAX_DIPOLE_Y + MTQ_MAX_DIPOLE_Z) / 3.0)

/** Proportional B-DOT gain [A·m²·s/T]
 *  k = m_max_ref / (ω_sat × B₀)
 *  At ω_sat, per-axis proportional output ≈ m_max → saturation boundary. 
 *  Currently, its around 1.283 * 10^3 A·m²·s/T*/
#define BDOT_GAIN_K     (BDOT_M_MAX_REF / (BDOT_SAT_OMEGA * DETUMBLE_B0_NOMINAL))

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
