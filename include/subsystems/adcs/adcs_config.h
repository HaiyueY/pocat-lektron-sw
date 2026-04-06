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

/** Detumbling control period bounds [s]
 *
 *  Adaptive ΔT based on ESA 4× Nyquist rule:
 *    f_ctrl = 4 × f_rot = 2|ω|/π   →   ΔT = π / (2|ω|)
 *    clamped to [DT_MIN, DT_MAX]
 *
 *  At 90°/s: ΔT = 1.0 s (1 Hz).  At 45°/s: ΔT = 2.0 s (0.5 Hz).
 *  This maintains constant ZOH efficiency G = sinc(π/4) ≈ 0.900.   */
#define DETUMBLE_DT_MIN     1.0   /**< Max control freq 1 Hz (at ω ≥ 90°/s) */
#define DETUMBLE_DT_MAX     2.0   /**< Min control freq 0.5 Hz (at ω ≤ 45°/s) */

/** Dead-time per control cycle [s]: MTQ off + I2C mag/gyro reads + compute.
 *  During this window the magnetorquer is not generating torque.
 *  See docs/adcs_detumble_high_rate_analysis.md §7.10.1              */
#define DETUMBLE_DEAD_TIME_S    0.0035  /**< ~3.5 ms (MTQ settle 20µs + I2C 2.5ms + margin) */

/** Nominal geomagnetic field strength at orbital altitude [T] */
#define DETUMBLE_B0_NOMINAL     3.0e-5

/** Proportional ω×B gain with per-axis saturation
 *  See docs/adcs_detumble_high_rate_analysis.md §9
 *
 *  Control law:  m_i = clamp( k × (ω × B)_i,  −m_max_i,  +m_max_i )
 *
 *  The gain k determines the saturation crossover angular velocity ω_sat:
 *    k = m_max_ref / (ω_sat × B₀)
 *
 *  Physical interpretation:
 *    ω > ω_sat: proportional output exceeds m_max → saturates → bang-bang
 *    ω < ω_sat: proportional output < m_max → smooth torque ∝ ω
 *
 *  Why proportional helps at low ω:
 *    The magnetic torque τ = m × B can only damp ω_perp (perpendicular
 *    to B). With bang-bang (full m_max), cross-axis coupling redistributes
 *    energy chaotically. With proportional (m ∝ ω), the torque naturally
 *    reduces when damping is ineffective, avoiding energy redistribution.
 */
#define BDOT_SAT_OMEGA  (20.0 * DEG_TO_RAD)  /**< Saturation crossover [rad/s] (tunable) */

/** Reference maximum dipole — average of 3 axes [A·m²] */
#define BDOT_M_MAX_REF  ((MTQ_MAX_DIPOLE_X + MTQ_MAX_DIPOLE_Y + MTQ_MAX_DIPOLE_Z) / 3.0)

/** Proportional ω×B gain [A·m²·s/T]
 *  k = m_max_ref / (ω_sat × B₀)
 *  At ω_sat, per-axis proportional output ≈ m_max → saturation boundary.
 *  Currently ≈ 1283 A·m²·s/T */
#define BDOT_GAIN_K     (BDOT_M_MAX_REF / (BDOT_SAT_OMEGA * DETUMBLE_B0_NOMINAL))

/* =========================================================================
 * Nadir Pointing Mode Parameters
 * ====================================================================== */

/** Proportional gain kP for magnetic control law [A·m²]
 *
 *  Original PIDMIMO value (4.373e-5) gives ~0.4 mA proportional current
 *  at 20° error — too weak for PoCat hardware (0.5 mA dead zone, 3.4 mA
 *  max dipole).  Increased to 1e-3 so proportional current at 20° error
 *  is ~1.6 mA (well above dead zone, ~47% of max).
 *
 *  Feasible range: [6e-4, 1.6e-2] — lower bound set by dead-zone,
 *  upper bound by hardware saturation at small errors.
 *
 *  Source: first-principles re-derivation for PoCat coil factor S=0.106 */
#define NADIR_KP    1.0e-3

/** Rate damping gain kR for magnetic control law [A·m²·s/rad]
 *
 *  With orbital rate compensation, kR damps only the deviation from
 *  orbit rate.  kR/kP = 90 gives ζ ≈ 1 (critical damping) for
 *  ω_n ≈ 0.01 rad/s effective bandwidth.
 *
 *  Source: first-principles re-derivation for PoCat coil factor S=0.106 */
#define NADIR_KR    1.907413e-2

/** Gyro low-pass filter time constant [s]
 *  EMA filter: ω_filt = α·ω_raw + (1-α)·ω_filt_prev
 *  α = dt/(dt+τ).  τ=10s at dt=1s gives α=0.091, reducing ARW noise
 *  by ~3.3× while adding ~10s phase lag (acceptable vs 5540s orbit). */
#define NADIR_GYRO_FILTER_TAU   0.0

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
 *  MMC5983MA datasheet: ±0.4 mG = 40 nT raw per sample.
 *  With on-board averaging of ~100 samples at 10 Hz,
 *  effective noise ≈ 40/√100 = 4 nT.
 *  MATLAB reference uses safeFact=0.1 → ~2 nT.
 *  Use 4 nT (conservative estimate with averaging). */
#define MAG_NOISE_RMS       4.0e-9

/** Gyroscope Angle Random Walk [rad/s/√Hz]
 *  Source: Sim_sat_sensors.m → 0.038 °/s = 6.63e-4 rad/s */
#define GYRO_ARW            (0.038 * DEG_TO_RAD)

/** Gyroscope bias instability [rad/s]
 *  Source: Sim_sat_sensors.m → 0.1°/s * safeFact(0.1) = 0.01°/s */
#define GYRO_BIAS_STD       (0.01 * DEG_TO_RAD)

/** Gyroscope Rate Random Walk [rad/s/√(s³)]
 *  Source: Sim_sensors_config_file.m → 0.000133°/√(s³)
 *  Governs bias drift rate: Δbias per step = RRW * √dt */
#define GYRO_RRW            (0.000133 * DEG_TO_RAD)

/** Photodiode noise standard deviation [V]
 *  Source: Sim_sat_sensors.m → 20 mV */
#define PHOTODIODE_NOISE_STD 0.020

#ifdef __cplusplus
}
#endif

#endif /* ADCS_CONFIG_H */
