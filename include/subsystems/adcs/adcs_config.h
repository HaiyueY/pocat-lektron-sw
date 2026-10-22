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

/** Magnetorquer coils (PQ_ADCS_HBridge board, one DRV8214 per axis).
 *
 *  Each axis is two PCB-etched coils in series on opposite faces:
 *    X, Z: two lateral boards (PQ_Lat_Board)
 *    Y:    PQ_MagYBoard (+Y) and PQ_BotBoard (−Y), which differ in size
 *  R and N·A per coil come from the board copper (35 µm, 20 °C) via
 *  scripts/adcs/coil_from_pcb.py:
 *    PQ_Lat_Board  21.66 Ω  0.052935 m²
 *    PQ_BotBoard   21.81 Ω  0.052255 m²
 *    PQ_MagYBoard  12.49 Ω  0.028066 m²
 *  The lateral and +Y values agree with docs/Max_mag_moment_calc.m to
 *  0.4 %; that script assumed two +Y-size coils on Y, which the board
 *  does not have. TBC by measurement in S2b.
 *
 *  dipole = intensity × coil_factor, coil_factor = Σ N·A [turns·m²] */
#define MTQ_COIL_FACTOR_X   0.105870  /**< 2 × 0.052935 */
#define MTQ_COIL_FACTOR_Y   0.080321  /**< 0.028066 + 0.052255 */
#define MTQ_COIL_FACTOR_Z   0.105870  /**< 2 × 0.052935 */

/** Coil resistance per axis at 20 °C [Ω] (series pair, as above) */
#define MTQ_COIL_R_OHM_X    43.32
#define MTQ_COIL_R_OHM_Y    34.30
#define MTQ_COIL_R_OHM_Z    43.32

/** DRV8214 voltage regulation (REG_CTRL = 11b, VM_GAIN_SEL = 1b).
 *  The bridge regulates the coil terminal voltage to WSET_VSET × LSB,
 *  0–3.92 V in 255 steps (datasheet SLVSH04 Table 8-23). The coils run
 *  open loop: current = V / R, so R(T) is a gain error (copper
 *  +0.39 %/K). VM is 3V3_PERM; the setpoint is capped below VM to keep
 *  headroom for the bridge (≈0.6 Ω HS + LS) and rail tolerance. */
#define MTQ_VSET_LSB_V      (3.92 / 255.0)  /**< ≈ 15.37 mV per code */
#define MTQ_VSET_MAX_CODE   201             /**< 3.09 V of 3.3 V VM */

/** Current per code and maximum current per axis [mA] */
#define MTQ_INTENSITY_STEP_MA_X (MTQ_VSET_LSB_V / MTQ_COIL_R_OHM_X * 1.0e3)
#define MTQ_INTENSITY_STEP_MA_Y (MTQ_VSET_LSB_V / MTQ_COIL_R_OHM_Y * 1.0e3)
#define MTQ_INTENSITY_STEP_MA_Z (MTQ_VSET_LSB_V / MTQ_COIL_R_OHM_Z * 1.0e3)
#define MTQ_MAX_INTENSITY_MA_X  (MTQ_VSET_MAX_CODE * MTQ_INTENSITY_STEP_MA_X)
#define MTQ_MAX_INTENSITY_MA_Y  (MTQ_VSET_MAX_CODE * MTQ_INTENSITY_STEP_MA_Y)
#define MTQ_MAX_INTENSITY_MA_Z  (MTQ_VSET_MAX_CODE * MTQ_INTENSITY_STEP_MA_Z)

/** Maximum magnetic dipole per axis [A·m²]
 *  X, Z ≈ 71.3 mA × 0.1059 ≈ 7.55e-3;  Y ≈ 90.1 mA × 0.0803 ≈ 7.24e-3.
 *  (The BD2606MVV-era model assumed 150 mA: 15.9e-3 / 8.4e-3.) */
#define MTQ_MAX_DIPOLE_X    (MTQ_MAX_INTENSITY_MA_X * 1.0e-3 * MTQ_COIL_FACTOR_X)
#define MTQ_MAX_DIPOLE_Y    (MTQ_MAX_INTENSITY_MA_Y * 1.0e-3 * MTQ_COIL_FACTOR_Y)
#define MTQ_MAX_DIPOLE_Z    (MTQ_MAX_INTENSITY_MA_Z * 1.0e-3 * MTQ_COIL_FACTOR_Z)

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
 *  Reduced to 5e-5 (from 1e-3) to make the controller damping-heavy.
 *  Critical damping condition for the magnetic PD plant is
 *      kR_crit = 2·sqrt(2·J·kP)/|B|
 *  With J ≈ 1.3e-4 kg·m², |B| ≈ 30 µT, kR = 1.9e-2 (fixed):
 *      kP = 1e-3  → ζ ≈ 5e-4  (1700× under-damped)
 *      kP = 5e-5  → ζ ≈ 7e-3  (still under-damped but ~14× better)
 *  Lowering kP cannot make critical damping feasible (would require
 *  kP ≈ 1e-9, far below dead-zone), but it removes the over-stiff P-term
 *  that previously fed energy into the body faster than kR could remove
 *  it, producing the limit cycle and 90° excursions seen at 30 orbits.
 *
 *  At 20° pointing error this gives moment ~ kP·sin(20°) ≈ 1.7e-5 A·m²
 *  on the proportional term — below the 5e-4 A·m² dead zone, so the
 *  rate damping term is the dominant driver and the proportional term
 *  acts only on top of it.  This matches the design philosophy of the
 *  MATLAB reference (kP_MATLAB = 4.4e-5). */
#define NADIR_KP    5.0e-5

/** Rate damping gain kR for magnetic control law [A·m²·s/rad]
 *
 *  With orbital rate compensation, kR damps only the deviation from
 *  orbit rate.  Value matches the MATLAB PIDMIMO calibration; together
 *  with NADIR_KP = 5e-5 (lower than original 1e-3) this gives effective
 *  damping ratio ζ ≈ 0.65 against the 2-DOF zenith-referenced target.
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
 * Sun Sensors
 * ====================================================================== */

/** Sun-visibility threshold on the brightest of the six CSS faces [V].
 *  The brightest face sees at least cos(54.7°) = 1/√3 of the full-Sun
 *  output. 0.1 V is 5σ of the 20 mV simulation noise; the full-Sun output
 *  of the SLCD-61N8 + 2 kΩ TIA stage is not yet measured, so this is
 *  TBC with the Sun simulator in S6. */
#define ADCS_CSS_SUN_THRESHOLD_V    0.1

/* =========================================================================
 * Control Loop Timing
 * ====================================================================== */

/** Default control loop timestep [s]
 *  Source: Sim_time_parameters.m → dT = 1 */
#define ADCS_CONTROL_DT     1.0

/* =========================================================================
 * Orbit / Environment Models
 * ====================================================================== */

/** IGRF-14 truncation degree used on board (1 = tilted dipole … 13 = full).
 *
 *  Chosen by the S3 trade study (docs/adcs_orbit_env_models.md §5): the
 *  smallest degree whose field-direction error vs degree 13 is ≤ 0.5° at
 *  the 95th percentile over 450–550 km SSO orbits (n = 7: 0.25°, max 0.60°)
 *  and whose closed-loop nadir APE is indistinguishable from degree 13.
 *  Costs 35 of the 104 (n, m) terms. A tilted dipole (n = 1) is off by
 *  21° at the 95th percentile. */
#define ADCS_IGRF_ORDER         7

/** TLE age beyond which ephemeris outputs are flagged ADCS_EPH_TLE_STALE [s].
 *  Reported only; the ADCS task decides what to do with it. */
#define ADCS_TLE_MAX_AGE_S      (14.0 * 86400.0)

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
