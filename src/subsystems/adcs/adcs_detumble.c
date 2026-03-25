/**
 * @file adcs_detumble.c
 * @brief Proportional B-DOT detumbling controller with adaptive saturation.
 *
 * Algorithm:
 *   1. Read magnetometer data → B_body (magnetic field in body frame)
 *   2. Compute time derivative: dB/dt = (B_body - B_body_prev) / dT
 *   3. Compute adaptive gain: k = BDOT_GAIN_COEFF / dT
 *   4. Apply proportional B-DOT law with per-axis saturation:
 *        m_raw[i] = -k * dB_dt[i]
 *        m[i]     = clamp(m_raw[i], -m_max[i], +m_max[i])
 *   5. Quantize intensity via mtq_compute_command()
 *   6. Check gyroscope: if |ω| < threshold for sustained period → done
 *
 * At high angular velocity, |k × dB/dt| exceeds m_max and the law
 * saturates — behaving identically to the sign-based bang-bang law.
 * At low angular velocity, the output is proportional to dB/dt,
 * avoiding the discrete-time overshoot that occurs when bang-bang
 * torque impulse exceeds the current angular momentum.
 *
 * The gain k = BDOT_GAIN_COEFF / ΔT is derived from orbit-averaged
 * discrete stability analysis (see docs/adcs_detumble_high_rate_analysis.md §9):
 *   BDOT_GAIN_COEFF = λ × (3/2) × I_avg / B₀²
 * where λ (BDOT_GAIN_LAMBDA) is the per-step damping ratio.
 *
 * MATLAB reference:
 *   ref/PoCat-Lektron-ADCS/ADCS/Detumbling.m
 *     L107-117:  Sensor readout
 *     L119-135:  B-DOT derivative and sign-based moment computation
 *     L144-184:  Intensity quantization (0.5-32 mA, 0.5 mA step)
 *     L192-214:  Gyroscope readout for threshold check
 *
 *   ref/PoCat-Lektron-ADCS/ADCS/utils/CubeSatSimulation.m
 *     L137:      d.k = 10^-5   (proportional gain reference)
 *     L166-170:  Proportional variant (commented out in MATLAB)
 */

#include "adcs_detumble.h"
#include "adcs_magnetorquer.h"
#include "adcs_config.h"
#include "adcs_math.h"
#include <math.h>

/** Maximum dipole per axis for sign-based law [A·m²] */
static const double max_moment[3] = {
    MTQ_MAX_DIPOLE_X,
    MTQ_MAX_DIPOLE_Y,
    MTQ_MAX_DIPOLE_Z
};

void detumble_init(adcs_state_t *state)
{
    state->mag_field_prev = vec3d_zero();
    state->detumble_stable_count = 0;
    state->step_count = 0;
}

double detumble_select_dt(const adcs_state_t *state)
{
    double omega = vec3d_norm(state->gyro.angular_vel);

    /* Guard: if ω ≈ 0, use maximum period (avoid division by zero) */
    if (omega < 1.0e-6) {
        return DETUMBLE_DT_CEIL;
    }

    /* Eq. 32: ΔT = SNR_MIN / (SNR_COEFF × ω)
     * where SNR_COEFF = B₀ / (√2 × σ_mag)  ≈ 530.3
     * This keeps dB/dt SNR ≥ SNR_MIN at all angular velocities. */
    double dt = DETUMBLE_SNR_MIN / (DETUMBLE_SNR_COEFF * omega);

    /* Clamp to [DT_FLOOR, DT_CEIL] */
    if (dt < DETUMBLE_DT_FLOOR) dt = DETUMBLE_DT_FLOOR;
    if (dt > DETUMBLE_DT_CEIL)  dt = DETUMBLE_DT_CEIL;

    return dt;
}

int detumble_step(adcs_state_t *state)
{
    vec3d_t b_body = state->mag.field;
    vec3d_t b_prev = state->mag_field_prev;
    double dt = state->dt;

    /* Step 1: Compute magnetic field derivative in body frame
     * Ref: Detumbling.m L123 — magbodyderivative = (d.bFieldBody - d.bFieldBodyBefore) / dT */
    vec3d_t db_dt;
    db_dt.x = (b_body.x - b_prev.x) / dt;
    db_dt.y = (b_body.y - b_prev.y) / dt;
    db_dt.z = (b_body.z - b_prev.z) / dt;

    /* Handle zero derivative (first step) to avoid commanding zero moment */
    if (fabs(db_dt.x) < 1.0e-15 && fabs(db_dt.y) < 1.0e-15 && fabs(db_dt.z) < 1.0e-15) {
        db_dt.x = 1.0e-9;
        db_dt.y = 1.0e-9;
        db_dt.z = 1.0e-9;
    }

    /* Step 2: Proportional B-DOT law with per-axis saturation
     *
     * m_raw[i] = −k × dB_dt[i]
     * m[i]     = clamp(m_raw[i], −m_max[i], +m_max[i])
     *
     * Fixed gain: k = BDOT_GAIN_K = m_max_ref / (ω_sat × B₀)
     *   At high ω: |k × dB/dt| > m_max → saturates → equivalent to bang-bang
     *   At low ω:  |k × dB/dt| < m_max → proportional → no overshoot
     *
     * Ref: CubeSatSimulation.m L137,166-170 (proportional variant)
     */
    double k = BDOT_GAIN_K;

    vec3d_t desired_dipole;
    desired_dipole.x = -k * db_dt.x;
    desired_dipole.y = -k * db_dt.y;
    desired_dipole.z = -k * db_dt.z;

    /* Per-axis saturation to hardware limits (also done inside
     * mtq_compute_command, but explicit here for clarity) */
    if (fabs(desired_dipole.x) > max_moment[0])
        desired_dipole.x = (desired_dipole.x >= 0.0) ? max_moment[0] : -max_moment[0];
    if (fabs(desired_dipole.y) > max_moment[1])
        desired_dipole.y = (desired_dipole.y >= 0.0) ? max_moment[1] : -max_moment[1];
    if (fabs(desired_dipole.z) > max_moment[2])
        desired_dipole.z = (desired_dipole.z >= 0.0) ? max_moment[2] : -max_moment[2];

    /* Step 3: Quantize intensity via MTQ driver
     * Ref: Detumbling.m L144-184 */
    mtq_compute_command(desired_dipole, &state->mtq_cmd);

    /* Step 4: Update previous magnetic field for next step */
    state->mag_field_prev = b_body;

    /* Step 5: Check angular velocity threshold
     * Ref: Detumbling.m — exit when |ω| < 0.017 rad/s ≈ 1°/s */
    double omega_mag = vec3d_norm(state->gyro.angular_vel);

    if (omega_mag < DETUMBLE_OMEGA_THRESHOLD) {
        state->detumble_stable_count++;
    } else {
        state->detumble_stable_count = 0;
    }

    state->step_count++;

    /* Detumbling complete when stable for sustained period */
    return (state->detumble_stable_count >= DETUMBLE_STABLE_COUNT) ? 1 : 0;
}
