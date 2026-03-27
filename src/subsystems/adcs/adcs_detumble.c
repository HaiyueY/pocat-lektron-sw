/**
 * @file adcs_detumble.c
 * @brief Gyroscope-based proportional detumbling controller (ω×B law).
 *
 * Algorithm:
 *   1. Read gyroscope → ω_body, magnetometer → B_body
 *   2. Compute ω × B_body (equivalent to ideal -dB_body/dt)
 *   3. Apply proportional law with per-axis saturation:
 *        m_raw[i] = k * (ω × B)_i
 *        m[i]     = clamp(m_raw[i], -m_max[i], +m_max[i])
 *   4. Quantize intensity via mtq_compute_command()
 *   5. Check gyroscope: if |ω| < threshold for sustained period → done
 *
 * The ω×B formulation is mathematically equivalent to the continuous
 * B-DOT law (since dB_body/dt = dB_eci/dt − ω × B ≈ −ω × B when
 * the orbital B-field variation is slow compared to tumbling), but
 * eliminates the finite-difference phase error and sinc attenuation
 * that degrade the classical discrete B-DOT at low control frequencies.
 *
 * This allows reliable detumbling at 1 Hz control rate for initial
 * angular velocities up to 90°/s (per ESA 4× Nyquist rule).
 * The control frequency adapts to the current angular velocity:
 *   f_ctrl = 4 × f_rot = 2|ω|/π, clamped to [0.5 Hz, 1 Hz].
 *
 * At high angular velocity, |k × (ω × B)| exceeds m_max and the law
 * saturates — behaving identically to the sign-based bang-bang law.
 * At low angular velocity, the output is proportional to ω,
 * avoiding the discrete-time overshoot that occurs when bang-bang
 * torque impulse exceeds the current angular momentum.
 *
 * The gain k = m_max_ref / (ω_sat × B₀) is derived from saturation
 * crossover analysis (see docs/adcs_detumble_high_rate_analysis.md §9).
 *
 * MATLAB reference (original finite-difference version):
 *   ref/PoCat-Lektron-ADCS/ADCS/Detumbling.m L119-135
 *   This C implementation upgrades to ω×B using the available gyroscope.
 */

#include "adcs_detumble.h"
#include "adcs_magnetorquer.h"
#include "adcs_config.h"
#include "adcs_math.h"
#include <math.h>

/** Maximum dipole per axis [A·m²] */
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
    /* ESA 4× Nyquist rule: f_ctrl = 4 × f_rot = 2|ω|/π
     * → ΔT = π / (2|ω|), clamped to [DT_MIN, DT_MAX].
     * Maintains constant ZOH efficiency G = sinc(π/4) ≈ 0.900. */
    double omega_mag = vec3d_norm(state->gyro.angular_vel);

    if (omega_mag < 1e-6)
        return DETUMBLE_DT_MAX;

    double dt = M_PI / (2.0 * omega_mag);

    if (dt < DETUMBLE_DT_MIN) dt = DETUMBLE_DT_MIN;
    if (dt > DETUMBLE_DT_MAX) dt = DETUMBLE_DT_MAX;
    return dt;
}

int detumble_step(adcs_state_t *state)
{
    vec3d_t omega = state->gyro.angular_vel;
    vec3d_t b_body = state->mag.field;

    /* Step 1: Compute ω × B_body
     *
     * This is equivalent to the ideal (continuous) -dB_body/dt:
     *   dB_body/dt = dB_eci/dt − ω × B ≈ −ω × B
     * (orbital B variation is negligible compared to tumbling)
     *
     * So: m = -k × dB/dt = k × (ω × B)
     */
    vec3d_t omega_cross_b = vec3d_cross(omega, b_body);

    /* Step 2: Proportional law with per-axis saturation
     *
     * m_raw[i] = k × (ω × B)_i
     * m[i]     = clamp(m_raw[i], −m_max[i], +m_max[i])
     *
     * Gain: k = BDOT_GAIN_K = m_max_ref / (ω_sat × B₀)
     *   At high ω: |k × (ω × B)| > m_max → saturates → bang-bang equivalent
     *   At low ω:  proportional → smooth convergence, no overshoot
     */
    double k = BDOT_GAIN_K;

    vec3d_t desired_dipole;
    desired_dipole.x = k * omega_cross_b.x;
    desired_dipole.y = k * omega_cross_b.y;
    desired_dipole.z = k * omega_cross_b.z;

    /* Per-axis saturation to hardware limits */
    if (fabs(desired_dipole.x) > max_moment[0])
        desired_dipole.x = (desired_dipole.x >= 0.0) ? max_moment[0] : -max_moment[0];
    if (fabs(desired_dipole.y) > max_moment[1])
        desired_dipole.y = (desired_dipole.y >= 0.0) ? max_moment[1] : -max_moment[1];
    if (fabs(desired_dipole.z) > max_moment[2])
        desired_dipole.z = (desired_dipole.z >= 0.0) ? max_moment[2] : -max_moment[2];

    /* Step 3: Quantize intensity via MTQ driver */
    mtq_compute_command(desired_dipole, &state->mtq_cmd);

    /* Step 4: Store B_body for telemetry/diagnostics (no longer used for control) */
    state->mag_field_prev = b_body;

    /* Step 5: Check angular velocity threshold
     * Exit when |ω| < 0.017 rad/s ≈ 1°/s */
    double omega_mag = vec3d_norm(omega);

    if (omega_mag < DETUMBLE_OMEGA_THRESHOLD) {
        state->detumble_stable_count++;
    } else {
        state->detumble_stable_count = 0;
    }

    state->step_count++;

    /* Detumbling complete when stable for sustained period */
    return (state->detumble_stable_count >= DETUMBLE_STABLE_COUNT) ? 1 : 0;
}
