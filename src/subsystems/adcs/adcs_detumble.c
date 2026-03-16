/**
 * @file adcs_detumble.c
 * @brief Sign-based B-DOT detumbling controller implementation.
 *
 * Algorithm:
 *   1. Read magnetometer data → B_body (magnetic field in body frame)
 *   2. Compute time derivative: dB/dt = (B_body - B_body_prev) / dT
 *   3. Apply sign-based B-DOT law:
 *        moment[i] = -max_moment[i] * sign(dB_dt[i])
 *   4. Clamp and quantize intensity via mtq_compute_command()
 *   5. Check gyroscope: if |ω| < threshold for sustained period → done
 *
 * MATLAB reference:
 *   ref/PoCat-Lektron-ADCS/ADCS/Detumbling.m
 *     L107-117:  Sensor readout
 *     L119-135:  B-DOT derivative and sign-based moment computation
 *     L144-184:  Intensity quantization (0.5-32 mA, 0.5 mA step)
 *     L192-214:  Gyroscope readout for threshold check
 *
 *   ref/PoCat-Lektron-ADCS/ADCS/SimParameters/Sim_data_structure.m
 *     d.maxmoment = [32.0, 36.2, 32.0] × 10⁻⁴ A·m²
 *
 * The sign-based law (rather than proportional k*dB/dt) is used because
 * it always commands the maximum available torque, which speeds up
 * convergence at the cost of precision — acceptable for detumbling where
 * the goal is to reduce ω below a threshold, not achieve fine pointing.
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

    /* Step 2: Sign-based B-DOT law
     * Ref: Detumbling.m L129-135
     *   signmomx  = sign(magbodyderivative(1));
     *   moment(1) = - d.maxmoment(1) * signmomx;
     */
    vec3d_t desired_dipole;
    desired_dipole.x = -max_moment[0] * ((db_dt.x >= 0.0) ? 1.0 : -1.0);
    desired_dipole.y = -max_moment[1] * ((db_dt.y >= 0.0) ? 1.0 : -1.0);
    desired_dipole.z = -max_moment[2] * ((db_dt.z >= 0.0) ? 1.0 : -1.0);

    /* Step 3: Clamp and quantize via MTQ driver
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
