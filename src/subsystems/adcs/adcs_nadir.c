/**
 * @file adcs_nadir.c
 * @brief Nadir pointing controller using magnetic control law.
 *
 * Algorithm overview:
 *   1. Run TRIAD attitude determination to estimate q_eci_body
 *   2. Compute target quaternion: q_target = U2Q(nadir_eci, body_axis)
 *   3. Compute error quaternion: q_err = conj(q_est) * q_target
 *   4. Enforce positive scalar part of q_err
 *   5. Extract ε (vector part of error quaternion)
 *   6. Compute moment:
 *        m = (kP * cross(B_body, ε) - kR * cross(B_body, ω)) / ||B_body||
 *   7. Clamp dipole and quantize intensity via mtq_compute_command()
 *
 * MATLAB reference:
 *   ref/H-bridge-simulations/Simulation Nadir Pointing/Nadir_pointing.m
 *     L100-103: Sensor readout
 *     L109-147: TRIAD attitude determination (sun/eclipse branches)
 *     L152:     q_target = U2Q(p.eci_vector, p.body_vector)
 *     L156:     q_target_body = QMult(QPose(quaternion), q_target)
 *     L164-166: Enforce positive scalar part
 *     L180:     moment = (kP*cross(B, q_err_vec) - kR*cross(B, ω)) / norm(B)
 *     L198-238: Intensity quantization (0.5-32 mA, 0.5 mA step)
 *
 *   ref/H-bridge-simulations/Simulation Nadir Pointing/Config/Sim_PID_controller.m
 *     L21-23:   p.eci_vector (initial = nadir, overwritten to zenith in main loop)
 *     L25-54:   kP, kR gain derivation via PIDMIMO
 *
 * NOTE: The MATLAB reference uses zenith (= +r_sat/||r_sat||) as eci_vector
 *   (Nadir_pointing.m L244: p.eci_vector = x(1:3)/norm(x(1:3))),
 *   NOT nadir. Due to the quaternion kinematics sign convention
 *   (dq/dt = -0.5*[0,ω]⊗q), the B-cross control law repels body_z
 *   from eci_vector. Using zenith pushes body_z toward nadir.
 *
 * The magnetic control law projects the desired torque onto the
 * B-field plane, which is the only torque direction achievable with
 * magnetorquers. This approach is standard for satellite magnetic
 * attitude control (Wisniewski & Blanke, 1999).
 */

#include "adcs_nadir.h"
#include "adcs_determination.h"
#include "adcs_magnetorquer.h"
#include "adcs_config.h"
#include "adcs_math.h"
#include <math.h>

void nadir_init(adcs_state_t *state)
{
    state->mag_field_prev = vec3d_zero();
    state->b_field_eci_prev = vec3d_zero();
    state->q_eci_body = quat_identity();
    state->step_count = 0;
}

void nadir_step(adcs_state_t *state)
{
    /* Step 1: Attitude determination (TRIAD)
     * Ref: Nadir_pointing.m L109-147 */
    determination_update(state);

    /* Step 2: Target quaternion — rotation from zenith ECI to body axis
     * Ref: Nadir_pointing.m L244
     *   p.eci_vector = x(1:3)/norm(x(1:3))  (zenith = +r_sat / ||r_sat||)
     *   p.body_vector = [0;0;1]              (Z body axis)
     *
     * The B-cross control law with kinematic equation dq/dt = -0.5*[0,ω]⊗q
     * REPELS body_z from eci_vector. Using zenith as eci_vector therefore
     * pushes body_z toward nadir, achieving Earth-pointing.
     * See: docs/adcs_nadir_pointing_analysis.md for full derivation. */
    vec3d_t zenith_eci = vec3d_scale(vec3d_normalize(state->nadir_eci), -1.0);
    vec3d_t body_axis = vec3d_make(NADIR_BODY_AXIS_X,
                                   NADIR_BODY_AXIS_Y,
                                   NADIR_BODY_AXIS_Z);
    quat_t q_target = quat_from_two_vectors(zenith_eci, body_axis);

    /* Step 3: Error quaternion
     * Ref: Nadir_pointing.m L156
     *   q_target_body = QMult(QPose(quaternion), q_target)
     *
     * MATLAB QMult(Q2, Q1) computes Hamilton product Q1 ⊗ Q2.
     * So QMult(QPose(q_est), q_target) = q_target ⊗ conj(q_est).
     * This gives the rotation from current body to desired body,
     * with the error axis expressed in the current body frame. */
    quat_t q_est_conj = quat_conjugate(state->q_eci_body);
    quat_t q_err = quat_multiply(q_target, q_est_conj);

    /* Step 4: Enforce positive scalar part
     * Ref: Nadir_pointing.m L164-166 */
    q_err = quat_positive_scalar(q_err);

    /* Step 5: Extract vector part ε (attitude error in body frame)
     * ε = [q_err.x, q_err.y, q_err.z] */
    vec3d_t epsilon = quat_vector_part(q_err);

    /* Step 6: Magnetic control law
     * Ref: Nadir_pointing.m L180
     *   moment = (kP * cross(B, ε) - kR * cross(B, ω)) / norm(B) */
    vec3d_t b_body = state->mag.field;
    vec3d_t omega  = state->gyro.angular_vel;

    double b_norm = vec3d_norm(b_body);
    if (b_norm < 1.0e-12) {
        /* Invalid magnetometer reading: command zero dipole */
        state->mtq_cmd.dipole = vec3d_zero();
        state->mtq_cmd.intensity_ma = vec3d_zero();
        return;
    }

    vec3d_t b_cross_eps = vec3d_cross(b_body, epsilon);
    vec3d_t b_cross_w   = vec3d_cross(b_body, omega);

    vec3d_t desired_dipole;
    desired_dipole.x = (NADIR_KP * b_cross_eps.x - NADIR_KR * b_cross_w.x) / b_norm;
    desired_dipole.y = (NADIR_KP * b_cross_eps.y - NADIR_KR * b_cross_w.y) / b_norm;
    desired_dipole.z = (NADIR_KP * b_cross_eps.z - NADIR_KR * b_cross_w.z) / b_norm;

    /* Step 7: Clamp and quantize via MTQ driver
     * Ref: Nadir_pointing.m L198-238 */
    mtq_compute_command(desired_dipole, &state->mtq_cmd);

    /* Update previous readings for next step derivative */
    state->mag_field_prev = state->mag.field;
    state->b_field_eci_prev = state->b_field_eci;

    state->step_count++;
}
