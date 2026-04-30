/**
 * @file adcs_nadir.c
 * @brief Nadir pointing controller using magnetic control law.
 *
 * Algorithm overview:
 *   1. Run TRIAD attitude determination to estimate q_eci_body
 *   2. Compute 2-DOF target quaternion: q_target = U2Q(zenith_eci, body_z)
 *      Magnetic actuation is rank-2 (torque is always ⊥ B), so only two
 *      attitude DOF can be controlled at any instant; yaw about the
 *      Earth-pointing axis is uncontrollable and must be left free.
 *      Reference: Nadir_pointing.m L152.
 *   3. Compute error quaternion: q_err = q_target ⊗ conj(q_est)
 *   4. Enforce positive scalar part of q_err
 *   5. Extract ε (vector part of error quaternion)
 *   6. Compute orbit-rate-compensated body angular velocity:
 *        ω_orbit_eci = (r × v) / |r|²
 *        ω̃ = ω_gyro − bias_est − R(q_est) · ω_orbit_eci
 *      so the rate-damping term drives ω toward the LVLH orbit rate
 *      (≈ 0.063 °/s) instead of zero, eliminating the orbit-period
 *      limit cycle caused by D-term fighting against the rotation
 *      needed to track the (rotating) Earth-pointing direction.
 *   7. Compute moment:
 *        m = ( kP · (B_body × ε) − kR · (B_body × ω̃) ) / ||B_body||
 *   8. Clamp dipole and quantize intensity via mtq_compute_command()
 *
 * Improvement over the MATLAB reference (`Nadir_pointing.m`):
 *   MATLAB damps ω toward zero in body/ECI frame (no orbit-rate
 *   compensation), which produces an orbit-period limit cycle because
 *   nadir tracking *requires* body rotation at orbital rate.  This C
 *   implementation keeps the same 2-DOF target + B-cross PD law but
 *   subtracts the body-frame orbit rate from the damping reference.
 *
 *   An earlier revision tried a 3-DOF LVLH target on top of the orbit-
 *   rate compensation; that was strictly worse because the third DOF
 *   (yaw about nadir) is not controllable with magnetorquers, so the
 *   error vector ε contained uncontrollable yaw content that the
 *   controller chased with spurious torques.  Reverted.
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
    state->omega_filtered = vec3d_zero();
    state->gyro_bias_est = vec3d_zero();
    state->bias_update_count = 0;
    state->q_eci_body = quat_identity();
    state->q_triad_prev = quat_identity();
    state->step_count = 0;
    mtq_reset_accumulator();
}

void nadir_step(adcs_state_t *state)
{
    /* Step 1: Attitude determination + gyro bias estimation
     *
     * Sunlit: TRIAD gives full 3-axis attitude.  Estimate gyro bias by
     *   comparing gyro reading to TRIAD-derived angular velocity:
     *     ω_triad ≈ 2·vec(q_triad ⊗ conj(q_triad_prev)) / dt
     *     bias_est = EMA(ω_gyro - ω_triad)
     *
     * Eclipse: propagate attitude with bias-corrected gyro:
     *     ω_corrected = ω_gyro - bias_est
     *   This reduces eclipse drift from ~21° (uncorrected bias) to
     *   ~1° (residual from ARW and RRW between calibrations).
     *
     * Magnetometer correction (complementary filter) fixes remaining
     * drift perpendicular to B:
     *   δθ_⊥ = (B_pred × B_meas) / |B|²
     *   q = (1, K/2·δθ_⊥) ⊗ q  */
    if (state->sun.sun_visible) {
        determination_update(state);

        /* Estimate gyro bias from TRIAD vs gyro comparison.
         * ω_triad = 2 * vec(q_current ⊗ conj(q_prev)) / dt
         * Only valid after first sunlit TRIAD establishes q_triad_prev. */
        if (state->step_count > 1 && state->q_triad_prev.w != 1.0) {
            quat_t dq = quat_multiply(state->q_eci_body,
                                       quat_conjugate(state->q_triad_prev));
            dq = quat_positive_scalar(dq);

            /* Rate-based small-angle gate, scaled by control period dt:
             *   dq.w > cos(0.5 * ω_max * dt)
             * with ω_max = 0.087 rad/s (≈5°/s) — same physical rate
             * threshold the old `dq.w > 0.999` gate enforced at dt = 1s,
             * but now valid at any dt.  This keeps TRIAD noise spikes
             * (which appear as fast pseudo-rotations in dq) out of the
             * bias estimate while still admitting normal post-detumble
             * dynamics, and importantly does NOT spuriously reject
             * samples when the controller runs at slower dt. */
            const double bias_gate_omega_max = 0.087; /* rad/s */
            double gate_thresh = cos(0.5 * bias_gate_omega_max * state->dt);
            if (dq.w > gate_thresh) {
                vec3d_t omega_triad;
                omega_triad.x = -2.0 * dq.x / state->dt;
                omega_triad.y = -2.0 * dq.y / state->dt;
                omega_triad.z = -2.0 * dq.z / state->dt;

                /* Hybrid bias estimator: 1/N running mean for fast startup,
                 * transitioning to fixed α=0.001 EMA for steady-state noise
                 * rejection.  N_warmup = 1000 valid samples (≈ 1000 s of
                 * sunlit time, roughly 0.3 orbit cumulative).
                 *
                 *   α_eff = max(1/(N+1), 0.001)
                 *
                 * Early on (N < 1000): α_eff = 1/(N+1) gives an unbiased
                 * running mean that converges quickly when starting from
                 * zero — no slow exponential ramp-up that would leave the
                 * bias estimate badly wrong for the first 5 orbits.
                 * After N >= 1000: α_eff floors at 0.001 (τ ≈ 1000 s) so
                 * per-sample TRIAD noise (~0.1°/s on each ω_triad
                 * component, dominated by 1/dt differentiation) keeps
                 * averaging out at the noise-rejection rate we want.
                 * Gyro bias drift is sub-µrad/s², so this estimator is
                 * still much faster than the actual bias dynamics. */
                vec3d_t bias_meas = vec3d_sub(state->gyro.angular_vel, omega_triad);
                state->bias_update_count++;
                double alpha = 1.0 / (double)state->bias_update_count;
                if (alpha < 0.001) alpha = 0.001;
                state->gyro_bias_est = vec3d_add(
                    state->gyro_bias_est,
                    vec3d_scale(vec3d_sub(bias_meas, state->gyro_bias_est), alpha));
            }
        }
        state->q_triad_prev = state->q_eci_body;
    } else {
        /* Eclipse: propagate with bias-corrected gyro */
        vec3d_t omega_corrected = vec3d_sub(state->gyro.angular_vel,
                                             state->gyro_bias_est);
        state->q_eci_body = quat_gyro_propagate(
            state->q_eci_body, omega_corrected, state->dt);

        /* Magnetometer-aided correction: fix remaining drift ⊥ to B */
        vec3d_t b_pred = quat_rotate_vec(state->q_eci_body, state->b_field_eci);
        vec3d_t b_meas = state->mag.field;
        double b2 = vec3d_dot(b_meas, b_meas);
        if (b2 > 1e-24) {
            vec3d_t dtheta = vec3d_scale(
                vec3d_cross(b_pred, b_meas), 1.0 / b2);
            double K_mag = 0.05;
            double half_K = 0.5 * K_mag;
            quat_t q_corr = {1.0, half_K * dtheta.x,
                                  half_K * dtheta.y,
                                  half_K * dtheta.z};
            state->q_eci_body = quat_normalize(
                quat_multiply(q_corr, state->q_eci_body));
        }
    }

    /* Step 2: Compute yaw-invariant 2-DOF reduced-attitude error.
     *
     * IMPORTANT: a previous implementation built `q_target` from
     * `quat_from_two_vectors(zenith_eci, body_axis)` and then used
     * `ε = vector(q_target ⊗ conj(q_est))`.  That is NOT yaw-invariant:
     * `quat_from_two_vectors` returns ONE arbitrary minimum-rotation
     * quaternion among an entire yaw-family of valid attitudes, so
     * even at a perfectly nadir-pointing body with arbitrary yaw the
     * resulting `ε` is generally nonzero — the controller injects a
     * spurious P-command at the supposed equilibrium and the closed
     * loop slowly drifts away from a perfect nadir IC.
     *
     * Correct reduced-attitude error (yaw-invariant by construction):
     *   nadir_body = R(q_eci_body) · n̂_eci   (nadir direction in body)
     *   ε = ẑ_body × nadir_body
     *
     * Properties:
     *   - ε.z ≡ 0  (pure pitch/roll error, as required for 2-DOF)
     *   - |ε| = sin(θ) where θ is the angle between ẑ_body and nadir
     *   - ε direction is the rotation axis that brings ẑ_body to nadir
     *   - INVARIANT under any rotation about ẑ_body (i.e. any yaw)
     *
     * Sign: ε points along the angular-velocity direction needed to
     * align ẑ_body with nadir_body, so the B-cross law
     *   m = +kP·(B×ε)/|B|   →   τ = m×B ≈ +kP·|B|·(ε⊥B)
     * drives ẑ_body toward nadir as desired.  This is the same sign
     * convention as `Nadir_pointing.m` L180 in the MATLAB reference
     * once that reference's `quaternion error` is reduced to its
     * yaw-invariant projection. */
    vec3d_t nadir_eci_unit = vec3d_normalize(state->nadir_eci);
    vec3d_t nadir_body = quat_rotate_vec(state->q_eci_body, nadir_eci_unit);
    vec3d_t body_z_axis = vec3d_make(NADIR_BODY_AXIS_X,
                                     NADIR_BODY_AXIS_Y,
                                     NADIR_BODY_AXIS_Z);
    vec3d_t epsilon = vec3d_cross(body_z_axis, nadir_body);

    /* Step 6: Magnetic control law with orbit-rate-compensated damping.
     *   m = ( +kP·B×ε − kR·B×ω̃ ) / |B|
     *
     * Sign convention: with the 2-DOF zenith-referenced target above and
     * the codebase's quaternion conventions, the τ = m×B torque produced
     * by +kP drives body_z toward the OPPOSITE of zenith, i.e. toward
     * nadir.  This is the same form as the MATLAB reference
     * (Nadir_pointing.m L180).
     *
     * ω̃ = ω_gyro − bias_est − R(q_est)·ω_orbit_eci removes the orbit-
     * period limit cycle: the body must rotate at ~0.063 °/s to keep
     * body_z on the (rotating) Earth-pointing direction, so the D-term
     * is referenced to that rotation rather than to zero.  This is the
     * single algorithmic change vs. the MATLAB reference. */
    vec3d_t b_body = state->mag.field;
    double b_norm = vec3d_norm(b_body);

    if (b_norm < 1.0e-12 || state->step_count == 0) {
        state->mtq_cmd.dipole = vec3d_zero();
        state->mtq_cmd.intensity_ma = vec3d_zero();
        state->step_count++;
        return;
    }

    /* Instantaneous orbit angular velocity in ECI: ω_orbit = (r × v) / |r|².
     * Rotated into body frame and subtracted from gyro so the rate-damping
     * term drives ω toward the LVLH orbit rate (~0.063 °/s) instead of
     * zero. Without this the D-term fights the rotation needed to keep
     * body_z on the moving Earth-pointing direction. */
    double r2 = vec3d_dot(state->pos_eci, state->pos_eci);
    vec3d_t omega_orbit_body = vec3d_zero();
    if (r2 > 1.0) {
        vec3d_t omega_orbit_eci = vec3d_scale(
            vec3d_cross(state->pos_eci, state->vel_eci), 1.0 / r2);
        omega_orbit_body = quat_rotate_vec(state->q_eci_body, omega_orbit_eci);
    }

    vec3d_t omega_corr = vec3d_sub(state->gyro.angular_vel, state->gyro_bias_est);
    vec3d_t omega_tilde = vec3d_sub(omega_corr, omega_orbit_body);

    vec3d_t b_cross_eps = vec3d_cross(b_body, epsilon);
    vec3d_t b_cross_omega = vec3d_cross(b_body, omega_tilde);

    vec3d_t desired_dipole;
    desired_dipole.x = (NADIR_KP * b_cross_eps.x - NADIR_KR * b_cross_omega.x) / b_norm;
    desired_dipole.y = (NADIR_KP * b_cross_eps.y - NADIR_KR * b_cross_omega.y) / b_norm;
    desired_dipole.z = (NADIR_KP * b_cross_eps.z - NADIR_KR * b_cross_omega.z) / b_norm;

    /* Step 7: Clamp and quantize via MTQ driver */
    mtq_compute_command(desired_dipole, &state->mtq_cmd);

    state->step_count++;
}
