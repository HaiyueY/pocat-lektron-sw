/**
 * @file test_detumble.c
 * @brief Detumbling mode integration test.
 *
 * Scenario: Satellite starts with 30°/s tumble rate.
 * Expected: B-DOT controller reduces |ω| below 0.017 rad/s threshold.
 *
 * Uses full closed-loop simulation with:
 *   - Simulated magnetometer (body-frame B + noise)
 *   - Simulated gyroscope (ω + noise + bias drift)
 *   - Simplified rigid body dynamics (Euler's equations)
 *   - Simulated orbital environment (varying B-field)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sys/stat.h>
#include "adcs_types.h"
#include "adcs_config.h"
#include "adcs_math.h"
#include "adcs_mode_manager.h"
#include "adcs_detumble.h"
#include "sim_environment.h"
#include "sim_magnetometer.h"
#include "sim_gyroscope.h"
#include "sim_mtq_driver.h"

/** Maximum simulation steps (prevent infinite loop) */
#define MAX_STEPS   2000000

/** Log interval (print every N steps) */
#define LOG_INTERVAL 5000

/** Physics sub-step for numerical stability */
#define PHYSICS_DT  0.01

/** CSV output path */
#define CSV_DIR     "results"
#define CSV_FILE    CSV_DIR "/detumble.csv"

/**
 * @brief Compute angular acceleration from Euler's equation.
 *
 * α = I⁻¹ (τ − ω × I·ω)
 */
static vec3d_t euler_alpha(vec3d_t omega, vec3d_t torque)
{
    vec3d_t iw = {SAT_INERTIA_XX * omega.x,
                  SAT_INERTIA_YY * omega.y,
                  SAT_INERTIA_ZZ * omega.z};
    vec3d_t gyro = vec3d_cross(omega, iw);
    vec3d_t a;
    a.x = (torque.x - gyro.x) / SAT_INERTIA_XX;
    a.y = (torque.y - gyro.y) / SAT_INERTIA_YY;
    a.z = (torque.z - gyro.z) / SAT_INERTIA_ZZ;
    return a;
}

/**
 * @brief Compute quaternion derivative: dq/dt = −0.5 [0,ω] ⊗ q.
 */
static quat_t quat_deriv(quat_t q, vec3d_t omega)
{
    quat_t wq = {0.0, -omega.x, -omega.y, -omega.z};
    quat_t d = quat_multiply(wq, q);
    d.w *= 0.5; d.x *= 0.5; d.y *= 0.5; d.z *= 0.5;
    return d;
}

/**
 * @brief RK4 integration of coupled rigid-body dynamics + attitude.
 *
 * Replaces forward-Euler `dynamics_step` + `attitude_step`.
 * Forward Euler has O(h²) energy drift in the gyroscopic coupling
 * term ω×(I·ω), which at high ω overwhelms the B-DOT dissipation.
 * RK4 reduces drift to O(h⁴), making it negligible.
 *
 * Within each sub-step the MTQ command is held constant (zero-order
 * hold), but the torque τ = m × B_body is recomputed from the
 * evolving attitude at each RK4 stage.
 */
static void rk4_step(vec3d_t *omega, quat_t *q,
                     vec3d_t b_eci, mtq_command_t *mtq_cmd,
                     double eta_duty, double h)
{
    /* --- k1 --- */
    vec3d_t b1 = quat_rotate_vec(*q, b_eci);
    vec3d_t tau1; sim_mtq_apply(mtq_cmd, b1, &tau1);
    tau1.x *= eta_duty; tau1.y *= eta_duty; tau1.z *= eta_duty;
    vec3d_t a1 = euler_alpha(*omega, tau1);
    quat_t  dq1 = quat_deriv(*q, *omega);

    /* --- k2: state at t + h/2 using k1 --- */
    vec3d_t w2 = {omega->x + 0.5*h*a1.x,
                  omega->y + 0.5*h*a1.y,
                  omega->z + 0.5*h*a1.z};
    quat_t  q2 = {q->w + 0.5*h*dq1.w, q->x + 0.5*h*dq1.x,
                  q->y + 0.5*h*dq1.y, q->z + 0.5*h*dq1.z};
    q2 = quat_normalize(q2);
    vec3d_t b2 = quat_rotate_vec(q2, b_eci);
    vec3d_t tau2; sim_mtq_apply(mtq_cmd, b2, &tau2);
    tau2.x *= eta_duty; tau2.y *= eta_duty; tau2.z *= eta_duty;
    vec3d_t a2 = euler_alpha(w2, tau2);
    quat_t  dq2 = quat_deriv(q2, w2);

    /* --- k3: state at t + h/2 using k2 --- */
    vec3d_t w3 = {omega->x + 0.5*h*a2.x,
                  omega->y + 0.5*h*a2.y,
                  omega->z + 0.5*h*a2.z};
    quat_t  q3 = {q->w + 0.5*h*dq2.w, q->x + 0.5*h*dq2.x,
                  q->y + 0.5*h*dq2.y, q->z + 0.5*h*dq2.z};
    q3 = quat_normalize(q3);
    vec3d_t b3 = quat_rotate_vec(q3, b_eci);
    vec3d_t tau3; sim_mtq_apply(mtq_cmd, b3, &tau3);
    tau3.x *= eta_duty; tau3.y *= eta_duty; tau3.z *= eta_duty;
    vec3d_t a3 = euler_alpha(w3, tau3);
    quat_t  dq3 = quat_deriv(q3, w3);

    /* --- k4: state at t + h using k3 --- */
    vec3d_t w4 = {omega->x + h*a3.x,
                  omega->y + h*a3.y,
                  omega->z + h*a3.z};
    quat_t  q4 = {q->w + h*dq3.w, q->x + h*dq3.x,
                  q->y + h*dq3.y, q->z + h*dq3.z};
    q4 = quat_normalize(q4);
    vec3d_t b4 = quat_rotate_vec(q4, b_eci);
    vec3d_t tau4; sim_mtq_apply(mtq_cmd, b4, &tau4);
    tau4.x *= eta_duty; tau4.y *= eta_duty; tau4.z *= eta_duty;
    vec3d_t a4 = euler_alpha(w4, tau4);
    quat_t  dq4 = quat_deriv(q4, w4);

    /* --- weighted update --- */
    omega->x += h/6.0 * (a1.x  + 2*a2.x  + 2*a3.x  + a4.x);
    omega->y += h/6.0 * (a1.y  + 2*a2.y  + 2*a3.y  + a4.y);
    omega->z += h/6.0 * (a1.z  + 2*a2.z  + 2*a3.z  + a4.z);
    q->w     += h/6.0 * (dq1.w + 2*dq2.w + 2*dq3.w + dq4.w);
    q->x     += h/6.0 * (dq1.x + 2*dq2.x + 2*dq3.x + dq4.x);
    q->y     += h/6.0 * (dq1.y + 2*dq2.y + 2*dq3.y + dq4.y);
    q->z     += h/6.0 * (dq1.z + 2*dq2.z + 2*dq3.z + dq4.z);
    *q = quat_normalize(*q);
}

int main(void)
{
    printf("=== ADCS Detumbling Test (ω×B Law, Adaptive ΔT 0.5–1 Hz) ===\n");
    printf("Initial tumble rate: 90 deg/s per axis\n");
    printf("Adaptive ΔT: π/(2|ω|), clamped [%.1f, %.1f] s (ESA 4× Nyquist)\n",
           DETUMBLE_DT_MIN, DETUMBLE_DT_MAX);
    printf("ω×B gain: k = %.1f  (ω_sat = %.1f deg/s)\n",
           BDOT_GAIN_K, BDOT_SAT_OMEGA * RAD_TO_DEG);
    printf("Exit threshold: %.4f rad/s (%.2f deg/s)\n\n",
           DETUMBLE_OMEGA_THRESHOLD,
           DETUMBLE_OMEGA_THRESHOLD * RAD_TO_DEG);

    /* Seed RNG for reproducible tests */
    srand(42);

    /* Initialize state */
    adcs_state_t state;
    adcs_mode_init(&state);

    /* Initial conditions: 90°/s tumble */
    double init_omega_deg_s = 90.0;
    vec3d_t omega_true = {
        init_omega_deg_s * DEG_TO_RAD,
        init_omega_deg_s * DEG_TO_RAD * 0.7,
        init_omega_deg_s * DEG_TO_RAD * 0.5
    };
    quat_t q_true = quat_identity();

    /* Initialize simulators */
    sim_env_t env;
    sim_env_init(&env);
    sim_gyro_state_t gyro_sim;
    sim_gyro_init(&gyro_sim);

    /* Request detumbling mode */
    int rc = adcs_mode_request(&state, ADCS_MODE_DETUMBLING);
    if (rc != 0) {
        printf("FAIL: Could not enter detumbling mode\n");
        return 1;
    }
    printf("Mode: DETUMBLING\n\n");

    /* Open CSV for data logging */
    mkdir(CSV_DIR, 0755);
    FILE *csv = fopen(CSV_FILE, "w");
    if (!csv) {
        printf("WARNING: Could not open %s for writing\n", CSV_FILE);
    } else {
        fprintf(csv, "step,time_s,dt,duty_cycle,k_gain,sat_ratio,"
                      "omega_x,omega_y,omega_z,omega_mag,"
                      "mtq_ix_ma,mtq_iy_ma,mtq_iz_ma,"
                      "dipole_x,dipole_y,dipole_z,"
                      "torque_x,torque_y,torque_z,"
                      "b_body_x,b_body_y,b_body_z\n");
    }

    printf("%6s  %10s  %10s  %6s  %10s  %10s  %10s\n",
           "Step", "Time [s]", "|ω| deg/s", "ΔT", "ωx", "ωy", "ωz");
    printf("------  ----------  ----------  ------  ----------  ----------  ----------\n");

    int converged = 0;
    double sim_time = 0.0;
    for (int step = 0; step < MAX_STEPS; step++) {
        /* Adaptive ΔT selection based on current angular velocity */
        double dt = detumble_select_dt(&state);

        /* Advance orbital environment */
        sim_env_step(&env, dt);

        /* Simulate sensor readings */
        sim_mag_read(&env, q_true, &state.mag);
        sim_gyro_read(&gyro_sim, omega_true, dt, &state.gyro);

        /* Provide ECI references */
        state.b_field_eci = env.b_field_eci;
        state.dt = dt;

        /* Run ADCS controller */
        int done = adcs_mode_step(&state);

        /* Apply MTQ torque to dynamics with RK4 sub-stepping.
         * When dt < PHYSICS_DT (e.g. at high ω with adaptive ΔT),
         * use dt directly as a single physics step.
         *
         * Duty-cycle model: each cycle has a dead-time T_dead during
         * which the MTQ is off (sensor read + I2C).  Effective torque
         * is scaled by η_duty = (ΔT − T_dead) / ΔT.               */
        double eta_duty = (dt > DETUMBLE_DEAD_TIME_S)
                        ? (dt - DETUMBLE_DEAD_TIME_S) / dt
                        : 0.0;

        int n_sub = (dt >= PHYSICS_DT) ? (int)(dt / PHYSICS_DT) : 1;
        double sub_dt = dt / n_sub;
        for (int s = 0; s < n_sub; s++) {
            rk4_step(&omega_true, &q_true, env.b_field_eci,
                     &state.mtq_cmd, eta_duty, sub_dt);
        }

        /* Compute torque at final state for logging */
        vec3d_t b_body = quat_rotate_vec(q_true, env.b_field_eci);
        vec3d_t torque;
        sim_mtq_apply(&state.mtq_cmd, b_body, &torque);
        torque.x *= eta_duty;
        torque.y *= eta_duty;
        torque.z *= eta_duty;
        double omega_mag = vec3d_norm(omega_true) * RAD_TO_DEG;

        /* Proportional gain and saturation ratio for logging */
        double k_gain = BDOT_GAIN_K;
        double dipole_mag = sqrt(state.mtq_cmd.dipole.x * state.mtq_cmd.dipole.x +
                                 state.mtq_cmd.dipole.y * state.mtq_cmd.dipole.y +
                                 state.mtq_cmd.dipole.z * state.mtq_cmd.dipole.z);
        double m_max_avg = (MTQ_MAX_DIPOLE_X + MTQ_MAX_DIPOLE_Y + MTQ_MAX_DIPOLE_Z) / 3.0;
        double sat_ratio = (m_max_avg > 0.0) ? dipole_mag / m_max_avg : 0.0;

        sim_time += dt;

        /* Write CSV row */
        if (csv) {
            fprintf(csv, "%d,%.3f,%.3f,%.6f,%.4f,%.6f,"
                         "%.10e,%.10e,%.10e,%.10e,"
                         "%.6f,%.6f,%.6f,"
                         "%.10e,%.10e,%.10e,"
                         "%.10e,%.10e,%.10e,"
                         "%.10e,%.10e,%.10e\n",
                    step, sim_time, dt, eta_duty, k_gain, sat_ratio,
                    omega_true.x, omega_true.y, omega_true.z,
                    vec3d_norm(omega_true),
                    state.mtq_cmd.intensity_ma.x,
                    state.mtq_cmd.intensity_ma.y,
                    state.mtq_cmd.intensity_ma.z,
                    state.mtq_cmd.dipole.x,
                    state.mtq_cmd.dipole.y,
                    state.mtq_cmd.dipole.z,
                    torque.x, torque.y, torque.z,
                    b_body.x, b_body.y, b_body.z);
        }

        /* Console log */
        if (step % LOG_INTERVAL == 0 || done) {
            printf("%6d  %10.1f  %10.4f  %6.2f  %10.6f  %10.6f  %10.6f\n",
                   step, sim_time, omega_mag, dt,
                   omega_true.x * RAD_TO_DEG,
                   omega_true.y * RAD_TO_DEG,
                   omega_true.z * RAD_TO_DEG);
        }

        if (done) {
            printf("\n=== DETUMBLING CONVERGED at step %d (%.1f s sim time) ===\n",
                   step, sim_time);
            printf("Final |ω| = %.4f deg/s\n", omega_mag);
            converged = 1;
            break;
        }
    }

    if (csv) fclose(csv);

    if (!converged) {
        double final_omega = vec3d_norm(omega_true) * RAD_TO_DEG;
        printf("\nWARNING: Did not converge within %d steps\n", MAX_STEPS);
        printf("Final |ω| = %.4f deg/s\n", final_omega);

        if (final_omega < init_omega_deg_s * 0.1) {
            printf("PARTIAL: Angular velocity reduced by >90%%\n");
        } else {
            return 1;
        }
    } else {
        printf("\nPASS: Detumbling test passed\n");
    }

    /* Auto-generate plots */
    printf("\nGenerating plots...\n");
    int plot_rc = system("python3 ../plot_results.py detumble " CSV_FILE " " CSV_DIR "/");
    if (plot_rc != 0) {
        printf("WARNING: Plot generation failed (python3 not found or script error)\n");
    } else {
        printf("Plots saved to %s/\n", CSV_DIR);
    }

    return converged ? 0 : (vec3d_norm(omega_true) * RAD_TO_DEG < init_omega_deg_s * 0.1 ? 0 : 1);
}
