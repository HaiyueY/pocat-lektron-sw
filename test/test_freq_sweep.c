/**
 * @file test_freq_sweep.c
 * @brief Frequency sweep test for detumbling performance.
 *
 * Standalone binary that runs a single detumble simulation with
 * user-specified DT_FLOOR and initial angular velocity.
 *
 * Usage: ./test_freq_sweep <dt_floor_s> <omega_deg_per_axis> [--csv <path>]
 *
 * Output (single line):
 *   CONVERGED <time_s>    — if |ω| < threshold within 12h
 *   DIVERGED  <final_ω>   — if not converged
 *
 * When --csv is given, writes full simulation trajectory (same columns
 * as test_detumble.c) for post-run plotting with plot_results.py.
 *
 * Designed to be called from sweep_freq_omega.sh for parameter sweeps.
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

/** 12-hour simulation time limit */
#define SIM_TIME_LIMIT  43200.0

/** Physics sub-step */
#define PHYSICS_DT  0.01

/* ------------------------------------------------------------------ */
/*  Rigid-body dynamics (same as test_detumble.c)                     */
/* ------------------------------------------------------------------ */

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

static quat_t quat_deriv(quat_t q, vec3d_t omega)
{
    quat_t wq = {0.0, -omega.x, -omega.y, -omega.z};
    quat_t d = quat_multiply(wq, q);
    d.w *= 0.5; d.x *= 0.5; d.y *= 0.5; d.z *= 0.5;
    return d;
}

static void rk4_step(vec3d_t *omega, quat_t *q,
                     vec3d_t b_eci, mtq_command_t *mtq_cmd,
                     double eta_duty, double h)
{
    vec3d_t b1 = quat_rotate_vec(*q, b_eci);
    vec3d_t tau1; sim_mtq_apply(mtq_cmd, b1, &tau1);
    tau1.x *= eta_duty; tau1.y *= eta_duty; tau1.z *= eta_duty;
    vec3d_t a1 = euler_alpha(*omega, tau1);
    quat_t  dq1 = quat_deriv(*q, *omega);

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

    omega->x += h/6.0 * (a1.x  + 2*a2.x  + 2*a3.x  + a4.x);
    omega->y += h/6.0 * (a1.y  + 2*a2.y  + 2*a3.y  + a4.y);
    omega->z += h/6.0 * (a1.z  + 2*a2.z  + 2*a3.z  + a4.z);
    q->w     += h/6.0 * (dq1.w + 2*dq2.w + 2*dq3.w + dq4.w);
    q->x     += h/6.0 * (dq1.x + 2*dq2.x + 2*dq3.x + dq4.x);
    q->y     += h/6.0 * (dq1.y + 2*dq2.y + 2*dq3.y + dq4.y);
    q->z     += h/6.0 * (dq1.z + 2*dq2.z + 2*dq3.z + dq4.z);
    *q = quat_normalize(*q);
}

/* ------------------------------------------------------------------ */
/*  Sweep uses fixed dt from CLI (not the adaptive controller dt)     */
/* ------------------------------------------------------------------ */

int main(int argc, char *argv[])
{
    if (argc < 3) {
        fprintf(stderr, "Usage: %s <dt_s> <omega_deg_per_axis> [--csv <path>]\n", argv[0]);
        return 1;
    }

    double dt = atof(argv[1]);
    double omega_deg = atof(argv[2]);

    /* Optional --csv <path> for full trajectory logging */
    const char *csv_path = NULL;
    for (int i = 3; i < argc - 1; i++) {
        if (strcmp(argv[i], "--csv") == 0) {
            csv_path = argv[i + 1];
            break;
        }
    }

    if (dt <= 0 || omega_deg <= 0) {
        fprintf(stderr, "ERROR: dt and omega must be positive\n");
        return 1;
    }

    srand(42);

    /* Initialize ADCS state */
    adcs_state_t state;
    adcs_mode_init(&state);

    /* Initial conditions */
    vec3d_t omega_true = {
        omega_deg * DEG_TO_RAD,
        omega_deg * DEG_TO_RAD * 0.7,
        omega_deg * DEG_TO_RAD * 0.5
    };
    quat_t q_true = quat_identity();

    sim_env_t env;
    sim_env_init(&env);
    sim_gyro_state_t gyro_sim;
    sim_gyro_init(&gyro_sim);

    adcs_mode_request(&state, ADCS_MODE_DETUMBLING);

    /* Open CSV if requested */
    FILE *csv = NULL;
    if (csv_path) {
        /* Create parent directory if needed */
        char dir_buf[512];
        strncpy(dir_buf, csv_path, sizeof(dir_buf) - 1);
        dir_buf[sizeof(dir_buf) - 1] = '\0';
        char *last_slash = strrchr(dir_buf, '/');
        if (last_slash) {
            *last_slash = '\0';
            mkdir(dir_buf, 0755);
        }

        csv = fopen(csv_path, "w");
        if (csv) {
            fprintf(csv, "step,time_s,dt,duty_cycle,k_gain,sat_ratio,"
                         "omega_x,omega_y,omega_z,omega_mag,"
                         "mtq_ix_ma,mtq_iy_ma,mtq_iz_ma,"
                         "dipole_x,dipole_y,dipole_z,"
                         "torque_x,torque_y,torque_z,"
                         "b_body_x,b_body_y,b_body_z\n");
        }
    }

    double sim_time = 0.0;
    int converged = 0;
    int step = 0;

    while (sim_time < SIM_TIME_LIMIT) {
        /* Fixed control period from CLI argument */

        sim_env_step(&env, dt);
        sim_mag_read(&env, q_true, &state.mag);
        sim_gyro_read(&gyro_sim, omega_true, dt, &state.gyro);

        state.b_field_eci = env.b_field_eci;
        state.dt = dt;

        int done = adcs_mode_step(&state);

        /* Duty cycle */
        double eta_duty = (dt > DETUMBLE_DEAD_TIME_S)
                        ? (dt - DETUMBLE_DEAD_TIME_S) / dt
                        : 0.0;

        /* RK4 physics */
        int n_sub = (dt >= PHYSICS_DT) ? (int)(dt / PHYSICS_DT) : 1;
        double sub_dt = dt / n_sub;
        for (int s = 0; s < n_sub; s++) {
            rk4_step(&omega_true, &q_true, env.b_field_eci,
                     &state.mtq_cmd, eta_duty, sub_dt);
        }

        sim_time += dt;

        /* CSV logging */
        if (csv) {
            vec3d_t b_body = quat_rotate_vec(q_true, env.b_field_eci);
            vec3d_t torque;
            sim_mtq_apply(&state.mtq_cmd, b_body, &torque);
            torque.x *= eta_duty; torque.y *= eta_duty; torque.z *= eta_duty;

            double k_gain = BDOT_GAIN_K;
            double dipole_mag = sqrt(state.mtq_cmd.dipole.x * state.mtq_cmd.dipole.x +
                                     state.mtq_cmd.dipole.y * state.mtq_cmd.dipole.y +
                                     state.mtq_cmd.dipole.z * state.mtq_cmd.dipole.z);
            double m_max_avg = (MTQ_MAX_DIPOLE_X + MTQ_MAX_DIPOLE_Y + MTQ_MAX_DIPOLE_Z) / 3.0;
            double sat_ratio = (m_max_avg > 0.0) ? dipole_mag / m_max_avg : 0.0;

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

        step++;

        if (done) {
            printf("CONVERGED %.1f\n", sim_time);
            converged = 1;
            break;
        }
    }

    if (csv) fclose(csv);

    if (!converged) {
        double final_omega = vec3d_norm(omega_true) * RAD_TO_DEG;
        printf("DIVERGED %.4f\n", final_omega);
    }

    return converged ? 0 : 1;
}
