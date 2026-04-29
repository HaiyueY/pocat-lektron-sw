/**
 * @file test_monte_carlo.c
 * @brief Monte Carlo robustness test for detumbling controller.
 *
 * Runs N simulations with randomized initial conditions:
 *   - Angular velocity: fixed magnitude, random direction (uniform sphere)
 *   - Attitude: uniform random quaternion
 *   - Sensor noise: different RNG seed per trial
 *
 * Usage: ./test_monte_carlo <omega_total_deg> <n_runs> [--seed <N>] [--csv-dir <dir>]
 *
 * Output: per-trial one-line result + summary statistics.
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
/*  Random sampling utilities                                         */
/* ------------------------------------------------------------------ */

/** Standard normal via Box–Muller transform */
static double randn(void)
{
    double u1 = ((double)rand() + 1.0) / ((double)RAND_MAX + 2.0);
    double u2 = ((double)rand() + 1.0) / ((double)RAND_MAX + 2.0);
    return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

/** Uniform random direction on unit sphere (via 3 Gaussians, normalized) */
static vec3d_t random_unit_vec(void)
{
    vec3d_t v;
    double norm;
    do {
        v.x = randn();
        v.y = randn();
        v.z = randn();
        norm = vec3d_norm(v);
    } while (norm < 1e-10);
    v.x /= norm; v.y /= norm; v.z /= norm;
    return v;
}

/** Uniform random quaternion (Shoemake method) */
static quat_t random_quaternion(void)
{
    double u0 = (double)rand() / (double)RAND_MAX;
    double u1 = (double)rand() / (double)RAND_MAX;
    double u2 = (double)rand() / (double)RAND_MAX;

    double s1 = sqrt(1.0 - u0);
    double s0 = sqrt(u0);

    quat_t q;
    q.w = s1 * sin(2.0 * M_PI * u1);
    q.x = s1 * cos(2.0 * M_PI * u1);
    q.y = s0 * sin(2.0 * M_PI * u2);
    q.z = s0 * cos(2.0 * M_PI * u2);
    return quat_normalize(q);
}

/* ------------------------------------------------------------------ */
/*  Single simulation run                                              */
/* ------------------------------------------------------------------ */

typedef struct {
    int converged;
    double conv_time;
    double final_omega_deg;
} run_result_t;

static run_result_t run_single(vec3d_t omega_init, quat_t q_init,
                               FILE *csv)
{
    run_result_t result = {0, 0.0, 0.0};

    adcs_state_t state;
    adcs_mode_init(&state);
    adcs_mode_request(&state, ADCS_MODE_DETUMBLING);

    vec3d_t omega_true = omega_init;
    quat_t q_true = q_init;

    sim_env_t env;
    sim_env_init(&env);
    sim_gyro_state_t gyro_sim;
    sim_gyro_init(&gyro_sim);

    if (csv) {
        fprintf(csv, "step,time_s,dt,omega_x,omega_y,omega_z,omega_mag\n");
    }

    double sim_time = 0.0;
    int step = 0;

    while (sim_time < SIM_TIME_LIMIT) {
        double dt = detumble_select_dt(&state);

        sim_env_step(&env, dt);
        sim_mag_read(&env, q_true, &state.mag);
        sim_gyro_read(&gyro_sim, omega_true, dt, &state.gyro);

        state.b_field_eci = env.b_field_eci;
        state.dt = dt;

        int done = adcs_mode_step(&state);

        double eta_duty = (dt > DETUMBLE_DEAD_TIME_S)
                        ? (dt - DETUMBLE_DEAD_TIME_S) / dt
                        : 0.0;

        int n_sub = (dt >= PHYSICS_DT) ? (int)(dt / PHYSICS_DT) : 1;
        double sub_dt = dt / n_sub;
        for (int s = 0; s < n_sub; s++) {
            rk4_step(&omega_true, &q_true, env.b_field_eci,
                     &state.mtq_cmd, eta_duty, sub_dt);
        }

        sim_time += dt;
        step++;

        if (csv) {
            fprintf(csv, "%d,%.3f,%.3f,%.10e,%.10e,%.10e,%.10e\n",
                    step, sim_time, dt,
                    omega_true.x, omega_true.y, omega_true.z,
                    vec3d_norm(omega_true));
        }

        if (done) {
            result.converged = 1;
            result.conv_time = sim_time;
            result.final_omega_deg = vec3d_norm(omega_true) * RAD_TO_DEG;
            break;
        }
    }

    if (!result.converged) {
        result.final_omega_deg = vec3d_norm(omega_true) * RAD_TO_DEG;
    }

    return result;
}

/* ------------------------------------------------------------------ */
/*  Main                                                              */
/* ------------------------------------------------------------------ */

int main(int argc, char *argv[])
{
    if (argc < 3) {
        fprintf(stderr,
            "Usage: %s <omega_total_deg> <n_runs> [--seed <N>] [--csv-dir <dir>]\n",
            argv[0]);
        return 1;
    }

    double omega_total_deg = atof(argv[1]);
    int n_runs = atoi(argv[2]);
    unsigned int base_seed = 42;
    const char *csv_dir = NULL;

    for (int i = 3; i < argc - 1; i++) {
        if (strcmp(argv[i], "--seed") == 0) {
            base_seed = (unsigned int)atoi(argv[i + 1]);
        } else if (strcmp(argv[i], "--csv-dir") == 0) {
            csv_dir = argv[i + 1];
        }
    }

    if (omega_total_deg <= 0 || n_runs <= 0) {
        fprintf(stderr, "ERROR: omega and n_runs must be positive\n");
        return 1;
    }

    double omega_total_rad = omega_total_deg * DEG_TO_RAD;

    if (csv_dir) {
        mkdir(csv_dir, 0755);
    }

    printf("=== Monte Carlo Detumble Test ===\n");
    printf("|ω₀| = %.1f deg/s, N = %d, base_seed = %u\n",
           omega_total_deg, n_runs, base_seed);
    printf("Adaptive ΔT: π/(2|ω|), clamped [%.1f, %.1f] s\n",
           DETUMBLE_DT_MIN, DETUMBLE_DT_MAX);
    printf("ω×B gain k = %.1f (ω_sat = %.1f deg/s)\n\n",
           BDOT_GAIN_K, BDOT_SAT_OMEGA * RAD_TO_DEG);

    printf("%5s  %8s  %10s  %10s  %8s  %8s  %8s  %8s  %8s  %8s  %8s\n",
           "Run", "Result", "Time(s)", "ω_final", "ωx₀", "ωy₀", "ωz₀",
           "qw₀", "qx₀", "qy₀", "qz₀");
    printf("-----  --------  ----------  ----------  --------  --------  "
           "--------  --------  --------  --------  --------\n");

    int n_pass = 0;
    double t_min = 1e9, t_max = 0.0, t_sum = 0.0, t_sq_sum = 0.0;

    /* Summary CSV */
    FILE *summary_csv = NULL;
    if (csv_dir) {
        char path[512];
        snprintf(path, sizeof(path), "%s/monte_carlo_summary.csv", csv_dir);
        summary_csv = fopen(path, "w");
        if (summary_csv) {
            fprintf(summary_csv,
                "run,seed,converged,conv_time_s,final_omega_deg,"
                "wx0,wy0,wz0,qw0,qx0,qy0,qz0\n");
        }
    }

    for (int r = 0; r < n_runs; r++) {
        unsigned int seed = base_seed + (unsigned int)r;
        srand(seed);

        /* Random initial angular velocity direction */
        vec3d_t dir = random_unit_vec();
        vec3d_t omega_init = {dir.x * omega_total_rad,
                              dir.y * omega_total_rad,
                              dir.z * omega_total_rad};

        /* Random initial quaternion */
        quat_t q_init = random_quaternion();

        /* Per-trial CSV */
        FILE *trial_csv = NULL;
        if (csv_dir) {
            char path[512];
            snprintf(path, sizeof(path), "%s/trial_%03d.csv", csv_dir, r);
            trial_csv = fopen(path, "w");
        }

        run_result_t res = run_single(omega_init, q_init, trial_csv);

        if (trial_csv) fclose(trial_csv);

        const char *status = res.converged ? "PASS" : "FAIL";
        printf("%5d  %8s  %10.1f  %10.4f  %8.2f  %8.2f  %8.2f  %8.4f  %8.4f  %8.4f  %8.4f\n",
               r, status, res.conv_time, res.final_omega_deg,
               omega_init.x * RAD_TO_DEG, omega_init.y * RAD_TO_DEG,
               omega_init.z * RAD_TO_DEG,
               q_init.w, q_init.x, q_init.y, q_init.z);

        if (summary_csv) {
            fprintf(summary_csv, "%d,%u,%d,%.3f,%.6f,%.10e,%.10e,%.10e,%.6f,%.6f,%.6f,%.6f\n",
                    r, seed, res.converged, res.conv_time, res.final_omega_deg,
                    omega_init.x, omega_init.y, omega_init.z,
                    q_init.w, q_init.x, q_init.y, q_init.z);
        }

        if (res.converged) {
            n_pass++;
            if (res.conv_time < t_min) t_min = res.conv_time;
            if (res.conv_time > t_max) t_max = res.conv_time;
            t_sum += res.conv_time;
            t_sq_sum += res.conv_time * res.conv_time;
        }
    }

    if (summary_csv) fclose(summary_csv);

    /* Summary statistics */
    printf("\n=== Summary ===\n");
    printf("Pass rate: %d / %d (%.1f%%)\n", n_pass, n_runs,
           100.0 * n_pass / n_runs);

    if (n_pass > 0) {
        double t_mean = t_sum / n_pass;
        double t_std = (n_pass > 1)
            ? sqrt((t_sq_sum - n_pass * t_mean * t_mean) / (n_pass - 1))
            : 0.0;
        printf("Convergence time: min=%.1f  max=%.1f  mean=%.1f  std=%.1f s\n",
               t_min, t_max, t_mean, t_std);
        printf("Orbits: min=%.2f  max=%.2f  mean=%.2f\n",
               t_min / 5540.0, t_max / 5540.0, t_mean / 5540.0);
    }

    int fail_count = n_runs - n_pass;
    if (fail_count > 0) {
        printf("FAILURES: %d trials did not converge within 12 hours\n", fail_count);
    }

    /* Auto-generate plots if csv_dir was provided */
    if (csv_dir) {
        printf("\nGenerating plots...\n");
        char cmd[1024];
        snprintf(cmd, sizeof(cmd),
                 "python3 ../plot_monte_carlo.py %s %s", csv_dir, csv_dir);
        int plot_rc = system(cmd);
        if (plot_rc != 0) {
            printf("WARNING: Plot generation failed (python3 or script error)\n");
        }
    }

    /* Exit code: 0 if all pass, 1 if any fail */
    return (n_pass == n_runs) ? 0 : 1;
}
