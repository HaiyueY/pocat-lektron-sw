/**
 * @file test_nadir.c
 * @brief Nadir pointing mode integration test.
 *
 * Scenario: Satellite starts with small angular velocity (post-detumble)
 *           and arbitrary initial attitude.
 * Expected: Nadir pointing controller aligns the +Z body axis with
 *           the nadir direction to within ~20° pointing error.
 *
 * Uses full closed-loop simulation with TRIAD attitude determination,
 * magnetic control law, and simulated sensors/actuators.
 *
 * Usage: test_nadir [--ephem [--igrf-order N] [--start-hours H]] [--seed S]
 *
 *   (default)  legacy environment; the controller is given the true ECI
 *              reference vectors (S1 regression case).
 *   --ephem    truth environment from sim_env_init_ephem() (design TLE,
 *              IGRF-14 degree 13, moving Sun, 2027-09-15). The controller
 *              gets its reference vectors from adcs_ephem_update() with
 *              degree N (default ADCS_IGRF_ORDER), exactly as on board,
 *              so IGRF truncation errors enter the loop.
 *              --start-hours shifts the start (and hence the ground
 *              track) from the TLE epoch.
 *   --seed     sensor-noise seed (default 42).
 *
 * The last output line is a machine-readable summary for sweep scripts:
 *   SUMMARY pass=<0|1> ape=<fraction> mean_last_orbit=<deg> min=<deg> peak=<deg>
 * where peak is the largest error after the first acquisition.
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
#include "adcs_ephemeris.h"
#include "sim_environment.h"
#include "sim_magnetometer.h"
#include "sim_gyroscope.h"
#include "sim_photodiode.h"
#include "sim_mtq_driver.h"

/** Total simulated time [s] — 5 orbital periods (MATLAB-mimic test) */
#define SIM_TIME_LIMIT  27700.0

/** Adaptive control rate parameters.
 *
 * Diagnostic study (2026-04-30) showed that PD analytical output is
 * below the MTQ 0.5 mA quantization step for any reasonable err < 30°,
 * so sensor noise drives the actuator across the deadband and the
 * resulting quantization-bias pumps energy into the body.  Slowing
 * the control loop averages per-sample noise and pushes more cycles
 * below the dead zone — but a fixed slow rate cannot react during
 * recovery from large excursions.
 *
 * Adaptive scheme: switch between a fast rate (1 s, used during
 * acquire/recovery) and a slow rate (10 s, used in steady-state hold)
 * based on instantaneous pointing error.  Hysteresis bands prevent
 * chattering at the boundary. */
#define DT_FAST            1.0    /* control period during acquire/recovery [s] */
#define DT_HOLD            1.0    /* control period during quiet hold [s] */
/* Effectively-disabled hysteresis: stay in fast (= dt=1s) mode permanently
 * for now while we're testing Σ-Δ dither.  Σ-Δ's noise-shaping benefit
 * scales with sample rate, so dt=1s is the correct test bench.  After
 * Σ-Δ is validated we can revisit slow-rate hold mode if needed. */
#define ERR_ENTER_HOLD   180.0
#define ERR_EXIT_HOLD    181.0

/** Mission APE requirement: error < 20° for ≥ 95% of the time */
#define APE_LIMIT_DEG     20.0
#define APE_MIN_FRACTION  0.95

/** Physics sub-step for numerical stability */
#define PHYSICS_DT  0.01

#define LOG_INTERVAL 100

/** CSV output path */
#define CSV_DIR     "results"
#ifndef FREE_FLIGHT
#define FREE_FLIGHT 0
#endif
#if FREE_FLIGHT
#define CSV_FILE    CSV_DIR "/nadir_freeflight.csv"
#else
#define CSV_FILE    CSV_DIR "/nadir.csv"
#endif

/** IGRF degree of the truth field in --ephem mode */
#define TRUTH_IGRF_ORDER  13

/**
 * @brief Compute pointing error angle between body Z-axis and nadir.
 * @return Pointing error [deg]
 */
static double compute_pointing_error(quat_t q_true, vec3d_t nadir_eci)
{
    vec3d_t body_z = {NADIR_BODY_AXIS_X, NADIR_BODY_AXIS_Y, NADIR_BODY_AXIS_Z};

    /* Transform body Z-axis to ECI frame using inverse (conjugate) rotation */
    quat_t q_inv = quat_conjugate(q_true);
    vec3d_t body_z_eci = quat_rotate_vec(q_inv, body_z);

    /* Angle between body Z in ECI and nadir direction */
    vec3d_t nadir_norm = vec3d_normalize(nadir_eci);
    double cos_angle = vec3d_dot(body_z_eci, nadir_norm);

    /* Clamp to [-1, 1] for numerical safety */
    if (cos_angle > 1.0) cos_angle = 1.0;
    if (cos_angle < -1.0) cos_angle = -1.0;

    return acos(cos_angle) * RAD_TO_DEG;
}

static void dynamics_step(vec3d_t *omega, vec3d_t torque, double dt)
{
    vec3d_t iw;
    iw.x = SAT_INERTIA_XX * omega->x;
    iw.y = SAT_INERTIA_YY * omega->y;
    iw.z = SAT_INERTIA_ZZ * omega->z;

    vec3d_t gyro_torque = vec3d_cross(*omega, iw);

    vec3d_t alpha;
    alpha.x = (torque.x - gyro_torque.x) / SAT_INERTIA_XX;
    alpha.y = (torque.y - gyro_torque.y) / SAT_INERTIA_YY;
    alpha.z = (torque.z - gyro_torque.z) / SAT_INERTIA_ZZ;

    omega->x += alpha.x * dt;
    omega->y += alpha.y * dt;
    omega->z += alpha.z * dt;
}

static void attitude_step(quat_t *q, vec3d_t omega, double dt)
{
    /* For q_{ECI→Body}: dq/dt = -0.5 * [0,ω] ⊗ q  (left multiplication) */
    quat_t omega_q = {0.0, -omega.x, -omega.y, -omega.z};
    quat_t dq = quat_multiply(omega_q, *q);
    q->w += 0.5 * dq.w * dt;
    q->x += 0.5 * dq.x * dt;
    q->y += 0.5 * dq.y * dt;
    q->z += 0.5 * dq.z * dt;
    *q = quat_normalize(*q);
}

int main(int argc, char **argv)
{
    int ephem = 0;
    int igrf_order = ADCS_IGRF_ORDER;
    unsigned seed = 42;
    double start_hours = 0.0;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--ephem") == 0) {
            ephem = 1;
        } else if (strcmp(argv[i], "--igrf-order") == 0 && i + 1 < argc) {
            igrf_order = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--seed") == 0 && i + 1 < argc) {
            seed = (unsigned)strtoul(argv[++i], NULL, 10);
        } else if (strcmp(argv[i], "--start-hours") == 0 && i + 1 < argc) {
            start_hours = atof(argv[++i]);
        } else {
            fprintf(stderr, "usage: %s [--ephem [--igrf-order N] [--start-hours H]] [--seed S]\n",
                    argv[0]);
            return 2;
        }
    }
    if (igrf_order < 1 || igrf_order > 13) {
        fprintf(stderr, "--igrf-order must be 1..13\n");
        return 2;
    }

    char csv_file[128];
    if (ephem) {
        snprintf(csv_file, sizeof(csv_file), CSV_DIR "/nadir_ephem_n%d.csv", igrf_order);
    } else if (seed != 42) {
        snprintf(csv_file, sizeof(csv_file), CSV_DIR "/nadir_seed%u.csv", seed);
    } else {
        snprintf(csv_file, sizeof(csv_file), "%s", CSV_FILE);
    }

    printf("=== ADCS Nadir Pointing Test ===\n");
    printf("Target: +Z body axis → nadir direction\n");
    printf("Success criterion: pointing error < 20 deg (steady-state)\n");
    if (ephem) {
        printf("Environment: truth = design TLE + IGRF-14 n=%d + moving Sun (2027-09-15)\n",
               TRUTH_IGRF_ORDER);
        printf("Controller references: adcs_ephem_update(), IGRF n=%d\n", igrf_order);
        printf("Start: TLE epoch + %.2f h, noise seed %u\n\n", start_hours, seed);
    } else {
        printf("Environment: legacy (circular 400 km polar, tilted dipole, fixed Sun)\n\n");
    }

    srand(seed);

    adcs_state_t state = {0};
    adcs_mode_init(&state);

    /* Initial conditions: PERFECT NADIR ATTITUDE for hold-test diagnosis.
     * Body axes aligned with LVLH (body_z → nadir, body_x → velocity).
     * Body ω = orbit rate so attitude is stationary in LVLH frame. */
    sim_env_t env;
    adcs_orbit_t onboard_orbit;
    if (ephem) {
        sim_env_ephem_cfg_t cfg = {SIM_DESIGN_TLE1, SIM_DESIGN_TLE2,
                                   0.0, TRUTH_IGRF_ORDER};
        adcs_orbit_init(&onboard_orbit);
        if (adcs_orbit_load_tle(&onboard_orbit, SIM_DESIGN_TLE1, SIM_DESIGN_TLE2) != ADCS_ORBIT_OK) {
            printf("FAIL: on-board TLE rejected\n");
            return 1;
        }
        cfg.start_unix = onboard_orbit.tle.epoch_unix + start_hours * 3600.0;
        if (sim_env_init_ephem(&env, &cfg) != 0) {
            printf("FAIL: truth environment could not be initialised\n");
            return 1;
        }
    } else {
        sim_env_init(&env);
    }
    sim_gyro_state_t gyro_sim;
    sim_gyro_init(&gyro_sim);

    /* Initial attitude: MATLAB-mimic — a "random-looking" fixed quaternion
     * from MATLAB Sim_sat_initial_state.m commented options.
     * (MATLAB uses randn(4,1)/norm; we pick a reproducible one to compare.) */
    quat_t q_true = {0.282051597168532, 0.683091541439498,
                     0.613312153382015, 0.278713194991566};
    q_true = quat_normalize(q_true);
    (void)quat_from_lvlh; /* unused in this IC */

    /* Orbit rate ≈ 2π/5540 ≈ 1.134e-3 rad/s about -y_body (anti orbit-normal).
     * MATLAB-mimic ω_0 = (n, -n, n): magnitude √3·n, deliberately
     * off-axis from orbit-normal so the controller must do real damping
     * work (cf. Sim_sat_initial_state.m line 27). */
    const double n_orbit = 2.0 * M_PI / 5540.0;
    vec3d_t omega_true = {n_orbit, -n_orbit, n_orbit};

    /* Request nadir pointing mode */
    int rc = adcs_mode_request(&state, ADCS_MODE_NADIR_POINTING);
    if (rc != 0) {
        printf("FAIL: Could not enter nadir pointing mode\n");
        return 1;
    }
    printf("Mode: NADIR_POINTING\n\n");

    /* Open CSV for data logging */
    mkdir(CSV_DIR, 0755);
    FILE *csv = fopen(csv_file, "w");
    if (!csv) {
        printf("WARNING: Could not open %s for writing\n", csv_file);
    } else {
        fprintf(csv, "step,time_s,err_deg,omega_x,omega_y,omega_z,omega_mag,"
                      "dipole_x,dipole_y,dipole_z,"
                      "torque_x,torque_y,torque_z,"
                      "quat_w,quat_x,quat_y,quat_z,"
                      "pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,eclipse\n");
    }

    printf("%6s  %10s  %10s  %10s  %7s\n",
           "Step", "Err [deg]", "|ω| deg/s", "|m| A·m²", "Eclipse");
    printf("------  ----------  ----------  ----------  -------\n");

    double min_error = 180.0;
    int steady_count = 0;
    int first_convergence = 0;
    int nadir_acquisitions = 0;  /* times error crosses below threshold */
    int was_above = 1;           /* track threshold crossings */
    double peak_after_acq = 0.0;   /* largest error after first acquisition [deg] */
    double ape_time_total = 0.0;   /* time since first acquisition [s] */
    double ape_time_within = 0.0;  /* ... of which error < APE_LIMIT_DEG [s] */
    double last_orbit_sum = 0.0;
    int last_orbit_count = 0;
    const double last_orbit_start_time = SIM_TIME_LIMIT - 5540.0;

    double sim_time = 0.0;
    double dt = DT_HOLD;          /* initial: assume we start in hold */
    int hold_mode = 1;            /* hysteresis state */
    int step = 0;
    int log_counter = 0;
    long fast_steps = 0, hold_steps = 0;

    while (sim_time < SIM_TIME_LIMIT) {
        /* Choose control period for this step based on current pointing
         * error (computed before sim_env_step so it reflects the state
         * the controller will actually see). */
        double err_now = compute_pointing_error(q_true, env.nadir_eci);
        if (hold_mode) {
            if (err_now > ERR_EXIT_HOLD) hold_mode = 0;
        } else {
            if (err_now < ERR_ENTER_HOLD) hold_mode = 1;
        }
        dt = hold_mode ? DT_HOLD : DT_FAST;
        if (hold_mode) hold_steps++; else fast_steps++;

        sim_env_step(&env, dt);

        /* Simulate all sensors */
        sim_mag_read(&env, q_true, &state.mag);
        sim_gyro_read(&gyro_sim, omega_true, dt, &state.gyro);
        sim_photodiode_read(&env, q_true, &state.sun);

        /* Provide ECI references: on-board models in --ephem mode,
         * otherwise the truth itself (legacy regression case) */
        if (ephem) {
            adcs_ephem_t eph;
            adcs_ephem_update(&onboard_orbit, env.unix_s, igrf_order, &eph);
            if (adcs_ephem_to_state(&eph, &state) != 0) {
                printf("FAIL: on-board ephemeris invalid (status 0x%02x) at t=%.0f s\n",
                       eph.status, sim_time);
                if (csv) fclose(csv);
                return 1;
            }
        } else {
            state.b_field_eci = env.b_field_eci;
            state.sun_eci = env.sun_eci;
            state.nadir_eci = env.nadir_eci;
            state.pos_eci = env.pos_eci;
            state.vel_eci = env.vel_eci;
        }
        state.dt = dt;

        /* Run controller */
        adcs_mode_step(&state);

#if FREE_FLIGHT
        state.mtq_cmd.dipole = vec3d_zero();
#endif

        /* Apply torque with sub-stepping */
        vec3d_t b_body = quat_rotate_vec(q_true, env.b_field_eci);
        vec3d_t torque;
        sim_mtq_apply(&state.mtq_cmd, b_body, &torque);

        int n_sub = (int)(dt / PHYSICS_DT);
        for (int si = 0; si < n_sub; si++) {
            b_body = quat_rotate_vec(q_true, env.b_field_eci);
            sim_mtq_apply(&state.mtq_cmd, b_body, &torque);
            dynamics_step(&omega_true, torque, PHYSICS_DT);
            attitude_step(&q_true, omega_true, PHYSICS_DT);
        }

        sim_time += dt;

        /* Compute pointing error after propagation */
        double err = compute_pointing_error(q_true, env.nadir_eci);
        double omega_mag = vec3d_norm(omega_true) * RAD_TO_DEG;
        double dipole_mag = vec3d_norm(state.mtq_cmd.dipole);

        if (err < min_error) min_error = err;

        if (err < APE_LIMIT_DEG) {
            steady_count++;
            if (was_above) {
                nadir_acquisitions++;
                was_above = 0;
            }
        } else {
            steady_count = 0;
            was_above = 1;
        }

        if (nadir_acquisitions > 0) {
            if (err > peak_after_acq) peak_after_acq = err;
            ape_time_total += dt;
            if (err < APE_LIMIT_DEG) ape_time_within += dt;
        }

        if (sim_time >= last_orbit_start_time) {
            last_orbit_sum += err;
            last_orbit_count++;
        }

        if (csv) {
            fprintf(csv, "%d,%.1f,%.6f,%.10e,%.10e,%.10e,%.10e,"
                         "%.10e,%.10e,%.10e,"
                         "%.10e,%.10e,%.10e,"
                         "%.10f,%.10f,%.10f,%.10f,"
                         "%.6e,%.6e,%.6e,%.6e,%.6e,%.6e,%d\n",
                    step, sim_time,
                    err,
                    omega_true.x, omega_true.y, omega_true.z,
                    vec3d_norm(omega_true),
                    state.mtq_cmd.dipole.x, state.mtq_cmd.dipole.y, state.mtq_cmd.dipole.z,
                    torque.x, torque.y, torque.z,
                    q_true.w, q_true.x, q_true.y, q_true.z,
                    env.pos_eci.x, env.pos_eci.y, env.pos_eci.z,
                    env.vel_eci.x, env.vel_eci.y, env.vel_eci.z,
                    env.eclipse ? 1 : 0);
        }

        if (log_counter++ % LOG_INTERVAL == 0) {
            printf("%6d  %10.2f  %10.4f  %10.6f  %7s\n",
                   step, err, omega_mag, dipole_mag,
                   env.eclipse ? "YES" : "NO");
        }

        if (steady_count >= 100 && !first_convergence) {
            printf("\n=== NADIR POINTING ACHIEVED at t=%.0fs ===\n", sim_time);
            printf("Pointing error: %.2f deg\n", err);
            first_convergence = 1;
        }
        step++;
    }

    if (csv) fclose(csv);

    double mean_last_orbit = last_orbit_count > 0
                             ? last_orbit_sum / last_orbit_count : 180.0;
    double ape_fraction = ape_time_total > 0.0
                          ? ape_time_within / ape_time_total : 0.0;

    printf("\n--- Results ---\n");
    printf("Minimum pointing error:  %.2f deg\n", min_error);
    printf("Nadir acquisitions:      %d  (error crossed below 20°)\n",
           nadir_acquisitions);
    printf("Within 20° after first acquisition: %.1f%% of %.0f s\n",
           100.0 * ape_fraction, ape_time_total);
    printf("Mean error (last orbit): %.2f deg\n", mean_last_orbit);
    printf("Adaptive rate: hold steps=%ld (%.1f%%), fast steps=%ld (%.1f%%)\n",
           hold_steps, 100.0*hold_steps/(hold_steps+fast_steps),
           fast_steps, 100.0*fast_steps/(hold_steps+fast_steps));

    /*
     * Pass criteria for magnetic-only B-cross control:
     *   1. Controller can find nadir: min_error < 10°
     *   2. Mission APE:              error < 20° for ≥ 95% of the time
     *                                after the first acquisition
     *   3. Stable pointing:          mean_last_orbit < 30°
     *
     * Orbit-period oscillation (~5-15°) is expected for magnetic-only
     * B-cross control (B-field rotates, controllability is intermittent).
     * Eclipse degrades TRIAD attitude estimation, causing temporary
     * error peaks. Criterion 2 used to be "≥ 3 acquisitions", which
     * penalised a controller that acquires once and never leaves the
     * 20° band; it now checks the APE requirement directly.
     */
    int ape_ok = ape_fraction >= APE_MIN_FRACTION;
    int pass = (min_error < 10.0)
            && ape_ok
            && (mean_last_orbit < 30.0);

    if (pass) {
        printf("\nPASS: Nadir pointing test passed\n");
        printf("  ✓ min error %.2f° < 10°\n", min_error);
        printf("  ✓ APE %.1f%% within 20° (≥ 95%%)\n", 100.0 * ape_fraction);
        printf("  ✓ last-orbit mean %.2f° < 30°\n", mean_last_orbit);
    } else {
        printf("\nFAIL: Nadir pointing test failed\n");
        if (min_error >= 10.0)
            printf("  ✗ min error %.2f° ≥ 10°\n", min_error);
        else
            printf("  ✓ min error %.2f° < 10°\n", min_error);
        if (!ape_ok)
            printf("  ✗ APE %.1f%% within 20° (< 95%%)\n", 100.0 * ape_fraction);
        else
            printf("  ✓ APE %.1f%% within 20° (≥ 95%%)\n", 100.0 * ape_fraction);
        if (mean_last_orbit >= 30.0)
            printf("  ✗ last-orbit mean %.2f° ≥ 30°\n", mean_last_orbit);
        else
            printf("  ✓ last-orbit mean %.2f° < 30°\n", mean_last_orbit);
    }

    /* Auto-generate plots (skipped when ADCS_TEST_NO_PLOT is set, e.g. ctest) */
    if (getenv("ADCS_TEST_NO_PLOT") == NULL) {
        printf("\nGenerating plots...\n");
        char cmd[256];
        snprintf(cmd, sizeof(cmd), "python3 ../plot_results.py nadir %s " CSV_DIR "/", csv_file);
        int plot_rc = system(cmd);
        if (plot_rc != 0) {
            printf("WARNING: Plot generation failed (python3 not found or script error)\n");
        } else {
            printf("Plots saved to %s/\n", CSV_DIR);
        }
    }

    printf("SUMMARY pass=%d ape=%.4f mean_last_orbit=%.3f min=%.3f peak=%.3f\n",
           pass, ape_fraction, mean_last_orbit, min_error, peak_after_acq);

    if (pass) return 0;
    else return 1;
}
