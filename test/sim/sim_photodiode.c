/**
 * @file sim_photodiode.c
 * @brief Simulated SLCD-61N8 photodiode sun sensor array.
 *
 * 6 photodiodes mounted on cube faces (+X, -X, +Y, -Y, +Z, -Z).
 * Each diode produces a voltage proportional to cos(θ) where θ
 * is the angle between the sun direction and the face normal.
 * Negative cosines (shadowed faces) read zero.
 *
 * Noise: 20 mV Gaussian (Source: Sim_sat_sensors.m)
 */

#include "sim_photodiode.h"
#include "adcs_config.h"
#include "adcs_math.h"
#include <stdlib.h>
#include <math.h>

/** Maximum photodiode voltage under full illumination [V] */
#define PD_VMAX  3.3

/** Face normals in body frame (±X, ±Y, ±Z) */
static const vec3d_t face_normals[6] = {
    { 1.0,  0.0,  0.0},  /* +X */
    {-1.0,  0.0,  0.0},  /* -X */
    { 0.0,  1.0,  0.0},  /* +Y */
    { 0.0, -1.0,  0.0},  /* -Y */
    { 0.0,  0.0,  1.0},  /* +Z */
    { 0.0,  0.0, -1.0}   /* -Z */
};

static double gaussian_noise(double sigma)
{
    double u1 = ((double)rand() / RAND_MAX);
    double u2 = ((double)rand() / RAND_MAX);
    if (u1 < 1.0e-15) u1 = 1.0e-15;
    return sigma * sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

void sim_photodiode_read(const sim_env_t *env, quat_t q_true,
                         sun_sensor_data_t *sun_out)
{
    if (env->eclipse) {
        /* In eclipse: no sun signal */
        for (int i = 0; i < 6; i++) {
            sun_out->voltage[i] = fabs(gaussian_noise(PHOTODIODE_NOISE_STD));
        }
        sun_out->sun_body = vec3d_zero();
        sun_out->sun_visible = 0;
        sun_out->valid = 1;
        return;
    }

    /* Transform sun ECI to body frame */
    vec3d_t sun_body = quat_rotate_vec(q_true, env->sun_eci);
    sun_body = vec3d_normalize(sun_body);

    /* Compute photodiode voltages: V = Vmax * max(0, cos(θ)) + noise */
    for (int i = 0; i < 6; i++) {
        double cos_theta = vec3d_dot(sun_body, face_normals[i]);
        double v = (cos_theta > 0.0) ? PD_VMAX * cos_theta : 0.0;
        v += gaussian_noise(PHOTODIODE_NOISE_STD);
        if (v < 0.0) v = 0.0;
        sun_out->voltage[i] = v;
    }

    /* Estimate sun direction from photodiode voltages
     * Weighted sum of face normals by their voltages */
    vec3d_t sun_est = vec3d_zero();
    for (int i = 0; i < 6; i++) {
        sun_est.x += sun_out->voltage[i] * face_normals[i].x;
        sun_est.y += sun_out->voltage[i] * face_normals[i].y;
        sun_est.z += sun_out->voltage[i] * face_normals[i].z;
    }
    sun_out->sun_body = vec3d_normalize(sun_est);
    sun_out->sun_visible = 1;
    sun_out->valid = 1;
}
