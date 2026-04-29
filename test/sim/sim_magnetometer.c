/**
 * @file sim_magnetometer.c
 * @brief Simulated MMC5983MA magnetometer.
 *
 * Generates body-frame magnetic field by rotating the ECI field
 * with the true attitude quaternion and adding Gaussian noise.
 *
 * Noise model: ±0.4 mG RMS ≈ 40 nT RMS per axis
 * Source: Sim_sat_sensors.m, MMC5983MA datasheet
 */

#include "sim_magnetometer.h"
#include "adcs_config.h"
#include "adcs_math.h"
#include <stdlib.h>
#include <math.h>

/** Box-Muller Gaussian noise generator */
static double gaussian_noise(double sigma)
{
    double u1 = ((double)rand() / RAND_MAX);
    double u2 = ((double)rand() / RAND_MAX);
    if (u1 < 1.0e-15) u1 = 1.0e-15;
    return sigma * sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

void sim_mag_read(const sim_env_t *env, quat_t q_true, mag_data_t *mag_out)
{
    /* Transform ECI field to body frame: B_body = q * B_eci * q* */
    vec3d_t b_body = quat_rotate_vec(q_true, env->b_field_eci);

    /* Add sensor noise */
    b_body.x += gaussian_noise(MAG_NOISE_RMS);
    b_body.y += gaussian_noise(MAG_NOISE_RMS);
    b_body.z += gaussian_noise(MAG_NOISE_RMS);

    mag_out->field = b_body;
    mag_out->valid = 1;
}
