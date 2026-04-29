/**
 * @file sim_gyroscope.c
 * @brief Simulated IIM-42652 gyroscope.
 *
 * Noise model:
 *   - Angle Random Walk: 0.038 °/s/√Hz → σ_arw = ARW * √(1/dt)
 *   - Bias instability: random walk on bias
 * Source: Sim_sat_sensors.m, IIM-42652 datasheet
 */

#include "sim_gyroscope.h"
#include "adcs_config.h"
#include "adcs_math.h"
#include <stdlib.h>
#include <math.h>

static double gaussian_noise(double sigma)
{
    double u1 = ((double)rand() / RAND_MAX);
    double u2 = ((double)rand() / RAND_MAX);
    if (u1 < 1.0e-15) u1 = 1.0e-15;
    return sigma * sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

void sim_gyro_init(sim_gyro_state_t *gs)
{
    gs->bias = vec3d_zero();
}

void sim_gyro_read(sim_gyro_state_t *gs, vec3d_t omega, double dt,
                   gyro_data_t *gyro_out)
{
    /* ARW noise: white noise with σ = ARW / √(dt) */
    double arw_sigma = GYRO_ARW / sqrt(dt);

    /* Bias random walk: drift per step using Rate Random Walk (RRW)
     * MATLAB: Sensors.gyros.rrw = 0.000133°/√(s³)
     * Δbias = RRW * √dt per step */
    double bias_walk_sigma = GYRO_RRW * sqrt(dt);
    gs->bias.x += gaussian_noise(bias_walk_sigma);
    gs->bias.y += gaussian_noise(bias_walk_sigma);
    gs->bias.z += gaussian_noise(bias_walk_sigma);

    /* Output = true ω + bias + ARW noise */
    gyro_out->angular_vel.x = omega.x + gs->bias.x + gaussian_noise(arw_sigma);
    gyro_out->angular_vel.y = omega.y + gs->bias.y + gaussian_noise(arw_sigma);
    gyro_out->angular_vel.z = omega.z + gs->bias.z + gaussian_noise(arw_sigma);
    gyro_out->valid = 1;
}
