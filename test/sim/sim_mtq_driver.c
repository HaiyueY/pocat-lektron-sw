/**
 * @file sim_mtq_driver.c
 * @brief Simulated DRV8214 magnetorquer drivers.
 *
 * Reproduces the flight output path: quantized current → signed voltage
 * code → coil current through the true resistance → dipole → torque
 * τ = m × B, used by the dynamics integrator.
 */

#include "sim_mtq_driver.h"
#include "adcs_config.h"
#include "adcs_magnetorquer.h"
#include "adcs_math.h"

static const double coil_r_ohm[3] = {
    MTQ_COIL_R_OHM_X, MTQ_COIL_R_OHM_Y, MTQ_COIL_R_OHM_Z
};

static const double coil_factor[3] = {
    MTQ_COIL_FACTOR_X, MTQ_COIL_FACTOR_Y, MTQ_COIL_FACTOR_Z
};

static double r_error[3] = {0.0, 0.0, 0.0};

void sim_mtq_set_r_error(vec3d_t error)
{
    r_error[0] = error.x;
    r_error[1] = error.y;
    r_error[2] = error.z;
}

void sim_mtq_apply(const mtq_command_t *cmd, vec3d_t b_body, vec3d_t *torque)
{
    const double intensity[3] = {
        cmd->intensity_ma.x, cmd->intensity_ma.y, cmd->intensity_ma.z
    };
    double dipole[3];
    int i;

    for (i = 0; i < 3; i++) {
        int code = mtq_intensity_to_code(i, intensity[i]);
        double current_a = code * MTQ_VSET_LSB_V / (coil_r_ohm[i] * (1.0 + r_error[i]));
        dipole[i] = current_a * coil_factor[i];
    }

    *torque = vec3d_cross(vec3d_make(dipole[0], dipole[1], dipole[2]), b_body);
}
