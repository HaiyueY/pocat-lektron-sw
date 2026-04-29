/**
 * @file sim_mtq_driver.c
 * @brief Simulated BD2606MVV magnetorquer driver.
 *
 * Computes the magnetic torque τ = m × B from the commanded dipole
 * and the ambient magnetic field. This torque is used by the
 * dynamics integrator to update the satellite angular velocity.
 */

#include "sim_mtq_driver.h"
#include "adcs_math.h"

void sim_mtq_apply(const mtq_command_t *cmd, vec3d_t b_body, vec3d_t *torque)
{
    /* Torque = dipole × B_body */
    *torque = vec3d_cross(cmd->dipole, b_body);
}
