/**
 * @file sim_mtq_driver.h
 * @brief Simulated BD2606MVV magnetorquer driver.
 */

#ifndef SIM_MTQ_DRIVER_H
#define SIM_MTQ_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "adcs_types.h"

/**
 * @brief Apply magnetorquer command (simulation: log and compute torque).
 *
 * In simulation, this function logs the commanded dipole and computes
 * the environmental torque that results from the magnetic dipole
 * in the ambient B-field: τ = m × B.
 *
 * @param[in]  cmd     Magnetorquer command (dipole + intensity)
 * @param[in]  b_body  Magnetic field in body frame [T]
 * @param[out] torque  Resulting torque on satellite [N·m]
 */
void sim_mtq_apply(const mtq_command_t *cmd, vec3d_t b_body, vec3d_t *torque);

#ifdef __cplusplus
}
#endif

#endif /* SIM_MTQ_DRIVER_H */
