/**
 * @file sim_mtq_driver.h
 * @brief Simulated DRV8214 magnetorquer drivers (voltage regulation, open loop).
 */

#ifndef SIM_MTQ_DRIVER_H
#define SIM_MTQ_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "adcs_types.h"

/**
 * @brief Apply magnetorquer command (simulation: compute torque).
 *
 * Each axis current is turned into the DRV8214 voltage code the flight
 * HAL would write (mtq_intensity_to_code()), and the coil current that
 * code drives through the true coil resistance R_nominal × (1 + error)
 * gives the dipole. With zero resistance error the dipole equals the
 * commanded one. The torque is τ = m × B.
 *
 * @param[in]  cmd     Magnetorquer command (dipole + intensity)
 * @param[in]  b_body  Magnetic field in body frame [T]
 * @param[out] torque  Resulting torque on satellite [N·m]
 */
void sim_mtq_apply(const mtq_command_t *cmd, vec3d_t b_body, vec3d_t *torque);

/**
 * @brief Set the relative coil resistance error per axis (default 0).
 *
 * Models the open-loop gain error of voltage drive, e.g. +0.25 for a coil
 * 25 % above nominal (≈ +64 °C over 20 °C for copper).
 *
 * @param[in] error  Relative error per axis (x, y, z)
 */
void sim_mtq_set_r_error(vec3d_t error);

#ifdef __cplusplus
}
#endif

#endif /* SIM_MTQ_DRIVER_H */
