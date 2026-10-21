/**
 * @file drv8214.h
 * @brief TI DRV8214 H-bridge driver for the magnetorquer coils (I2C only).
 *
 * PQ_ADCS_HBridge has one DRV8214 per axis. EN/IN1 and PH/IN2 are tied
 * to GND and nFAULT is not connected, so the bridge is driven entirely
 * through I2C (I2C_BC = 1) and faults are polled. nSLEEP of all three is
 * PoL_ADCS_POWER_EN; the registers reset while asleep, so drv8214_init()
 * must run after every wake.
 *
 * Drive mode: voltage regulation (REG_CTRL = 11b, VM_GAIN_SEL = 1b), so a
 * code n sets n × 15.37 mV across the coil (see MTQ_VSET_LSB_V). Current
 * regulation and stall detection are off; VREF is tied to 3V3 and would
 * otherwise clip the coil current through the 22 kΩ RIPROPI.
 */

#ifndef DRV8214_H
#define DRV8214_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** Axis indices (same order as mtq_command_t) */
#define DRV8214_AXIS_X  0
#define DRV8214_AXIS_Y  1
#define DRV8214_AXIS_Z  2

/**
 * 7-bit I2C addresses: 0x30 + A3..A0 from the A1/A0 tri-level straps
 * (datasheet Table 8-28). DRV8214_ADDR_NONE marks an axis the driver
 * must not address.
 *
 *  X (U5): A1 = GND, A0 = GND → 0x30, the same address as the MMC5983MA.
 *          TBD by hardware (e.g. A0 → VCC gives 0x32); disabled until then.
 *  Y (U8): A1 = open, A0 = GND → 0x33 (the schematic note says 0001b, 0x31).
 *  Z (U9): A1 = open, A0 = open → 0x34.
 */
#define DRV8214_ADDR_NONE   0x00u
#define DRV8214_ADDR_X      DRV8214_ADDR_NONE   /* TBD: strap collides with MMC5983MA */
#define DRV8214_ADDR_Y      0x33u
#define DRV8214_ADDR_Z      0x34u

/** FAULT register bits */
#define DRV8214_FAULT_ANY   0x80u
#define DRV8214_FAULT_STALL 0x20u
#define DRV8214_FAULT_OCP   0x10u
#define DRV8214_FAULT_OVP   0x08u
#define DRV8214_FAULT_TSD   0x04u
#define DRV8214_FAULT_NPOR  0x02u   /**< Set after power-up / wake (register reset) */

/** Status readback of one driver */
typedef struct {
    uint8_t fault;          /**< FAULT register */
    double  voltage_v;      /**< VMTR: coil terminal voltage [V] */
    double  current_ma;     /**< IMTR: coil current [mA] */
} drv8214_status_t;

/** @brief 1 if the axis has a usable I2C address. */
int drv8214_present(int axis);

/**
 * @brief Program the driver after wake: voltage regulation, outputs coasting.
 * @return 0 on success, -1 on bus error or if the axis is not present
 */
int drv8214_init(int axis);

/**
 * @brief Drive the coil with a signed voltage code.
 *
 * |code| is written to WSET_VSET; the sign selects the bridge direction
 * (positive = forward, current OUT1 → OUT2). 0 coasts the bridge (Hi-Z).
 *
 * @param code  Signed code, limited to ±255
 * @return 0 on success, -1 on bus error or if the axis is not present
 */
int drv8214_set_code(int axis, int code);

/** @brief Coast (outputs Hi-Z). */
int drv8214_coast(int axis);

/** @brief Brake (both low-side FETs on). */
int drv8214_brake(int axis);

/** @brief Read FAULT, VMTR and IMTR. */
int drv8214_read_status(int axis, drv8214_status_t *status);

/** @brief Clear latched faults (CLR_FLT). */
int drv8214_clear_faults(int axis);

#ifdef __cplusplus
}
#endif

#endif /* DRV8214_H */
