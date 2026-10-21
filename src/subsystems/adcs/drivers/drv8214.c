/**
 * @file drv8214.c
 * @brief DRV8214 magnetorquer driver (datasheet SLVSH04, November 2023).
 */

#include "drivers/drv8214.h"
#include "drivers/adcs_bus.h"

/* Registers */
#define REG_FAULT       0x00u
#define REG_STATUS1     0x04u   /* VMTR */
#define REG_STATUS2     0x05u   /* IMTR */
#define REG_CONFIG0     0x09u
#define REG_CONFIG3     0x0Cu
#define REG_CONFIG4     0x0Du
#define REG_CTRL0       0x0Eu
#define REG_CTRL1       0x0Fu   /* WSET_VSET */
#define REG_CTRL2       0x10u
#define REG_RC_CTRL0    0x11u

/* CONFIG0 */
#define CONFIG0_EN_OUT      0x80u
#define CONFIG0_EN_OVP      0x40u
#define CONFIG0_VM_GAIN_SEL 0x08u   /* 0–3.92 V voltage range */
#define CONFIG0_CLR_FLT     0x02u
/* EN_STALL = 0, VSNS_SEL = 0 (analog output filter), DUTY_CTRL = 0 */
#define CONFIG0_VALUE       (CONFIG0_EN_OUT | CONFIG0_EN_OVP | CONFIG0_VM_GAIN_SEL)

/* CONFIG3: IMODE = 00b (no current regulation), SMODE = 1, INT_VREF = 0,
 * TBLANK/TDEG default, OCP_MODE = 1 and TSD_MODE = 1 (auto-retry) */
#define CONFIG3_VALUE       0x23u

/* CONFIG4: no nFAULT reporting, PMODE = 1 (PWM: allows coast),
 * I2C_BC = 1 (bridge from I2C_EN_IN1 / I2C_PH_IN2) */
#define CONFIG4_BASE        0x0Cu
#define CONFIG4_IN1         0x02u
#define CONFIG4_IN2         0x01u

/* REG_CTRL0: EN_SS = 0 (setpoints apply at once), REG_CTRL = 11b
 * (voltage regulation), PWM_FREQ = 0 (50 kHz), W_SCALE unused */
#define CTRL0_VALUE         0x18u

/* REG_CTRL2: OUT_FLT = 11b (1 kHz output filter, ≥ 20× below 50 kHz PWM) */
#define CTRL2_VALUE         0xC0u

/* RC_CTRL0: ripple counting off, FLT_GAIN_SEL default, CS_GAIN_SEL = 011b
 * (AIPROPI 1125 µA/A, 0.5 A full scale: IPROPI stays below VM − 1.25 V
 * up to ≈ 83 mA with RIPROPI = 22 kΩ) */
#define RC_CTRL0_VALUE      0x0Bu

/* Readback scaling: VMTR B0h = 11 V; IMTR C0h = CS_GAIN_SEL full scale */
#define VMTR_V_PER_LSB      (11.0 / 176.0)
#define IMTR_MA_PER_LSB     (500.0 / 192.0)

#define CODE_MAX            255

static const uint8_t addr_of[3] = {
    DRV8214_ADDR_X,
    DRV8214_ADDR_Y,
    DRV8214_ADDR_Z
};

static int write_reg(int axis, uint8_t reg, uint8_t value)
{
    return adcs_bus_write(addr_of[axis], reg, &value, 1);
}

int drv8214_present(int axis)
{
    return axis >= 0 && axis < 3 && addr_of[axis] != DRV8214_ADDR_NONE;
}

int drv8214_init(int axis)
{
    if (!drv8214_present(axis)) {
        return -1;
    }
    /* Bridge first (coast), then the regulation scheme, then outputs on */
    if (write_reg(axis, REG_CONFIG4, CONFIG4_BASE) != 0
        || write_reg(axis, REG_CONFIG3, CONFIG3_VALUE) != 0
        || write_reg(axis, REG_RC_CTRL0, RC_CTRL0_VALUE) != 0
        || write_reg(axis, REG_CTRL0, CTRL0_VALUE) != 0
        || write_reg(axis, REG_CTRL1, 0x00u) != 0
        || write_reg(axis, REG_CTRL2, CTRL2_VALUE) != 0
        || write_reg(axis, REG_CONFIG0, CONFIG0_VALUE | CONFIG0_CLR_FLT) != 0) {
        return -1;
    }
    return 0;
}

int drv8214_set_code(int axis, int code)
{
    uint8_t magnitude;
    uint8_t bridge;

    if (!drv8214_present(axis)) {
        return -1;
    }
    if (code == 0) {
        return drv8214_coast(axis);
    }
    if (code > CODE_MAX) code = CODE_MAX;
    if (code < -CODE_MAX) code = -CODE_MAX;

    magnitude = (uint8_t)(code > 0 ? code : -code);
    /* PWM mode truth table: IN1 = 1, IN2 = 0 forward; IN1 = 0, IN2 = 1 reverse */
    bridge = (uint8_t)(CONFIG4_BASE | (code > 0 ? CONFIG4_IN1 : CONFIG4_IN2));

    if (write_reg(axis, REG_CTRL1, magnitude) != 0
        || write_reg(axis, REG_CONFIG4, bridge) != 0) {
        return -1;
    }
    return 0;
}

int drv8214_coast(int axis)
{
    if (!drv8214_present(axis)) {
        return -1;
    }
    return write_reg(axis, REG_CONFIG4, CONFIG4_BASE);
}

int drv8214_brake(int axis)
{
    if (!drv8214_present(axis)) {
        return -1;
    }
    return write_reg(axis, REG_CONFIG4, CONFIG4_BASE | CONFIG4_IN1 | CONFIG4_IN2);
}

int drv8214_read_status(int axis, drv8214_status_t *status)
{
    uint8_t fault;
    uint8_t vi[2];

    if (!drv8214_present(axis)) {
        return -1;
    }
    if (adcs_bus_read(addr_of[axis], REG_FAULT, &fault, 1) != 0
        || adcs_bus_read(addr_of[axis], REG_STATUS1, vi, sizeof(vi)) != 0) {
        return -1;
    }
    status->fault = fault;
    status->voltage_v = vi[0] * VMTR_V_PER_LSB;
    status->current_ma = vi[1] * IMTR_MA_PER_LSB;
    return 0;
}

int drv8214_clear_faults(int axis)
{
    if (!drv8214_present(axis)) {
        return -1;
    }
    return write_reg(axis, REG_CONFIG0, CONFIG0_VALUE | CONFIG0_CLR_FLT);
}
