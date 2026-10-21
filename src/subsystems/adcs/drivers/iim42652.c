/**
 * @file iim42652.c
 * @brief IIM-42652 gyroscope driver (datasheet DS-000440 rev 1.0).
 */

#include "drivers/iim42652.h"
#include "drivers/adcs_bus.h"
#include <stddef.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Bank 0 registers */
#define REG_DEVICE_CONFIG       0x11u
#define REG_TEMP_DATA1          0x1Du   /* TEMP 0x1D-0x1E, ACCEL 0x1F-0x24, GYRO 0x25-0x2A */
#define REG_PWR_MGMT0           0x4Eu
#define REG_GYRO_CONFIG0        0x4Fu
#define REG_GYRO_ACCEL_CONFIG0  0x52u
#define REG_WHO_AM_I            0x75u
#define REG_BANK_SEL            0x76u

/* DEVICE_CONFIG */
#define SOFT_RESET_CONFIG       0x01u

/* GYRO_CONFIG0: GYRO_FS_SEL = 011b (±250 dps), GYRO_ODR = 1000b (100 Hz) */
#define GYRO_CONFIG0_VALUE      ((0x3u << 5) | 0x8u)

/* GYRO_ACCEL_CONFIG0: accel BW default (1), gyro BW = max(400 Hz, ODR)/40 = 10 Hz (7) */
#define GYRO_ACCEL_CONFIG0_VALUE ((0x1u << 4) | 0x7u)

/* PWR_MGMT0: temperature on, GYRO_MODE = 11b (low noise), accel off */
#define PWR_MGMT0_VALUE         (0x3u << 2)

/* Sensitivity at ±250 dps [LSB/(°/s)] */
#define GYRO_LSB_PER_DPS        131.0

/* Temperature: T = TEMP_DATA / 132.48 + 25 [°C] */
#define TEMP_LSB_PER_C          132.48

/* Sense registers read −32768 while the gyro has no valid sample */
#define SAMPLE_INVALID          (-32768)

/* Soft reset 1 ms; gyro minimum on-time 45 ms (covers the 30 ms start-up) */
#define T_SOFT_RESET_MS         1u
#define T_GYRO_ON_MS            45u

static int write_reg(uint8_t reg, uint8_t value)
{
    return adcs_bus_write(IIM42652_I2C_ADDR, reg, &value, 1);
}

static int16_t be16(const uint8_t *b)
{
    return (int16_t)(((uint16_t)b[0] << 8) | b[1]);
}

int iim42652_init(void)
{
    uint8_t id;

    if (write_reg(REG_BANK_SEL, 0x00u) != 0
        || write_reg(REG_DEVICE_CONFIG, SOFT_RESET_CONFIG) != 0) {
        return -1;
    }
    adcs_bus_delay_ms(T_SOFT_RESET_MS);

    if (adcs_bus_read(IIM42652_I2C_ADDR, REG_WHO_AM_I, &id, 1) != 0
        || id != IIM42652_WHO_AM_I) {
        return -1;
    }

    if (write_reg(REG_GYRO_CONFIG0, GYRO_CONFIG0_VALUE) != 0
        || write_reg(REG_GYRO_ACCEL_CONFIG0, GYRO_ACCEL_CONFIG0_VALUE) != 0
        || write_reg(REG_PWR_MGMT0, PWR_MGMT0_VALUE) != 0) {
        return -1;
    }
    adcs_bus_delay_ms(T_GYRO_ON_MS);
    return 0;
}

int iim42652_read_gyro(double rate_rad_s[3], double *temp_c)
{
    uint8_t b[14];
    int i;

    if (adcs_bus_read(IIM42652_I2C_ADDR, REG_TEMP_DATA1, b, sizeof(b)) != 0) {
        return -1;
    }

    for (i = 0; i < 3; i++) {
        int16_t raw = be16(&b[8 + 2 * i]);
        if (raw == SAMPLE_INVALID) {
            return -1;
        }
        rate_rad_s[i] = (double)raw / GYRO_LSB_PER_DPS * (M_PI / 180.0);
    }
    if (temp_c != NULL) {
        *temp_c = (double)be16(&b[0]) / TEMP_LSB_PER_C + 25.0;
    }
    return 0;
}
