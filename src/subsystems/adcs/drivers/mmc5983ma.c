/**
 * @file mmc5983ma.c
 * @brief MEMSIC MMC5983MA driver (datasheet Rev A, 2019-04-03).
 */

#include "drivers/mmc5983ma.h"
#include "drivers/adcs_bus.h"

/* Registers */
#define REG_XOUT0       0x00u   /* Xout0..Zout1, XYZout2: 0x00-0x06 */
#define REG_TOUT        0x07u
#define REG_STATUS      0x08u
#define REG_CTRL0       0x09u
#define REG_CTRL1       0x0Au
#define REG_CTRL2       0x0Bu
#define REG_PRODUCT_ID  0x2Fu

/* Status bits */
#define STATUS_MEAS_M_DONE  0x01u
#define STATUS_MEAS_T_DONE  0x02u

/* Internal Control 0 bits */
#define CTRL0_TM_M      0x01u
#define CTRL0_TM_T      0x02u
#define CTRL0_SET       0x08u
#define CTRL0_RESET     0x10u

/* Internal Control 1 bits */
#define CTRL1_SW_RST    0x80u

/* 18-bit output sensitivity (null code 131072 cancels in SET − RESET) */
#define COUNTS_PER_T    (16384.0 * 1.0e4)   /* 16384 counts/G, 1 G = 1e-4 T */

/* Power-on time after SW_RST [ms] */
#define T_RESET_MS      10u

/* Settling after a SET/RESET pulse (500 ns pulse; margin for the cap recharge) [ms] */
#define T_SR_MS         1u

/* Poll budget per measurement: 8 ms at BW = 0, so 20 ms is ample [ms] */
#define T_MEAS_MAX_MS   20u

static int write_reg(uint8_t reg, uint8_t value)
{
    return adcs_bus_write(MMC5983MA_I2C_ADDR, reg, &value, 1);
}

static int wait_status(uint8_t mask)
{
    uint32_t t;
    uint8_t status;

    for (t = 0; t <= T_MEAS_MAX_MS; t++) {
        if (adcs_bus_read(MMC5983MA_I2C_ADDR, REG_STATUS, &status, 1) != 0) {
            return -1;
        }
        if (status & mask) {
            return 0;
        }
        adcs_bus_delay_ms(1);
    }
    return -1;
}

/** One magnetic measurement as raw 18-bit codes. */
static int measure_raw(uint32_t raw[3])
{
    uint8_t b[7];

    if (write_reg(REG_CTRL0, CTRL0_TM_M) != 0 || wait_status(STATUS_MEAS_M_DONE) != 0) {
        return -1;
    }
    if (adcs_bus_read(MMC5983MA_I2C_ADDR, REG_XOUT0, b, sizeof(b)) != 0) {
        return -1;
    }
    raw[0] = ((uint32_t)b[0] << 10) | ((uint32_t)b[1] << 2) | ((b[6] >> 6) & 0x3u);
    raw[1] = ((uint32_t)b[2] << 10) | ((uint32_t)b[3] << 2) | ((b[6] >> 4) & 0x3u);
    raw[2] = ((uint32_t)b[4] << 10) | ((uint32_t)b[5] << 2) | ((b[6] >> 2) & 0x3u);
    return 0;
}

int mmc5983ma_init(void)
{
    uint8_t id;

    if (write_reg(REG_CTRL1, CTRL1_SW_RST) != 0) {
        return -1;
    }
    adcs_bus_delay_ms(T_RESET_MS);

    if (adcs_bus_read(MMC5983MA_I2C_ADDR, REG_PRODUCT_ID, &id, 1) != 0
        || id != MMC5983MA_PRODUCT_ID) {
        return -1;
    }

    /* Single-shot mode: no continuous measurement, no periodic set */
    if (write_reg(REG_CTRL1, (uint8_t)(MMC5983MA_BW & 0x3u)) != 0
        || write_reg(REG_CTRL2, 0x00u) != 0) {
        return -1;
    }
    return 0;
}

int mmc5983ma_read_field(double field_t[3])
{
    uint32_t after_set[3], after_reset[3];
    int i;

    if (write_reg(REG_CTRL0, CTRL0_SET) != 0) {
        return -1;
    }
    adcs_bus_delay_ms(T_SR_MS);
    if (measure_raw(after_set) != 0) {
        return -1;
    }

    if (write_reg(REG_CTRL0, CTRL0_RESET) != 0) {
        return -1;
    }
    adcs_bus_delay_ms(T_SR_MS);
    if (measure_raw(after_reset) != 0) {
        return -1;
    }

    /* SET: +H + offset, RESET: −H + offset → H = (SET − RESET) / 2 */
    for (i = 0; i < 3; i++) {
        field_t[i] = ((double)after_set[i] - (double)after_reset[i]) / 2.0 / COUNTS_PER_T;
    }
    return 0;
}

int mmc5983ma_read_temp(double *temp_c)
{
    uint8_t t;

    if (write_reg(REG_CTRL0, CTRL0_TM_T) != 0 || wait_status(STATUS_MEAS_T_DONE) != 0) {
        return -1;
    }
    if (adcs_bus_read(MMC5983MA_I2C_ADDR, REG_TOUT, &t, 1) != 0) {
        return -1;
    }
    /* 0 = −75 °C, ≈0.8 °C/LSB over −75…125 °C */
    *temp_c = -75.0 + 0.8 * (double)t;
    return 0;
}
