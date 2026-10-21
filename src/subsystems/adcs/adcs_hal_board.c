/**
 * @file adcs_hal_board.c
 * @brief ADCS HAL for the PQ_ADCS_HBridge board on the OBC (STM32L476).
 *
 * Implements the board I/O of the drivers (adcs_bus.h) on I2C3, ADC2 and
 * the ADCS control GPIOs, the task-facing HAL (adcs_hal.h), and the
 * magnetorquer output mtq_apply_command() (overriding the weak default in
 * adcs_magnetorquer.c).
 */

#include "adcs_hal.h"
#include "adcs_calib.h"
#include "adcs_magnetorquer.h"
#include "drivers/adcs_bus.h"
#include "drivers/css.h"
#include "drivers/drv8214.h"
#include "drivers/iim42652.h"
#include "drivers/mmc5983ma.h"
#include "i2c_bus.h"
#include "periph.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <string.h>

/** ADC reference: VDDA is the 3V3_PERM rail [V] */
#define CSS_VDDA_V          3.3

/** ADC full scale (12 bit) */
#define CSS_ADC_FULL_SCALE  4095.0

/** ADC sampling time for the TIA outputs (low source impedance) */
#define CSS_SAMPLING_TIME   ADC_SAMPLETIME_92CYCLES_5

/** DRV8214 wake-up to I2C ready (tWAKE = 410 µs) [µs] */
#define MTQ_WAKE_US         500u

/** Active calibration (defaults until adcs_hal_set_calib()) */
static adcs_calib_t calib;
static int calib_loaded;

/** Last sensor temperatures [°C] */
static double gyro_temp_c = NAN;
static double mag_temp_c = NAN;

/** Magnetorquer drivers awake and configured */
static int mtq_on;

/* -------------------------------------------------------------------------
 * Board I/O for the drivers (adcs_bus.h)
 * ---------------------------------------------------------------------- */

int adcs_bus_read(uint8_t addr7, uint8_t reg, uint8_t *buf, uint16_t len)
{
    return i2c_bus_read(addr7, reg, buf, len);
}

int adcs_bus_write(uint8_t addr7, uint8_t reg, const uint8_t *buf, uint16_t len)
{
    return i2c_bus_write(addr7, reg, buf, len);
}

void adcs_bus_delay_ms(uint32_t ms)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        vTaskDelay(pdMS_TO_TICKS(ms) > 0 ? pdMS_TO_TICKS(ms) : 1);
    } else {
        HAL_Delay(ms);
    }
}

void adcs_bus_delay_us(uint32_t us)
{
    periph_delay_us(us);
}

void adcs_bus_css_select(uint8_t channel)
{
    HAL_GPIO_WritePin(ADCS_SEL_PH0_PORT, ADCS_SEL_PH0_PIN,
                      (channel & 0x1u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ADCS_SEL_PH1_PORT, ADCS_SEL_PH1_PIN,
                      (channel & 0x2u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ADCS_SEL_PH2_PORT, ADCS_SEL_PH2_PIN,
                      (channel & 0x4u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

int adcs_bus_css_sample(double *volts)
{
    ADC_ChannelConfTypeDef cfg = {0};
    uint32_t raw;

    cfg.Channel = ADCS_CSS_ADC_CHANNEL;
    cfg.Rank = ADC_REGULAR_RANK_1;
    cfg.SamplingTime = CSS_SAMPLING_TIME;
    cfg.SingleDiff = ADC_SINGLE_ENDED;
    cfg.OffsetNumber = ADC_OFFSET_NONE;
    cfg.Offset = 0;
    if (HAL_ADC_ConfigChannel(&hadc2, &cfg) != HAL_OK) {
        return -1;
    }

    if (HAL_ADC_Start(&hadc2) != HAL_OK) {
        return -1;
    }
    if (HAL_ADC_PollForConversion(&hadc2, 10) != HAL_OK) {
        HAL_ADC_Stop(&hadc2);
        return -1;
    }
    raw = HAL_ADC_GetValue(&hadc2);
    HAL_ADC_Stop(&hadc2);

    *volts = (double)raw * (CSS_VDDA_V / CSS_ADC_FULL_SCALE);
    return 0;
}

void adcs_bus_mtq_power(int on)
{
    HAL_GPIO_WritePin(ADCS_MTQ_EN_PORT, ADCS_MTQ_EN_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* -------------------------------------------------------------------------
 * Task-facing HAL (adcs_hal.h)
 * ---------------------------------------------------------------------- */

static void ensure_calib(void)
{
    if (!calib_loaded) {
        adcs_calib_defaults(&calib);
        calib_loaded = 1;
    }
}

int adcs_hal_init(void)
{
    int rc = 0;

    ensure_calib();
    (void)adcs_hal_mtq_enable(false);

    if (mmc5983ma_init() != 0) {
        rc = -1;
    }
    if (iim42652_init() != 0) {
        rc = -1;
    }
    return rc;
}

int adcs_hal_set_calib(const adcs_calib_t *cal)
{
    if (adcs_calib_check(cal) != 0) {
        return -1;
    }
    memcpy(&calib, cal, sizeof(calib));
    calib_loaded = 1;
    return 0;
}

int adcs_hal_read_mag(mag_data_t *mag)
{
    double raw[3];
    double t;

    ensure_calib();
    mag->valid = 0;
    if (mmc5983ma_read_field(raw) != 0) {
        return -1;
    }
    mag->field = adcs_calib_mag(&calib, raw);
    mag->valid = 1;

    if (mmc5983ma_read_temp(&t) == 0) {
        mag_temp_c = t;
    }
    return 0;
}

int adcs_hal_read_gyro(gyro_data_t *gyro)
{
    double raw[3];
    double t;

    ensure_calib();
    gyro->valid = 0;
    if (iim42652_read_gyro(raw, &t) != 0) {
        return -1;
    }
    gyro_temp_c = t;
    gyro->angular_vel = adcs_calib_gyro(&calib, raw, t);
    gyro->valid = 1;
    return 0;
}

int adcs_hal_read_css(sun_sensor_data_t *sun)
{
    double raw[CSS_CHANNELS];

    ensure_calib();
    sun->valid = 0;
    if (css_read(raw) != 0) {
        return -1;
    }
    adcs_calib_css(&calib, raw, sun->voltage);
    sun->valid = 1;
    return 0;
}

void adcs_hal_get_temps(double *gyro_c, double *mag_c)
{
    if (gyro_c != NULL) {
        *gyro_c = gyro_temp_c;
    }
    if (mag_c != NULL) {
        *mag_c = mag_temp_c;
    }
}

int adcs_hal_mtq_enable(bool on)
{
    int axis;
    int rc = 0;

    if (!on) {
        mtq_on = 0;
        adcs_bus_mtq_power(0);
        return 0;
    }

    adcs_bus_mtq_power(1);
    adcs_bus_delay_us(MTQ_WAKE_US);
    for (axis = 0; axis < 3; axis++) {
        if (drv8214_present(axis) && drv8214_init(axis) != 0) {
            rc = -1;
        }
    }
    mtq_on = 1;
    return rc;
}

int adcs_hal_mtq_status(adcs_mtq_status_t status[3])
{
    drv8214_status_t st;
    int axis;
    int rc = 0;

    for (axis = 0; axis < 3; axis++) {
        memset(&status[axis], 0, sizeof(status[axis]));
        if (!drv8214_present(axis)) {
            continue;
        }
        if (drv8214_read_status(axis, &st) != 0) {
            rc = -1;
            continue;
        }
        status[axis].present = 1;
        status[axis].fault = st.fault;
        status[axis].voltage_v = st.voltage_v;
        status[axis].current_ma = st.current_ma;
    }
    return rc;
}

/* -------------------------------------------------------------------------
 * Magnetorquer output (overrides the weak default in adcs_magnetorquer.c)
 * ---------------------------------------------------------------------- */

int mtq_apply_command(const mtq_command_t *cmd)
{
    const double intensity[3] = {
        cmd->intensity_ma.x, cmd->intensity_ma.y, cmd->intensity_ma.z
    };
    int axis;
    int rc = 0;

    if (!mtq_on) {
        return -1;
    }
    for (axis = 0; axis < 3; axis++) {
        if (!drv8214_present(axis)) {
            continue;   /* X until its I2C address is fixed in hardware */
        }
        if (drv8214_set_code(axis, mtq_intensity_to_code(axis, intensity[axis])) != 0) {
            rc = -1;
        }
    }
    return rc;
}
