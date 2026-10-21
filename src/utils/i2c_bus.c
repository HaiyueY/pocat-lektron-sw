/**
 * @file i2c_bus.c
 * @brief Shared I2C3 bus access with recovery (see i2c_bus.h).
 */

#include "i2c_bus.h"
#include "periph.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include <string.h>

/** @brief I2C3 pins on GPIOC. */
#define SCL_PIN     GPIO_PIN_0
#define SDA_PIN     GPIO_PIN_1

/** @brief Half period of the recovery clock [µs] (≈50 kHz). */
#define RECOVERY_HALF_PERIOD_US  10u

/** @brief Mutex serializing I2C3 transfers once the scheduler runs. */
static SemaphoreHandle_t bus_mutex = NULL;

/** @brief Per-address statistics. */
static i2c_bus_stats_t stats[I2C_BUS_MAX_DEVICES];

/** @brief Bus recoveries since boot. */
static uint32_t recoveries;

static int scheduler_running(void)
{
    return xTaskGetSchedulerState() == taskSCHEDULER_RUNNING;
}

static void lock(void)
{
    if (bus_mutex != NULL && scheduler_running()) {
        xSemaphoreTake(bus_mutex, portMAX_DELAY);
    }
}

static void unlock(void)
{
    if (bus_mutex != NULL && scheduler_running()) {
        xSemaphoreGive(bus_mutex);
    }
}

/** @brief Statistics slot of an address; the last slot collects overflow. */
static i2c_bus_stats_t *slot_for(uint8_t addr7)
{
    uint32_t i;

    for (i = 0; i < I2C_BUS_MAX_DEVICES; i++) {
        if (stats[i].addr7 == addr7) {
            return &stats[i];
        }
        if (stats[i].addr7 == 0) {
            stats[i].addr7 = addr7;
            return &stats[i];
        }
    }
    return &stats[I2C_BUS_MAX_DEVICES - 1];
}

/** @brief True for failures that may leave the bus stuck (not a plain NACK). */
static int is_bus_fault(HAL_StatusTypeDef status)
{
    uint32_t err = HAL_I2C_GetError(&hi2c3);

    if (status == HAL_BUSY || status == HAL_TIMEOUT) {
        return 1;
    }
    return (err & (HAL_I2C_ERROR_BERR | HAL_I2C_ERROR_ARLO | HAL_I2C_ERROR_TIMEOUT)) != 0;
}

/** @brief Release a stuck slave and reinitialize I2C3; call with the bus locked. */
static int recover_locked(void)
{
    GPIO_InitTypeDef gpio = {0};
    int i;
    int sda_free;

    recoveries++;
    HAL_I2C_DeInit(&hi2c3);

    /* Bit-bang SCL/SDA as open-drain GPIO, both released high */
    HAL_GPIO_WritePin(GPIOC, SCL_PIN | SDA_PIN, GPIO_PIN_SET);
    gpio.Pin = SCL_PIN | SDA_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &gpio);
    periph_delay_us(RECOVERY_HALF_PERIOD_US);

    /* Nine clocks let a slave finish the byte it is driving */
    for (i = 0; i < 9; i++) {
        HAL_GPIO_WritePin(GPIOC, SCL_PIN, GPIO_PIN_RESET);
        periph_delay_us(RECOVERY_HALF_PERIOD_US);
        HAL_GPIO_WritePin(GPIOC, SCL_PIN, GPIO_PIN_SET);
        periph_delay_us(RECOVERY_HALF_PERIOD_US);
    }

    /* STOP: SDA low → high while SCL is high */
    HAL_GPIO_WritePin(GPIOC, SCL_PIN, GPIO_PIN_RESET);
    periph_delay_us(RECOVERY_HALF_PERIOD_US);
    HAL_GPIO_WritePin(GPIOC, SDA_PIN, GPIO_PIN_RESET);
    periph_delay_us(RECOVERY_HALF_PERIOD_US);
    HAL_GPIO_WritePin(GPIOC, SCL_PIN, GPIO_PIN_SET);
    periph_delay_us(RECOVERY_HALF_PERIOD_US);
    HAL_GPIO_WritePin(GPIOC, SDA_PIN, GPIO_PIN_SET);
    periph_delay_us(RECOVERY_HALF_PERIOD_US);

    sda_free = HAL_GPIO_ReadPin(GPIOC, SDA_PIN) == GPIO_PIN_SET;

    /* HAL_I2C_Init → MspInit restores the AF configuration */
    if (HAL_I2C_Init(&hi2c3) != HAL_OK) {
        return -1;
    }
    return sda_free ? 0 : -1;
}

static int finish(uint8_t addr7, HAL_StatusTypeDef status)
{
    i2c_bus_stats_t *st = slot_for(addr7);

    st->transfers++;
    if (status == HAL_OK) {
        return 0;
    }
    st->errors++;
    if (is_bus_fault(status)) {
        (void)recover_locked();
    }
    return -1;
}

void i2c_bus_init(void)
{
    bus_mutex = xSemaphoreCreateMutex();
    configASSERT(bus_mutex);
}

int i2c_bus_read(uint8_t addr7, uint8_t reg, uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef status;
    int rc;

    lock();
    status = HAL_I2C_Mem_Read(&hi2c3, (uint16_t)(addr7 << 1), reg, I2C_MEMADD_SIZE_8BIT,
                              buf, len, I2C_BUS_TIMEOUT_MS);
    rc = finish(addr7, status);
    unlock();
    return rc;
}

int i2c_bus_write(uint8_t addr7, uint8_t reg, const uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef status;
    int rc;

    lock();
    status = HAL_I2C_Mem_Write(&hi2c3, (uint16_t)(addr7 << 1), reg, I2C_MEMADD_SIZE_8BIT,
                               (uint8_t *)buf, len, I2C_BUS_TIMEOUT_MS);
    rc = finish(addr7, status);
    unlock();
    return rc;
}

int i2c_bus_recover(void)
{
    int rc;

    lock();
    rc = recover_locked();
    unlock();
    return rc;
}

uint32_t i2c_bus_recovery_count(void)
{
    return recoveries;
}

void i2c_bus_get_stats(i2c_bus_stats_t out[I2C_BUS_MAX_DEVICES])
{
    lock();
    memcpy(out, stats, sizeof(stats));
    unlock();
}
