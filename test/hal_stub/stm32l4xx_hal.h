/**
 * @file stm32l4xx_hal.h
 * @brief Minimal HAL type stubs for host-PC compilation.
 *
 * Provides only the type definitions and macros needed by ADCS code
 * to compile on a host PC without the real STM32 HAL.
 */

#ifndef STM32L4XX_HAL_H
#define STM32L4XX_HAL_H

#include <stdint.h>
#include <stddef.h>

/* HAL status codes */
typedef enum {
    HAL_OK      = 0x00U,
    HAL_ERROR   = 0x01U,
    HAL_BUSY    = 0x02U,
    HAL_TIMEOUT = 0x03U
} HAL_StatusTypeDef;

/* Minimal I2C handle stub */
typedef struct {
    uint32_t dummy;
} I2C_HandleTypeDef;

/* Tick function stub */
static inline uint32_t HAL_GetTick(void) { return 0; }

#endif /* STM32L4XX_HAL_H */
