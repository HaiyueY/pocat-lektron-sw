/**
 * @file periph.h
 * @brief Peripheral handle declarations and initialization helpers.
 * @details
 * Exposes the global STM32 HAL peripheral handles used across the firmware and
 * the public entry points for peripheral initialization and clock-dependent
 * reconfiguration.
 * @author Guillermo O'Tuama Pascual
 * @date 2026-01-20
 */

#ifndef INC_UTILS_PERIPH_H_
#define INC_UTILS_PERIPH_H_

#include "clock.h"
#include "stm32l4xx_hal.h"

/** @brief Global TIM2 handle. Temporarily declared for tone generation with Radiolib.
 *  @todo Might not be needed for SX1262 operation, remove if not used.
 */
extern TIM_HandleTypeDef htim2;

/** @brief Global TIM5 handle used as the microsecond timebase. */
extern TIM_HandleTypeDef htim5;

/** @brief Global SPI2 handle used for SX1262 communication. */
extern SPI_HandleTypeDef hspi2;

/** @brief Global UART4 handle used for debug logging. */
extern UART_HandleTypeDef huart4;

/** @brief Global independent watchdog handle. */
extern IWDG_HandleTypeDef hiwdg;

/** @brief Global RTC handle. */
extern RTC_HandleTypeDef hrtc;

/** @brief Global ADC1 handle. Used for internal temperature sensor. */
extern ADC_HandleTypeDef hadc1;

/** @brief Global I2C3 handle: ADCS bus (PC0 = SCL, PC1 = SDA). Use through i2c_bus.h. */
extern I2C_HandleTypeDef hi2c3;

/** @brief Global ADC2 handle. Used for the ADCS sun-sensor mux output (PC5). */
extern ADC_HandleTypeDef hadc2;

/* ADCS board control pins (PQ_ADCS_HBridge through the stack connector) */
#define ADCS_MTQ_EN_PORT    GPIOA        /**< PoL_ADCS_POWER_EN → DRV8214 nSLEEP (high = awake) */
#define ADCS_MTQ_EN_PIN     GPIO_PIN_7
#define ADCS_SEL_PH0_PORT   GPIOC        /**< TMUX1108 A0 */
#define ADCS_SEL_PH0_PIN    GPIO_PIN_2
#define ADCS_SEL_PH1_PORT   GPIOA        /**< TMUX1108 A1 */
#define ADCS_SEL_PH1_PIN    GPIO_PIN_11
#define ADCS_SEL_PH2_PORT   GPIOA        /**< TMUX1108 A2 */
#define ADCS_SEL_PH2_PIN    GPIO_PIN_12
#define ADCS_CSS_ADC_CHANNEL ADC_CHANNEL_14 /**< ADC_PH on PC5 (ADC12_IN14) */

/**
 * @brief Initialize all board peripherals for a system clock frequency.
 * @param freq System clock selection.
 */
void periph_init_for_freq(ClockFreq_t freq);

/**
 * @brief Reconfigure peripherals that depend on the system clock frequency.
 * @param freq New system clock selection.
 */
void periph_reconfigure_for_freq(ClockFreq_t freq);

/**
 * @brief Busy-wait on the TIM5 microsecond timebase.
 * @param us Microseconds to wait.
 */
void periph_delay_us(uint32_t us);

#endif /* INC_UTILS_PERIPH_H_ */
