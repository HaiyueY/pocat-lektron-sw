/**
 * @file periph.h
 * @brief Global peripheral handle declarations.
 * @details 
 * This file declares global STM32 HAL peripheral handles. These handles are defined
 * in periph.c and initialized during system startup in main.c.
 * @author Guillermo O'Tuama Pascual
 * @date 2026-01-20
 * 
 */

#include "stm32l4xx_hal.h"

/** @brief Global SPI handle. Used for communication with the SX1262. 
 *  @todo Might have to modify it, picked spi1 for radiolib, but it might have to be another.
*/
extern SPI_HandleTypeDef hspi1;  

/** @brief Global TIM2 handle. Temporarily declared for tone generation with Radiolib.
 *  @todo Might not be needed for SX1262 operation, remove if not used.
 */
extern TIM_HandleTypeDef htim2;

/** @brief Global TIM5 handle. Used as a microsecond timebase. */
extern TIM_HandleTypeDef htim5;

/** @brief Global UART2 handle. Used for debug logging. */
extern UART_HandleTypeDef huart2;

/** @brief Global Independent Watchdog handle. */
extern IWDG_HandleTypeDef hiwdg;