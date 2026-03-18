/**
 * @file main.h
 * @brief Application entry point definitions.
 * @author Guillermo O'Tuama Pascual
 * @date 2026-01-20
 */
 

#ifndef __MAIN_H
#define __MAIN_H

#include "stm32l4xx_hal.h"
#include "stm32l476xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "log.h"
#include "periph.h"


/** @brief MSP post-initialization callback for TIM peripheral GPIO configuration. */
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/** @brief Error handler function  
* @todo Implement proper error handling mechanism.
*/
void Error_Handler(void);

#endif /* __MAIN_H */
