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
/**
 * @brief diferent flag events for comunication
 * 
 */
#define OBC_EVENT_OBDH_DONE (1UL << 0) //Bit 0, obdh ha acabat
#define OBC_EVENT_PAYLOAD_Experiments (1UL<<1)//Bit 1, enviem dades experiment
#define OBC_EVENT_EPS_Measurements (1UL<<2) //Bit 2, enviem mesures EPS
#define OBC_PHOTO_CAPTURE (1UL << 3) // bit 4

/** @brief MSP post-initialization callback for TIM peripheral GPIO configuration. */
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/** @brief Error handler function  
* @todo Implement proper error handling mechanism.
*/
void Error_Handler(void);

#endif /* __MAIN_H */
