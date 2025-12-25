/**
 * *****************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 */

#ifndef __MAIN_H
#define __MAIN_H

#include "stm32l4xx_hal.h"
#include "stm32l476xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "log.h"
#include "periph.h"
   

// OBC event group bits
#define OBC_PHOTO_CAPTURE (1 << 0) // bit 0

// PAYLOAD event group bits
#define PAYLOAD_PHOTO_CAPTURE (1 << 0) // bit 0

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

extern IWDG_HandleTypeDef hiwdg; // Watchdog handle, esto lo he puesto aqui ahora pero mirar de hacerlo mas modular...

void Error_Handler(void); // s'ha d'implementar

#endif /* __MAIN_H */
