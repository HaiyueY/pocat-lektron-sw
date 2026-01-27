#include "stm32l4xx_hal.h"

extern SPI_HandleTypeDef hspi1; // might have to change it, picked spi1 for radiolib, but it might have to be another
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5; // timer for micros()
extern UART_HandleTypeDef huart2;
extern IWDG_HandleTypeDef hiwdg;