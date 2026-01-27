/**
 * @file log.c
 * @brief Debug logging support. 
 * @details It currently simply provides a redirection of standard output to a serial interface.
 * @author Guillermo O'Tuama Pascual
 * @date 2026-01-22
 * 
 */

#include "log.h"
#include <string.h>
#include "FreeRTOS.h"
#include "semphr.h"

int _write(int file, char *ptr, int len)
{
    taskENTER_CRITICAL();
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, 1000);
    taskEXIT_CRITICAL();
    return len;
}