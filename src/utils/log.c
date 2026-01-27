/**
 * @file log.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2026-01-20
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "log.h"
#include <string.h>
#include "FreeRTOS.h"
#include "semphr.h"

// IMPORTANT: ONLY FOR DEBUGGING PURPOSES.
// TASK ENTERS CRITICAL SECTION TO PRINT WITHOUT INTERRUPTION.
int _write(int file, char *ptr, int len)
{
    taskENTER_CRITICAL();
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, 1000);
    taskEXIT_CRITICAL();
    return len;
}