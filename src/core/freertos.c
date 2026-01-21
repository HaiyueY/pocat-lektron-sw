/**
 * @file freertos.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2026-01-20
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "FreeRTOS.h"
#include "task.h"

// Can be implemented in the future for run time stats
void configureTimerForRunTimeStats(void) {}

unsigned long getRunTimeCounterValue(void) { return 0; }

// This function is called by FreeRTOS to get the memory for the idle task.
// It is used to allocate static memory for the idle task's TCB and stack.
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleTaskStackBuffer,
                                    uint32_t *pulIdleTaskStackSize )
{
  static StaticTask_t xIdleTaskTCB;
  static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
  *ppxIdleTaskStackBuffer = uxIdleTaskStack;
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

