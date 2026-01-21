#include "stm32l4xx_hal.h"
#include "stm32l476xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include "periph.h"

void log_init(void);
int _write(int file, char *ptr, int len);
