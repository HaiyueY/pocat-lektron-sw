#pragma once

#include "FreeRTOS.h"
#include "event_groups.h"
#include <stdint.h>

typedef enum {
    HEALTH_BIT_PAYLOAD = (1u << 0),
    HEALTH_BIT_OBDH    = (1u << 1),
    HEALTH_BIT_EPS     = (1u << 2),
    HEALTH_BIT_COMMS   = (1u << 3),
    HEALTH_BIT_ADCS    = (1u << 4),
} health_bit_t;

void health_kick(EventBits_t bit);

