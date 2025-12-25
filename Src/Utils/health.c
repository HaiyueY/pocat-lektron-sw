#include "health.h"

static EventGroupHandle_t health_eg = NULL;

static EventBits_t  expected_bits = 0;

static TickType_t period_ticks = 0;

static TickType_t next_deadline = 0;

static inline BaseType_t time_reached(TickType_t now, TickType_t target)
{
    return ((int32_t)(now - target) >= 0);
}

static inline void lock(void)   { taskENTER_CRITICAL(); }
static inline void unlock(void) { taskEXIT_CRITICAL();  }

static void start_new_period(TickType_t now)
{
    lock();
    EventBits_t expected = expected_bits;
    TickType_t period    = period_ticks;
    unlock();

    if (expected != 0 && health_eg != NULL)
    {
        xEventGroupClearBits(health_eg, expected);
    }

    next_deadline = now + period; // siguiente periodo
}


// public functions:

void health_init(void)
{
    if (health_eg == NULL)
    {
        health_eg = xEventGroupCreate();
    }
}

void health_kick(EventBits_t bit)
{
    if (health_eg != NULL)
    {
        xEventGroupSetBits(health_eg, bit);
    }
}

void health_config(TickType_t period)
{
    // revisar estos checks, quanto ponemos como minimo ... 1 tiene poco sentido...
    if (period == 0) period = 1;

    lock();
    period_ticks = period;
    unlock();

    start_new_period(xTaskGetTickCount());
}

void health_set_expected(EventBits_t exp_bits)
{
    lock();
    expected_bits = exp_bits;
    unlock();

    /* Restart window when mode expectations change */
    start_new_period(xTaskGetTickCount());
}


EventBits_t system_health(void)
{

    TickType_t now = xTaskGetTickCount();

    if (time_reached(now, next_deadline))
    {
        lock();
        EventBits_t expected = expected_bits;
        unlock();

        EventBits_t got = xEventGroupGetBits(health_eg);

        EventBits_t missing = expected & ~got;

        start_new_period(now);

        return missing;  
    }

    return 0;
}