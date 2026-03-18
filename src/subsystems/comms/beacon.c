/* ---- Includes ---- */

#include "FreeRTOS.h"
#include "timers.h"
#include "beacon.h"
#include "notifications.h"
#include "obc.h"


/* ---- Module-level variables ---- */

static TimerHandle_t beacon_timer;


/* ---- Private function declarations ---- */

static void beacon_timer_callback(TimerHandle_t xTimer);


/* ---- Public function definitions ---- */

void beacon_init(void)
{
    beacon_timer = xTimerCreate(
        "Beacon",
        pdMS_TO_TICKS(BEACON_PERIOD_MS),
        pdTRUE,   /* auto-reload */
        NULL,
        beacon_timer_callback
    );

    xTimerStart(beacon_timer, 0);
}


/* ---- Private function definitions ---- */

static void beacon_timer_callback(TimerHandle_t xTimer)
{
    (void)xTimer;

    TaskHandle_t comms = obc_get_comms_handle();
    if (comms != NULL) {
        xTaskNotify(comms, N_COMMS_TRANSMIT_BEACON, eSetBits);
    }
}
