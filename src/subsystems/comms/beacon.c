/* ---- Includes ---- */

#include "FreeRTOS.h"
#include "timers.h"
#include "beacon.h"
#include "notifications.h"
#include "obc.h"
#include "tx_queue.h"
#include "time.h"
#include "temperature.h"


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

void send_beacon(void)
{
    uint8_t beacon_pkt[12];

    // First 4 bytes are epoch:
    uint32_t epoch = time_get_unix();
    beacon_pkt[0] = (epoch >> 24) & 0xFF;
    beacon_pkt[1] = (epoch >> 16) & 0xFF;
    beacon_pkt[2] = (epoch >> 8) & 0xFF;
    beacon_pkt[3] = epoch & 0xFF;

    // PQ ID (0 for now):
    beacon_pkt[4] = 0;

    // Downlink ID
    beacon_pkt[5] = 0;

    // Temperature MCU
    // TODO: Verify calibration and scaling once ADC is set up. For now, just return raw value in degrees C (e.g. 25 = 25C).
    beacon_pkt[6] = (uint8_t)mcu_get_temperature();

    // Temperature BATT (dummy value for now)
    beacon_pkt[7] = 0xFF; // -1 in two's complement

    // OBC state
    beacon_pkt[8] = obc_get_current_state();

    // MCU supply voltage (Vdda) in units of 0.1V (e.g. 33 = 3.3V)
    // TODO: Get actual battery voltage once ADC is set up and calibrated
    beacon_pkt[9] = (uint8_t)(mcu_get_vdda_mv() / 100);

    // Battery Amp (dummy value for now)
    beacon_pkt[10] = 0xFF; // -1 in two's complement

    // Deployment status
    beacon_pkt[11] = 0; // Not deployed

    txq_enqueue(beacon_pkt, 12, 0, 1);
    return;
}