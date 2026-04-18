/* ---- Includes ---- */

#include "beacon.h"
#include "comms.h"
#include "notifications.h"
#include "obc.h"
#include "time.h"
#include "temperature.h"
#include "flash.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>


/* ---- Public function definitions ---- */

void beacon_task(void *pv_parameters)
{
    (void)pv_parameters;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(BEACON_PERIOD_MS));

        TxQueueEntry_t beacon_entry = {0};

        // First 4 bytes are epoch:
        uint32_t epoch = time_get_unix();
        beacon_entry.data[0] = (epoch >> 24) & 0xFF;
        beacon_entry.data[1] = (epoch >> 16) & 0xFF;
        beacon_entry.data[2] = (epoch >> 8) & 0xFF;
        beacon_entry.data[3] = epoch & 0xFF;

        // PQ ID (0 for now):
        beacon_entry.data[4] = 0;

        // Downlink ID
        beacon_entry.data[5] = 0;

        // Temperature MCU
        beacon_entry.data[6] = (uint8_t)mcu_get_temperature();

        // Temperature BATT (dummy value for now)
        beacon_entry.data[7] = 0xFF;

        // OBC state
        OBDH_Read_Request(CURRENT_STATE_ADDR, &beacon_entry.data[8], sizeof(ObcState_t));

        // MCU supply voltage (Vdda) in units of 0.1V
        beacon_entry.data[9] = (uint8_t)(mcu_get_vdda_mv() / 100);

        // Battery Amp (dummy value for now)
        beacon_entry.data[10] = 0xFF;

        // Deployment status
        beacon_entry.data[11] = 0;

        beacon_entry.length = 12;
        beacon_entry.needs_ack = 0;
        beacon_entry.tries = 0;
        beacon_entry.seq_num = 0;

        QueueHandle_t tx_q = comms_get_tx_queue();
        if (tx_q != NULL && xQueueSend(tx_q, &beacon_entry, pdMS_TO_TICKS(100)) == pdTRUE) {
            TaskHandle_t transceiver = obc_get_transceiver_handle();
            if (transceiver != NULL) {
                xTaskNotify(transceiver, N_TRANSCEIVER_TX_READY_BIT, eSetBits);
            }
        }
    }
}
