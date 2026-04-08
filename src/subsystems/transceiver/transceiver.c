/**
 * @file transceiver.c
 * @brief Interrupt-driven RF transceiver task implementation.
 */

/* ---- Includes ---- */

#include "FreeRTOS.h"
#include "task.h"
#include "transceiver.h"
#include "comms.h"
#include "radiolib_wrapper.h"
#include "health.h"
#include "interleaving.h"
#include "notifications.h"
#include <string.h>


/* ---- Macros ---- */

#define LORA_PREAMBLE_LENGTH   20
#define TX_POST_TX_GUARD_MS    0
#define TX_DONE_TIMEOUT_MS     5000

/* ---- Public function definitions ---- */

void transceiver_task(void *pv_parameters)
{
    (void)pv_parameters;

    QueueHandle_t rx_q = comms_get_rx_queue();
    QueueHandle_t tx_q = comms_get_tx_queue();

    // Register this task for DIO1 notifications
    RadioLib_SetIrqTask(xTaskGetCurrentTaskHandle());

    // Enter RX mode immediately
    RadioLib_StartReceive(LORA_PREAMBLE_LENGTH);

    for (;;) {
        uint32_t notif = 0;
        xTaskNotifyWait(0, N_TRANSCEIVER_RADIO_IRQ_BIT | N_TRANSCEIVER_TX_READY_BIT,
                        &notif, portMAX_DELAY);

        /* Handle radio IRQ (RX_DONE or TX_DONE) */
        if (notif & N_TRANSCEIVER_RADIO_IRQ_BIT) {
            uint32_t irq = RadioLib_GetIrqFlags();

            if (irq & RADIOLIB_SX126X_IRQ_RX_DONE) {
                RxPacket_t pkt = {0};
                int16_t st = RadioLib_ReadRxData(pkt.data, COMMS_PKT_SIZE,
                                                 &pkt.length, &pkt.rssi, &pkt.snr);
                if (st == RADIOLIB_ERR_NONE && pkt.length > 0) {
                    Deinterleave(pkt.data, pkt.length);
                    xQueueSend(rx_q, &pkt, 0);
                }
            }
            RadioLib_ClearIrqFlags(RADIOLIB_SX126X_IRQ_ALL);
        }

        /* Handle TX ready (packet available in tx_queue) */
        if (notif & N_TRANSCEIVER_TX_READY_BIT) {
            TxQueueEntry_t entry;
            while (xQueueReceive(tx_q, &entry, 0) == pdTRUE) {
                uint8_t tx_buf[COMMS_PKT_SIZE];
                memcpy(tx_buf, entry.data, entry.length);
                Interleave(tx_buf, entry.length);

                RadioLib_Standby();
                RadioLib_StartTransmit(tx_buf, entry.length);

                // Wait for TX_DONE IRQ before sending next packet
                xTaskNotifyWait(0, N_TRANSCEIVER_RADIO_IRQ_BIT, NULL,
                                pdMS_TO_TICKS(TX_DONE_TIMEOUT_MS));
                RadioLib_ClearIrqFlags(RADIOLIB_SX126X_IRQ_ALL);

                vTaskDelay(pdMS_TO_TICKS(TX_POST_TX_GUARD_MS));
            }
        }

        // Back to RX after handling everything
        RadioLib_StartReceive(LORA_PREAMBLE_LENGTH);
        health_kick(HEALTH_BIT_COMMS);
    }
}
