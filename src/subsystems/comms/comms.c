/**
 * @file comms.c
 * @brief Communications task: protocol logic and packet processing.
 *
 * This task handles all protocol-level operations:
 * - Receiving packets from transceiver_task via rx_queue
 * - Processing TCs with tc_process()
 * - Enqueuing responses/ACKs to tx_queue for transceiver_task to transmit
 * - Managing ARQ pending-ACK window
 * - Generating periodic beacons
 *
 * It does NOT touch RadioLib directly.
 */

/* ---- Includes ---- */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "comms.h"
#include "tc_handler.h"
#include "health.h"
#include "notifications.h"
#include "obc.h"
#include "beacon.h"
#include "radiolib_wrapper.h"
#include <stdio.h>
#include <string.h>

/* ---- Macros and constants ---- */

#define TX_OUTPUT_POWER                     18
#define LORA_BANDWIDTH                      0
#define LORA_PREAMBLE_LENGTH                20
#define LORA_IQ_INVERSION                   0

#define ACK_M  2  /* ACK message type */

#define BEACON_PERIOD_MS  5000  /* must match beacon.h */
#define COMMS_QUEUE_LEN   8

/* ---- Module-level variables ---- */

static QueueHandle_t rx_queue = NULL;
static QueueHandle_t tx_queue = NULL;

static CommsSettings_t CommsSettings = {
    .RF_F = 868000000,
    .sleepTime = 10000,
    .rxTime = 2000,
    .ackTime = 4000
};

/* ARQ pending-ACK window */
static PendingAckEntry_t pending_ack_window[ARQ_WINDOW_SIZE];
static uint8_t next_seq_num = 1;

/* ---- Private function declarations ---- */

static void arq_add_pending(const TxQueueEntry_t *entry, uint8_t seq_num);
static void arq_remove_pending(uint8_t seq_num);
static void arq_check_timeouts(void);
static void enqueue_ack(uint8_t tc_id, uint8_t seq_num);

/* ---- Public function definitions ---- */

void comms_task(void *pv_parameters)
{
    (void)pv_parameters;

    /* Create FreeRTOS queues */
    rx_queue = xQueueCreate(COMMS_QUEUE_LEN, sizeof(RxPacket_t));
    tx_queue = xQueueCreate(COMMS_QUEUE_LEN, sizeof(TxQueueEntry_t));

    if (rx_queue == NULL || tx_queue == NULL) {
        printf("COMMS: Queue creation failed\r\n");
        return;
    }

    /* RadioLib init (but not transceiver hardware; that's transceiver_task's job) */
    if (RadioLib_Init() != 0) {
        printf("COMMS: Radio init failed\r\n");
        return;
    }

    RadioLib_SetChannel(CommsSettings.RF_F);

    /* Configure TX */
    RadioLib_SetTxConfig(
        8,                  /* SF */
        1,                  /* CR 4/5 */
        TX_OUTPUT_POWER,
        LORA_BANDWIDTH,
        LORA_IQ_INVERSION,
        1,                  /* CRC on */
        LORA_PREAMBLE_LENGTH);

    /* Configure RX */
    RadioLib_SetRxConfig(
        8,
        1,
        LORA_BANDWIDTH,
        LORA_IQ_INVERSION,
        1,
        LORA_PREAMBLE_LENGTH);

    /* Initialize pending ACK window */
    memset(pending_ack_window, 0, sizeof(pending_ack_window));

    /* Main loop */
    for (;;) {
        /* Check for config/control notifications (non-blocking) */
        uint32_t notif = 0;
        xTaskNotifyWait(0, 0xFFFFFFFF, &notif, 0);

        if (notif & N_COMMS_NEW_CONFIG) {
            /* TODO: reload config from OBDH */
        }
        if (notif & N_COMMS_NEW_PARAMS) {
            /* TODO: reload params from OBDH */
        }
        if (notif & N_COMMS_STOP_RF) {
            /* TODO: stop RF operations */
        }
        if (notif & N_COMMS_RESUME_RF) {
            /* TODO: resume RF operations */
        }

        /* ARQ retry tick (check for timed-out pending packets) */
        arq_check_timeouts();

        /* Wait for RX packet or beacon timeout */
        RxPacket_t pkt;
        BaseType_t rx_ok = xQueueReceive(rx_queue, &pkt, pdMS_TO_TICKS(BEACON_PERIOD_MS));

        if (rx_ok == pdTRUE) {
            /* Packet received from transceiver_task */
            if (pkt.data[5] == ACK_M) {
                /* Incoming ACK: remove from pending window */
                uint8_t tc_id = pkt.data[2];
                arq_remove_pending(tc_id);
            } else {
                /* Incoming TC: process and send ACK if needed */
                uint8_t tc_id = pkt.data[2];
                int need_ack = tc_process(pkt.data);

                if (need_ack) {
                    enqueue_ack(tc_id, next_seq_num);
                }
            }
        } else {
            /* Queue timeout = beacon period elapsed */
            send_beacon();
        }

        health_kick(HEALTH_BIT_COMMS);
    }
}

QueueHandle_t comms_get_rx_queue(void)
{
    return rx_queue;
}

QueueHandle_t comms_get_tx_queue(void)
{
    return tx_queue;
}

/* ---- Private function definitions ---- */

/**
 * @brief Add an entry to the pending-ACK window (after transmission).
 */
static void arq_add_pending(const TxQueueEntry_t *entry, uint8_t seq_num)
{
    for (int i = 0; i < ARQ_WINDOW_SIZE; i++) {
        if (!pending_ack_window[i].in_use) {
            pending_ack_window[i].entry = *entry;
            pending_ack_window[i].seq_num = seq_num;
            pending_ack_window[i].sent_at = xTaskGetTickCount();
            pending_ack_window[i].retries = 0;
            pending_ack_window[i].in_use = 1;
            return;
        }
    }
    /* Window full — this shouldn't happen if ARQ_WINDOW_SIZE is adequate */
    printf("COMMS: ARQ window full, dropping packet\r\n");
}

/**
 * @brief Remove an entry from the pending-ACK window (ACK received).
 */
static void arq_remove_pending(uint8_t seq_num)
{
    for (int i = 0; i < ARQ_WINDOW_SIZE; i++) {
        if (pending_ack_window[i].in_use && pending_ack_window[i].seq_num == seq_num) {
            pending_ack_window[i].in_use = 0;
            return;
        }
    }
}

/**
 * @brief Check for timed-out pending ACKs and re-enqueue for retransmission.
 */
static void arq_check_timeouts(void)
{
    TickType_t now = xTaskGetTickCount();

    for (int i = 0; i < ARQ_WINDOW_SIZE; i++) {
        if (!pending_ack_window[i].in_use) {
            continue;
        }

        TickType_t elapsed = now - pending_ack_window[i].sent_at;
        if (elapsed > pdMS_TO_TICKS(ACK_TIMEOUT_MS)) {
            /* Timeout: check retry count */
            if (pending_ack_window[i].retries >= ARQ_MAX_RETRIES) {
                /* Give up */
                printf("COMMS: ARQ max retries exceeded for seq=%u\r\n",
                       pending_ack_window[i].seq_num);
                pending_ack_window[i].in_use = 0;
            } else {
                /* Re-enqueue for transmission */
                pending_ack_window[i].retries++;
                TxQueueEntry_t entry = pending_ack_window[i].entry;
                entry.seq_num = pending_ack_window[i].seq_num;

                if (xQueueSend(tx_queue, &entry, pdMS_TO_TICKS(100)) == pdTRUE) {
                    /* Notify transceiver that TX is ready */
                    TaskHandle_t transceiver = obc_get_transceiver_handle();
                    if (transceiver != NULL) {
                        xTaskNotify(transceiver, N_TRANSCEIVER_TX_READY_BIT, eSetBits);
                    }
                    /* Update sent_at timestamp */
                    pending_ack_window[i].sent_at = xTaskGetTickCount();
                } else {
                    printf("COMMS: Failed to re-enqueue packet for retry\r\n");
                }
            }
        }
    }
}

/**
 * @brief Build and enqueue an ACK packet.
 */
static void enqueue_ack(uint8_t tc_id, uint8_t seq_num)
{
    TxQueueEntry_t ack_entry = {0};

    ack_entry.data[0] = 0xC8;
    ack_entry.data[1] = 0x9D;
    ack_entry.data[2] = tc_id;
    ack_entry.data[3] = 0;
    ack_entry.data[4] = 0;
    ack_entry.data[5] = ACK_M;
    ack_entry.length = COMMS_PKT_SIZE;
    ack_entry.seq_num = seq_num;
    ack_entry.is_ack = 1;
    ack_entry.tries = 0;

    if (xQueueSend(tx_queue, &ack_entry, pdMS_TO_TICKS(100)) == pdTRUE) {
        /* Notify transceiver that TX is ready */
        TaskHandle_t transceiver = obc_get_transceiver_handle();
        if (transceiver != NULL) {
            xTaskNotify(transceiver, N_TRANSCEIVER_TX_READY_BIT, eSetBits);
        }
    }
}
