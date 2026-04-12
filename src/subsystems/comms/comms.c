
/* ---- Includes ---- */

#include "FreeRTOS.h"
#include "task.h"
#include "comms.h"
#include "tx_queue.h"
#include "tc_handler.h"
#include "radiolib_wrapper.h"
#include "health.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "interleaving.h"
#include "beacon.h"
#include "notifications.h"
#include "events.h"


/* ---- Macros and constants ---- */

#define TX_OUTPUT_POWER                     18        // dBm
#define TX_POST_TX_GUARD_MS                 100       // TODO: have to test

#define LORA_BANDWIDTH                      0         // [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
#define LORA_PREAMBLE_LENGTH                20        // Must be > 2×CAD symbols for reliable CAD→RX. GS must match.
#define LORA_SYMBOL_TIMEOUT                 100       // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON          0
#define LORA_IQ_INVERSION                   0         // 0 = off, 1 = on

//     Transmit message types 
#define ACK_M     							2
#define DATA_M  							3

//     CAD parameters
#define CAD_SYMBOL_NUM          LORA_CAD_02_SYMBOL
#define CAD_DET_PEAK            23
#define CAD_DET_MIN             1

#define MODEM_LORA              1 // To-do: revise


/* ---- Module-level variables ---- */

// COMMS State Machine starts in startup state
static CommsState_t CommsState = SLEEP;
static bool paused;
static uint32_t deferred_notifications;


static CommsPackets_t CommsPackets = {
    .packetWindow = 5
};

static CommsFlags_t CommsFlags = {
    .rxMode = RX_MODE_DUTY_CYCLE,
    .cadRx = 0,
    .callbackFinished = 0,
    // .txAck = 0, legacy. ACK's should be enqueued in tx_queue
    .txPayload = 0
};

// COMMS configuration structure inicialization
static CommsSettings_t CommsSettings = {
    .RF_F = 868000000, //NAME???
    .sleepTime = 10000, // Sleep time in milliseconds
    .rxTime = 2000,
    .ackTime = 4000
};

void TxPrepare(uint8_t messageType);


/* ---- Public function definitions ---- */


// Function prototypes
void setup_comms(void);
void process_comms(void);
void state_sleep(void);
void state_process(void);
void state_transmit(void);
static uint32_t wait_for_notification(void);


// DEFINITIONS

void comms_task(void *pv_parameters)
{
    setup_comms();

    for(;;) 
    {
        process_comms();
        health_kick(HEALTH_BIT_COMMS);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void setup_comms(void)
{
    // Apply the default configuration
    paused = false;
    deferred_notifications = 0;

    // RadioEvents.TxDone = OnTxDone; 
    // RadioEvents.RxDone = OnRxDone; 
    // RadioEvents.TxTimeout = OnTxTimeout;
    // RadioEvents.RxTimeout = OnRxTimeout;
    // RadioEvents.RxError = OnRxError;
    // RadioEvents.CadDone = OnCadDone;

    if (RadioLib_Init() != 0) {
        printf("COMMS: Radio init failed, aborting setup\r\n");
        return;
    }
    
    RadioLib_SetChannel(CommsSettings.RF_F); // Configures the transceiver

    // SF=8, CR=1 to match the CubeCell ground station
    RadioLib_SetTxConfig(
        8,                      // SF
        1,                      // CR  4/5
        TX_OUTPUT_POWER,        // Potència de transmissió
        LORA_BANDWIDTH,         // BW  125 kHz
        LORA_IQ_INVERSION,     // IQ inversion off
        1,                      // CRC on
        LORA_PREAMBLE_LENGTH);  // Preamble 8

    RadioLib_SetRxConfig(
        8,                      // SF 
        1,                      // CR  4/5
        LORA_BANDWIDTH,         // BW  125 kHz
        LORA_IQ_INVERSION,     // IQ inversion off
        1,                      // CRC on
        LORA_PREAMBLE_LENGTH);  // Preamble 8

    CommsState = SLEEP; // Start in sleep state

    beacon_init();
}

void process_comms(void)
{
    // TODO: Notifications
    // if N_COMMS_NEW_CONFIG        New comms configuration available in memory
    // if N_COMMS_NEW_PARAMS        New parameter set available in memory
    // if N_COMMS_STOP_RF           Stop RF transmission
    // if N_COMMS_RESUME_RF         Resume RF transmission

    uint32_t notif = wait_for_notification();

    if (paused) {
        deferred_notifications |= notif & ~(N_TASK_PAUSE | N_TASK_RESUME);
        if (notif & N_TASK_RESUME) {
            paused = false;
            notif = deferred_notifications;
            deferred_notifications = 0;
        }
        else return;
    }

    if (notif & N_TASK_PAUSE) {
        deferred_notifications |= notif & ~(N_TASK_PAUSE | N_TASK_RESUME);
        paused = true;
        xEventGroupSetBits(task_events_handle, EV_TASK_ACK_COMMS);
        return;
    }

    switch(CommsState)
    {
        case SLEEP:
            state_sleep(); break;

        case PROCESS:
            state_process(); break;

        case TRANSMIT:
            state_transmit(); break;
    }

    if (notif & N_COMMS_TRANSMIT_BEACON) {
        send_beacon();
        if (CommsState == SLEEP) {
            CommsState = TRANSMIT;
        }
    }

}

static uint32_t wait_for_notification(void)
{
    uint32_t notificationValue = 0;
    xTaskNotifyWait(0, 0xFFFFFFFF, &notificationValue, 0);
    return notificationValue;
}


void state_sleep(void)
{
    uint16_t rx_len  = 0;
    int16_t  rx_rssi = 0;
    int8_t   rx_snr  = 0;
    int16_t  ret;

    switch (CommsFlags.rxMode) {
        case RX_MODE_CAD:
            ret = RadioLib_CadReceive(CommsSettings.rxTime,   /* CAD scanning budget  . TODO: review*/
                                      CommsSettings.rxTime,   /* RX timeout after CAD */
                                      CommsPackets.RxData, COMMS_PKT_SIZE,
                                      &rx_len, &rx_rssi, &rx_snr);
            break;
        case RX_MODE_DUTY_CYCLE:
            ret = RadioLib_DutyCycleReceive(CommsSettings.rxTime, LORA_PREAMBLE_LENGTH,
                                            CommsPackets.RxData, COMMS_PKT_SIZE,
                                            &rx_len, &rx_rssi, &rx_snr);
            break;
        default: 
            ret = RadioLib_Receive(CommsSettings.rxTime,
                                   CommsPackets.RxData, COMMS_PKT_SIZE,
                                   &rx_len, &rx_rssi, &rx_snr);
            break;
    }

    if (ret != 0) { return; }

    Deinterleave(CommsPackets.RxData, (int)rx_len);

    /* TODO: Validate that the packet is ours and is correct */

    CommsState = PROCESS;
}

void state_process(void)
{

    if (CommsPackets.RxData[5] == ACK_M) {
        TxQueueEntry_t *head = txq_peek();
        if (head != NULL && head->stop_and_wait) {
            txq_dequeue();
        }
    } else {
        uint8_t tc_id = CommsPackets.RxData[2];
        int need_ack = tc_process(CommsPackets.RxData);
        if (need_ack) {
            uint8_t ack_pkt[COMMS_PKT_SIZE] = {0};
            ack_pkt[0] = 0xC8;   
            ack_pkt[1] = 0x9D;   
            ack_pkt[2] = tc_id;  // TC id so that GS knows what is being ACKed
            ack_pkt[3] = 0;
            ack_pkt[4] = 0;
            ack_pkt[5] = ACK_M;
            txq_enqueue(ack_pkt, COMMS_PKT_SIZE, 0, 1);
        }
    }

    CommsState = txq_is_empty() ? SLEEP : TRANSMIT;
}


void state_transmit(void)
{
    TxQueueEntry_t *entry = txq_peek();
    if (entry == NULL) {
        CommsState = SLEEP;
        return;
    }

    entry->tries++;

    /* Interleave a copy so the queued data stays intact for retransmission */
    uint8_t tx_buf[entry->length];
    memcpy(tx_buf, entry->data, entry->length);
    Interleave(tx_buf, entry->length);
    RadioLib_Transmit(tx_buf, (uint16_t)entry->length);

    if (entry->stop_and_wait) {
        CommsState = SLEEP;
    } else {
        txq_dequeue();
        int queue_empty = txq_is_empty();
        if (!queue_empty) {
            vTaskDelay(pdMS_TO_TICKS(TX_POST_TX_GUARD_MS));
        }
        CommsState = queue_empty ? SLEEP : TRANSMIT;
    }
}



//ack_m or OP?? simply name convention
void TxPrepare(uint8_t messageType) {
    uint32_t unixTime32 = 0; //(uint32_t) HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN); // to-do: mutex
    switch(messageType)
    {
        case ACK_M:
            // look into it

        case DATA_M:
            // look into it

		    break;


    }
    CommsPackets.TxData[0] = (unixTime32 >> 24) & 0xFF;
    CommsPackets.TxData[1] = (unixTime32 >> 16) & 0xFF;
    CommsPackets.TxData[2] = (unixTime32 >> 8) & 0xFF;
    CommsPackets.TxData[3] = unixTime32 & 0xFF;
    CommsPackets.TxData[4] = 0;
    CommsPackets.TxData[5] = messageType;

    // COMENTED BECAUSE TOTALPACKETSIZE NOT DEFINED Interleave((uint8_t*) CommsPackets.TxData, totalpacketsize); // aqui he simplificado el proceso que se hacia en cmake_comms,
    // antes se hacia un memcpy de TxData a un buffer Encoded_Packet. Alomejor es necesario hacerlo asi más adelante
}


