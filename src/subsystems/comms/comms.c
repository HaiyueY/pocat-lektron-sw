
/* ---- Includes ---- */

#include "FreeRTOS.h"
#include "task.h"
#include "comms.h"
#include "tc_handler.h"
#include "radiolib_wrapper.h"
#include "health.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "interleaving.h"


/* ---- Macros and constants ---- */

#define TX_OUTPUT_POWER                     18        // dBm

#define LORA_BANDWIDTH                      0         // [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
#define LORA_PREAMBLE_LENGTH                8//108    // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                 100       // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON          0
#define LORA_IQ_INVERSION_ON                0

//     Transmit message types 
#define ACK_M     							2
#define DATA_M  							3

//     CAD parameters
#define CAD_SYMBOL_NUM          LORA_CAD_02_SYMBOL
#define CAD_DET_PEAK            23
#define CAD_DET_MIN             1

#define MODEM_LORA              1 // To-do: revise

#define COMMS_PKT_SIZE 48


/* ---- Module-level variables ---- */

// COMMS State Machine starts in startup state
static CommsState_t CommsState = SLEEP;

// Task handles for telecommand notification targets (populated during init)
static tc_task_handles_t tc_handles = {0}; 

// Radio event handler struct
static RadioEvents_t RadioEvents;

static CommsPackets_t CommsPackets = {
    .packetWindow = 5
};

static CommsFlags_t CommsFlags = {
    .cadMode = 1, // set to 0 in original code
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


// DEFINITIONS

void comms_task(void *pv_parameters)
{

    setup_comms();

    for(;;) 
    {
        process_comms();
        health_kick(HEALTH_BIT_COMMS);
        vTaskDelay(pdMS_TO_TICKS(1000));
        //printf("COMMS loop\r\n");

    }
}

void setup_comms(void)
{
    // Apply the default configuration

    // RadioEvents.TxDone = OnTxDone; 
    // RadioEvents.RxDone = OnRxDone; 
    // RadioEvents.TxTimeout = OnTxTimeout;
    // RadioEvents.RxTimeout = OnRxTimeout;
    // RadioEvents.RxError = OnRxError;
    // RadioEvents.CadDone = OnCadDone;

    RadioLib_Init(&RadioEvents);  // Initializes the Radio with radiolib
    
    RadioLib_SetChannel(CommsSettings.RF_F); // Configures the transceiver

    RadioLib_SetTxConfig( // Configura els parametres de TX
        11,                     // SF
        1,                      // CR
        TX_OUTPUT_POWER,        // Potencia de transmissio
        LORA_BANDWIDTH,         // BW
        LORA_IQ_INVERSION_ON,   // IQ
        1,                   // CRC ON
        LORA_PREAMBLE_LENGTH);  // Sequencia la sincronitzacio

    RadioLib_SetRxConfig( // Configura els parametres de RX
        11,                     // SF
        1,                      // CR
        LORA_BANDWIDTH,         // BW
        LORA_IQ_INVERSION_ON,   // IQ
        1,                   // CRC ON
        LORA_PREAMBLE_LENGTH) ; // Sequencia la sincronitzacio

    CommsState = SLEEP; // Start in sleep state

}

void process_comms(void)
{
    // TODO: Notifications    
    // if N_COMMS_NEW_CONFIG        New comms configuration available in memory
    // if N_COMMS_NEW_PARAMS        New parameter set available in memory
    // if N_COMMS_STOP_RF           Stop RF transmission
    // if N_COMMS_RESUME_RF         Resume RF transmission
    // if N_COMMS_TRANSMIT_BEACON   Transmit the beacon

    switch(CommsState)
    {   
        case SLEEP:
            state_sleep(); break;

        case PROCESS:
            state_process(); break;

        case TRANSMIT:
            state_transmit(); break;
    }
}


void state_sleep(void) 
{   
    RadioLib_Sleep();
    vTaskDelay(pdMS_TO_TICKS(CommsSettings.sleepTime));

    // TODO:
    // Receive in CAD mode blocking until timeout or reception, 
    // if timeout -> stay in sleep, 
    // if reception -> save packet in memory and change state to process
    
    // Reference:
    // if (CommsFlags.cadMode) {

    //     if (CommsFlags.cadRx) {
    //         // CAD detecta si hi ha activitat
    //         CommsFlags.cadRx = 0;
    //         RadioLib_Rx(CommsSettings.rxTime);
    //     } else {
    //         // Fem CAD real (scanChannel)
    //         RadioLib_StartCad();
    //     }

    // } else {
    //     // Sense CAD: recepció directa
    //     RadioLib_Rx(CommsSettings.rxTime);
    // }

    // memset(CommsPackets.RxData,0,sizeof(CommsPackets.RxData));
    // memcpy(CommsPackets.RxData, payload, size);
    // Deinterleave(CommsPackets.RxData,size);

    // // ??
    // // RssiValue = rssi; 
    // // SnrValue = snr;
    
    // //??
    // //RssiMoy = (((RssiMoy*RxCorrectCnt)+RssiValue)/(RxCorrectCnt+1));
    // //SnrMoy = (((SnrMoy*RxCorrectCnt)+SnrValue)/(RxCorrectCnt+1));

    // // ??
    // //xEventGroupSetBits(xEventGroup, COMMS_RXIRQFlag_EVENT);
    // if (CommsPackets.RxData[0]==0xC8 && CommsPackets.RxData[1]==0x9D)
    // {
    //     int ack = tc_process(CommsPackets.RxData, &tc_handles);
    //     if (ack) {
    //         CommsFlags.txAck = 1;
    //         CommsState = TRANSMIT;
    //     } else {
    //         CommsState = SLEEP;
    //     }
    // }
    // else
    // {
	// 	// COMMSNotUs++;
	// 	memset(CommsPackets.RxData,0,sizeof(CommsPackets.RxData));
    //     RadioLib_Standby();
	// 	CommsState = STANDBY;
    // }
}

void state_process(void)
{
   // TODO:
   // Process the received packet, 

   // if telecommmand -> call tc_process (it shall enqueue ack)
   // if ACK -> remove acknoleged packet from tx_queue

   // if tx_queue is not empty -> change state to transmit, 
   // else -> change state to sleep 
}


void state_transmit(void)
{
    // TODO:
    // Transmit once every packet in the tx_queue and change state to sleep when done. 
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


