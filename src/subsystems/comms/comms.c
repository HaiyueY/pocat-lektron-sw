
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


/* ---- Macros and constants ---- */

#define TX_OUTPUT_POWER                     18        // dBm

#define LORA_BANDWIDTH                      0         // [0: 125 kHz,
                                                      //  1: 250 kHz,
                                                      //  2: 500 kHz,
                                                      //  3: Reserved]
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


/* ---- Type definitions ---- */

typedef enum {
        STARTUP,
        TRANSMIT,
        RECEIVE,
        SLEEP,
        STANDBY,
} CommsState_t;

typedef struct {
    uint8_t RxData[48];
    uint8_t TxData[48];
    uint8_t packetWindow;
} CommsPackets_t;

typedef struct {
    int cadMode;
    int callbackFinished;
    int cadRx;
    int txAck;
    int txPayload;
} CommsFlags_t;

typedef struct {
    uint32_t RF_F; 
    uint32_t sleepTime;
    uint32_t rxTime;
    uint16_t ackTime;
} CommsSettings_t;


/* ---- Module-level variables ---- */

// COMMS State Machine starts in startup state
static CommsState_t CommsState = STARTUP;

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
    .txAck = 0,
    .txPayload = 0
};

// COMMS configuration structure inicialization
static CommsSettings_t CommsSettings = {
    .RF_F = 868000000, //NAME???
    .sleepTime = 10000, // Sleep time in milliseconds
    .rxTime = 2000,
    .ackTime = 4000
};


/* ---- Private function prototypes ---- */
void NextState(void);
void ProcessRadioCallbacks(void);
void Startup(void);
void Sleep(void);
void Receive(void);
void Transmit(void);
void StandBy(void);

/*!
 * \brief Function to be executed on Radio Rx Done event
 */
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr ); 

/*!
 * \brief Function to be executed on Radio Tx Done event
 */
void OnTxDone( void );


/*!
 * \brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Error event
 */
void OnRxError( void );

/*!
 * \brief Function executed on Radio CAD Done event
 */
void OnCadDone( int channelActivityDetected);

/**
 * @brief Configures the SX1262 module with specified parameters.
 *
 * @param SF Spreading factor.
 * @param CR Coding rate.
 * @param RF_F RF frequency.
 */
void SX1262Config(uint8_t SF, uint8_t CR, uint32_t RF_F);

/*!
 * \brief Function configuring CAD parameters
 * \param [in]  cadSymbolNum   The number of symbol to use for CAD operations
 *                             [LORA_CAD_01_SYMBOL, LORA_CAD_02_SYMBOL,
 *                              LORA_CAD_04_SYMBOL, LORA_CAD_08_SYMBOL,
 *                              LORA_CAD_16_SYMBOL]
 * \param [in]  cadDetPeak     Limit for detection of SNR peak used in the CAD
 * \param [in]  cadDetMin      Set the minimum symbol recognition for CAD
 * \param [in]  cadTimeout     Defines the timeout value to abort the CAD activity
 */
// need to implement RadioLoRaCadSymbols_t
// void SX126xConfigureCad( RadioLoRaCadSymbols_t cadSymbolNum, uint8_t cadDetPeak, uint8_t cadDetMin , uint32_t cadTimeout);

void TxPrepare(uint8_t messageType);

void Interleave(uint8_t *inputarr, int size);

void Deinterleave(uint8_t *inputarr, int size);

/* ---- Public function definitions ---- */


// Function prototypes
void setup_comms(void);
void process_comms(void);


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
    printf("Setting up COMMS...\r\n");
}

void process_comms(void)
{
    return;
    switch(CommsState)
    {   

        case STARTUP:
            Startup(); break; // Done // Correspondiente a Startup en cmake_comms

        case SLEEP:
            Sleep(); break; // Done // Correspondiente a una mitad de Sleep en cmake_comms.
            // Falta COMMS_DEBUG_MODE

        case RECEIVE:
            Receive(); break; // Done // Correspondiente a la otra mitad de Sleep en cmake_comms, 
            // añadiendole la recepción de ACKs que eso forma parte de RX en cmake_comms

        case TRANSMIT:
            Transmit(); break; // To do
        
        case STANDBY:
            StandBy(); break; // To do

    }

    NextState();
}

void NextState(void) // CAMBIOS DE ESTADO SE HACEN AQUÍ
{
    // This function is called at the end of each state to determine the next state
    switch(CommsState)
    {
        case STARTUP:
            CommsState = SLEEP; break;

        case SLEEP:
            CommsState = RECEIVE; break;

        case TRANSMIT:
            CommsState = SLEEP; break;

        case STANDBY:
            CommsState = SLEEP; break;
        
        case RECEIVE:
            CommsState = SLEEP; break;

        default:
            break;
            //ProcessRadioCallbacks(); break; // default si el cambio de estado depende del resultado de un callback
            // en este caso el estado se cambia al final de cada callback
    }
}

// MIRAR ?? 
/*
void ProcessRadioCallbacks(void) 
{
    while (!CommsFlags.callbackFinished) {
        RadioLib_IrqProcess();
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    CommsFlags.callbackFinished = 0;
}
    */


// Unica funcio que esta en RADIOLIB
// BoardInitMcu(); tret
void Startup(void)
{
    RadioEvents.TxDone = OnTxDone; 
    RadioEvents.RxDone = OnRxDone; 
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;
    RadioEvents.CadDone = OnCadDone;

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

}

// Sleep state 
//    COMMS_DEBUG_MODE -> constante <- HAVE TO IMPLEMENT
void Sleep(void) 
{   
    RadioLib_Sleep();
    vTaskDelay(pdMS_TO_TICKS(CommsSettings.sleepTime));
}

// Receive state 
//    COMMS_DEBUG_MODE -> constante <- HAVE TO IMPLEMENT
//    HAVE TO IMPLEMENT ACK RECEPTION
//    HAVE IMPLEMENTED CAD MODE
void Receive(void)
{
    if (CommsFlags.cadMode) {

        if (CommsFlags.cadRx) {
            // CAD detecta si hi ha activitat
            CommsFlags.cadRx = 0;
            RadioLib_Rx(CommsSettings.rxTime);
        } else {
            // Fem CAD real (scanChannel)
            RadioLib_StartCad();
        }

    } else {
        // Sense CAD: recepció directa
        RadioLib_Rx(CommsSettings.rxTime);
    }
}


void Transmit(void)
{
    // Tractar el ACK
    if (CommsFlags.txAck)
    {
        memset(CommsPackets.TxData, 0, sizeof(CommsPackets.TxData)); // netejar el buffer

        TxPrepare(ACK_M);

        Interleave(CommsPackets.TxData, COMMS_PKT_SIZE);

        RadioLib_Send(CommsPackets.TxData, COMMS_PKT_SIZE);

        return;
    }

    // Payload data ACABAR
    if (CommsFlags.txPayload)
    {
        // Fer tot lo de preparar dades
        return;
    }

    // Si arribes aquí, no hi ha res a transmetre
    CommsState = SLEEP;
}


// Queda UPLOAD_COMMS_CONFIG y COMMS_UPLOAD_PARAMS y OBC_SOFT_REBOOT
void StandBy(void) 
{
    // switch(CommsFlags.tlcReceived)
    // {

    //     default:
    // }
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    memset(CommsPackets.RxData,0,sizeof(CommsPackets.RxData));
    memcpy(CommsPackets.RxData, payload, size);
    Deinterleave(CommsPackets.RxData,size);

    // ??
    // RssiValue = rssi; 
    // SnrValue = snr;
    
    //??
    //RssiMoy = (((RssiMoy*RxCorrectCnt)+RssiValue)/(RxCorrectCnt+1));
    //SnrMoy = (((SnrMoy*RxCorrectCnt)+SnrValue)/(RxCorrectCnt+1));

    // ??
    //xEventGroupSetBits(xEventGroup, COMMS_RXIRQFlag_EVENT);
    if (CommsPackets.RxData[0]==0xC8 && CommsPackets.RxData[1]==0x9D)
    {
        int ack = tc_process(CommsPackets.RxData, &tc_handles);
        if (ack) {
            CommsFlags.txAck = 1;
            CommsState = TRANSMIT;
        } else {
            CommsState = SLEEP;
        }
    }
    else
    {
		// COMMSNotUs++;
		memset(CommsPackets.RxData,0,sizeof(CommsPackets.RxData));
        RadioLib_Standby();
		CommsState = STANDBY;
    }
}

void OnTxDone(void)
{
    // neteja de flags i variables
    CommsFlags.txAck = 0;
    CommsFlags.txPayload = 0;

    RadioLib_Standby();
    CommsState = SLEEP;
}


// COMMSRxErrors
void OnRxError( void )
{
    RadioLib_Standby();
    CommsState = RECEIVE;
}

// COMMSRxTimeouts
// ADCS_counter = 1;
// TLE_counter = 1;
void OnRxTimeout( void)
{
    CommsState = SLEEP;
}

void OnTxTimeout(void) 
{
    // neteja de flags i variables
    CommsFlags.txAck = 0;
    CommsFlags.txPayload = 0;

    RadioLib_Standby();
    CommsState = STANDBY;
}

void OnCadDone( int channelActivityDetected)
{
    if (channelActivityDetected == 1) {
        CommsFlags.cadRx = 1;
        CommsState = RECEIVE; // If channel activity is detected stay in RECEIVE state
    }
    else {
        RadioLib_Standby();
        CommsState = SLEEP;
    }
}

void SX1262Config(uint8_t SF, uint8_t CR, uint32_t RF_F)
{
    /* Reads the SF, CR and time between packets variables from memory */
    /* Configuration of the LoRa frequency and TX and RX parameters */
    RadioLib_SetChannel(RF_F);
    // estam ya lo hacemos en setup_comms
    //RadioLib_SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH, SF, CR,
    //                                LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
    //                                1, 0, 0, LORA_IQ_INVERSION_ON, 3000 );

    //RadioLib_SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, SF, CR, 0, LORA_PREAMBLE_LENGTH,
    //                                LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
    //                                0, 1, 0, 0, LORA_IQ_INVERSION_ON, 1 );

}

// // need to implement RadioLoRaCadSymbols_t
// void SX126xConfigureCad(RadioLoRaCadSymbols_t cadSymbolNum, uint8_t cadDetPeak, uint8_t cadDetMin , uint32_t cadTimeout)
// {   
//     SX126xSetDioIrqParams( 	IRQ_CAD_DONE | IRQ_CAD_ACTIVITY_DETECTED, IRQ_CAD_DONE | IRQ_CAD_ACTIVITY_DETECTED,
//                                     IRQ_RADIO_NONE, IRQ_RADIO_NONE );

//     SX126xSetCadParams(cadSymbolNum, cadDetPeak, cadDetMin, LORA_CAD_RX, cadTimeout);
//     //THE TOTAL CAD TIMEOUT CAN BE EQUAL TO RX TIMEOUT (IT SHALL NOT BE HIGHER THAN 4 SECONDS)
// }

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

void Interleave(uint8_t *inputarr, int size) 
{
    // Check that the size is a multiple of 6.
    if (size % 6 != 0) {
        return;
    }

    int groupSize = size / 6;

    // Allocate temporary array to hold the interleaved result.
    uint8_t *temp =(uint8_t *) malloc(size * sizeof(uint8_t));
    if (temp == NULL) {
        // EXIT_FAILURE no definido
        //exit(EXIT_FAILURE);
    }

    // For each index within the groups,
    // pick one element from each of the 6 groups.
    for (int i = 0; i < groupSize; i++) {
        for (int j = 0; j < 6; j++) {
            temp[i * 6 + j] = inputarr[j * groupSize + i];
        }
    }

    // Copy the interleaved elements back into the original array.
    memcpy(inputarr, temp, size);

    free(temp);
}

void Deinterleave(uint8_t *inputarr, int size) 
{
    if (size % 6 != 0) {
        return;
    }

    int groupSize = size / 6;
    uint8_t *temp =(uint8_t *) malloc(size * sizeof(uint8_t));
    if (temp == NULL) {
        // EXIT_FAILURE no definido
        //exit(EXIT_FAILURE);
    }

    // Reconstruct the original groups.
    // For each group index i and for each group j:
    // The interleaved array holds the jth element of group j at position i*6 + j.
    // We restore it to temp[ j * groupSize + i ].
    for (int i = 0; i < groupSize; i++) {
        for (int j = 0; j < 6; j++) {
            temp[j * groupSize + i] = inputarr[i * 6 + j];
        }
    }

    memcpy(inputarr, temp, size);
    free(temp);
}

