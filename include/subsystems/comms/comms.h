/**
 * @file comms.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2026-01-20
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef INC_COMMS_H_
#define INC_COMMS_H_

#include <stdint.h>

/* ---- Constants ---- */

#define COMMS_PKT_SIZE 48

/* ---- Type definitions ---- */

typedef enum {
        SLEEP,
        PROCESS,
        TRANSMIT,

} CommsState_t;

typedef struct {
    uint8_t RxData[COMMS_PKT_SIZE];
    uint8_t TxData[COMMS_PKT_SIZE];
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

/**
 * @brief Communications task function, it runs the COMMS state machine.
 */
void comms_task(void *pv_parameters);


#endif /* INC_COMMS_H_ */