#ifndef RADIO_MOCK
#include "radiolib_wrapper.h"

/* C++ headers */
#include <RadioLib.h>
#include "stm32_radiolib_hal.h"
#include <stdio.h>


// Instantiate C++ outside of extern "C"
static stm32RadioLibHal hal(&hspi2);  

// Pin encoding: (portIndex << 16) | GPIO_PIN_x
// Port index: A=0, B=1, C=2, D=3, ...

#define RADIO_PIN_NSS    ((1 << 16) | GPIO_PIN_12)  // PB12 - SX1262 Chip Select
#define RADIO_PIN_DIO1   ((0 << 16) | GPIO_PIN_10)  // PA10 - SX1262 Interrupt (DIO1)
#define RADIO_PIN_RESET  ((2 << 16) | GPIO_PIN_9)   // PC9  - SX1262 Reset
#define RADIO_PIN_BUSY   ((0 << 16) | GPIO_PIN_8)   // PA8  - SX1262 Busy Indicator

// Module constructor order: Module(hal, cs, irq, rst, gpio)
//   cs   = NSS         (PB12)
//   irq  = DIO1        (PA10)
//   rst  = SX1262_NRST (PC9)
//   gpio = BUSY        (PA8)
static Module mod(&hal, RADIO_PIN_NSS, RADIO_PIN_DIO1, RADIO_PIN_RESET, RADIO_PIN_BUSY);
static SX1262 radio(&mod);

static float bwCodeToKHz(uint8_t bw_code) {
  switch(bw_code) {
    case 0: return 125.0f;
    case 1: return 250.0f;
    case 2: return 500.0f;
    default: return 125.0f;
  }
}

extern "C" { // to stop name mangling

    int16_t RadioLib_Init(void) {
        printf("RadioLib_Init: calling radio.begin()...\r\n");

        // SX1262MB2xAS uses a crystal oscillator (XTAL), not a TCXO.
        // Must set this BEFORE begin() so RadioLib skips DIO3 TCXO setup.
        radio.XTAL = true;
        const int16_t state = radio.begin(
            434.0,  
            125.0,  
            9,      
            7,       
            RADIOLIB_SX126X_SYNC_WORD_PRIVATE,
            10,     
            8,      
            0.0,     // tcxoVoltage = 0 (no TCXO on this board)
            false   
        );
        if(state != RADIOLIB_ERR_NONE) {
            printf("Error initializing radio: %d\r\n", state);
        }
        return state;
    }

    void RadioLib_SetChannel(uint32_t freq_hz) {
       float freq_mhz = freq_hz / 1000000.0f;
       const int state = radio.setFrequency(freq_mhz); //esperem MHz
       if(state != RADIOLIB_ERR_NONE) {
           // Handle error (could add error callback or logging)
           //printf("Error en la configuracio de la frequencia: %d\n", state);
       }
    }

    void RadioLib_SetTxConfig(uint8_t sf, uint8_t cr, int8_t power,
                            uint8_t bw, int iqInverted,
                            int crcOn, uint16_t preambleLen)
    {
        int state;

        // Set output power
        state = radio.setOutputPower(power);
        if (state != RADIOLIB_ERR_NONE) {
            // Handle error
        }

        // Set spreading factor
        state = radio.setSpreadingFactor(sf);
        if (state != RADIOLIB_ERR_NONE) {
            // Handle error
        }

        // Set coding rate (RadioLib CR uses values 5–8)
        state = radio.setCodingRate(cr + 4);
        if (state != RADIOLIB_ERR_NONE) {
            // Handle error
        }

        // Set bandwidth
        state = radio.setBandwidth(bwCodeToKHz(bw));
        if (state != RADIOLIB_ERR_NONE) {
            // Handle error
        }

        // CRC enable/disable
        state = radio.setCRC(crcOn);
        if (state != RADIOLIB_ERR_NONE) {
            // Handle error
        }

        // Preamble length
        state = radio.setPreambleLength(preambleLen);
        if (state != RADIOLIB_ERR_NONE) {
            // Handle error
        }

        // Note: IQ inversion is not a separate method in RadioLib SX1262
        // If needed, handle in modulation settings or ignore.
    }

    void RadioLib_SetRxConfig(uint8_t sf, uint8_t cr, uint8_t bw,
                            int iqInverted, int crcOn,
                            uint16_t preambleLen)
    {
        int state;

        // Set spreading factor
        state = radio.setSpreadingFactor(sf);
        if (state != RADIOLIB_ERR_NONE) {
            // Handle error
        }

        // Set coding rate
        state = radio.setCodingRate(cr + 4);
        if (state != RADIOLIB_ERR_NONE) {
            // Handle error
        }

        // Set bandwidth
        state = radio.setBandwidth(bwCodeToKHz(bw));
        if (state != RADIOLIB_ERR_NONE) {
            // Handle error
        }

        // CRC enable/disable
        state = radio.setCRC(crcOn);
        if (state != RADIOLIB_ERR_NONE) {
            // Handle error
        }

        // Preamble length
        state = radio.setPreambleLength(preambleLen);
        if (state != RADIOLIB_ERR_NONE) {
            // Handle error
        }

        // Note: IQ inversion doesn’t have a separate setter in RadioLib SX1262
    }


    int16_t RadioLib_Receive(uint32_t timeoutMs,
                        uint8_t *outBuf, uint16_t bufSize,
                        uint16_t *outLen, int16_t *outRssi, int8_t *outSnr)
    {
        int16_t st = radio.receive(outBuf, bufSize, timeoutMs);

        if (st == RADIOLIB_ERR_NONE) {
            if (outLen)  *outLen  = radio.getPacketLength();
            if (outRssi) *outRssi = radio.getRSSI();
            if (outSnr)  *outSnr  = radio.getSNR();
        }
        return st;
    }

    int16_t RadioLib_Transmit(uint8_t *buf, uint16_t len) {
        return radio.transmit(buf, len);
    }

    int16_t RadioLib_Sleep(void) {
        return radio.sleep();
    }

    int16_t RadioLib_Standby(void) {
        return radio.standby();
    }

    int16_t RadioLib_ScanChannel(void) {
        return radio.scanChannel();
    }

    // CAD + RX
    int16_t RadioLib_CadReceive(uint32_t rxTimeoutMs,
                           uint8_t *outBuf, uint16_t bufSize,
                           uint16_t *outLen, int16_t *outRssi, int8_t *outSnr)
    {
        ChannelScanConfig_t cfg = {
            .cad = {
                .symNum = RADIOLIB_SX126X_CAD_PARAM_DEFAULT,
                .detPeak = RADIOLIB_SX126X_CAD_PARAM_DEFAULT,
                .detMin = RADIOLIB_SX126X_CAD_PARAM_DEFAULT,
                .exitMode = RADIOLIB_SX126X_CAD_GOTO_RX,
                .timeout = (RadioLibTime_t)rxTimeoutMs * 1000UL,
                .irqFlags = RADIOLIB_IRQ_CAD_DEFAULT_FLAGS,
                .irqMask = RADIOLIB_IRQ_CAD_DEFAULT_MASK,
            },
        };

        int16_t cadResult = radio.scanChannel(cfg);
        if (cadResult != RADIOLIB_LORA_DETECTED) {
            return cadResult;
        }
        bool softTimeout = false;
        RadioLibTime_t start = hal.millis();
        for (;;) {
            uint32_t irq = radio.getIrqFlags();
            if (irq & RADIOLIB_SX126X_IRQ_RX_DONE)   break;
            if (irq & RADIOLIB_SX126X_IRQ_TIMEOUT)    { softTimeout = true; break; }
            if (hal.millis() - start > rxTimeoutMs)    { softTimeout = true; break; }
            hal.yield();
        }

        // receive should not be called here, but following radiolibs implementation of receive: 
        int16_t state = radio.standby();
        if ((state != RADIOLIB_ERR_NONE) && (state != RADIOLIB_ERR_SPI_CMD_TIMEOUT)) {
            return state;
        }

        if (softTimeout || (radio.getIrqFlags() & RADIOLIB_SX126X_IRQ_TIMEOUT)) {
            (void)radio.finishReceive();
            return RADIOLIB_ERR_RX_TIMEOUT;
        }

        size_t pktLen = radio.getPacketLength();
        if (pktLen > bufSize) pktLen = bufSize;

        int16_t st = radio.readData(outBuf, pktLen);
        if (st == RADIOLIB_ERR_NONE) {
            if (outLen)  *outLen  = (uint16_t)pktLen;
            if (outRssi) *outRssi = (int16_t)radio.getRSSI();
            if (outSnr)  *outSnr  = (int8_t)radio.getSNR();
        }

        return st;
    }

    void RadioLib_IrqProcess(void) {
        // De moment no fa falta implementar res aqui ja que radiolib gestiona els interrupts internament.
        // No obstant, si ens posem en un mode no bloquejant, potser caldra implementar alguna cosa aqui.
    }

}
#endif /* RADIO_MOCK */
