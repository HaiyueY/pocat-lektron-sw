#include "stm32_radiolib_hal.h"

// Realizamos un callback forwarding de la funcion de stm32 HAL a nuestra implementacion en stm32RadioLibHal
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    // Forward to our static method
    stm32RadioLibHal::handleExtiCallback(GPIO_Pin);
}


void (*stm32RadioLibHal::_extiCallbacks[16])(void) = { nullptr };


// for radiolib use with stm32 spi and tim5 have to be correctly initialized in main.c


// ----------------Helper private functions ---------------------

// Los pines los codificamos como un uint32_t donde los 16 bits superiores son el port y los 16 inferiores el pin (port+pin)


GPIO_TypeDef* stm32RadioLibHal::getPort(uint32_t pin) { // helper for extracting port from port+pin number
    uint32_t portIndex = (pin >> 16) & 0xFF;
    switch (portIndex) {
        case 0: return GPIOA;   
        case 1: return GPIOB;
        case 2: return GPIOC;
        case 3: return GPIOD;
        case 4: return GPIOE;
        case 5: return GPIOF;
        default: return GPIOA; // fallback
    }
}

// Extract the pin mask (lower 16 bits)
uint16_t stm32RadioLibHal::getPinMask(uint32_t pin) { // helper for extracting pin from port+pin number
    return (uint16_t)(pin & 0xFFFF);
}

void stm32RadioLibHal::enablePortClock(GPIO_TypeDef* port) {
    if (port == GPIOA)      __HAL_RCC_GPIOA_CLK_ENABLE();
    else if (port == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
    else if (port == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();
    else if (port == GPIOD) __HAL_RCC_GPIOD_CLK_ENABLE();
    else if (port == GPIOE) __HAL_RCC_GPIOE_CLK_ENABLE();
    else if (port == GPIOF) __HAL_RCC_GPIOF_CLK_ENABLE();
}


// ---------------- stm32RadioLibHal implementation ---------------------

stm32RadioLibHal::stm32RadioLibHal(SPI_HandleTypeDef* spi) 
    : RadioLibHal(
        GPIO_MODE_INPUT,
        GPIO_MODE_OUTPUT_PP, // push-pull output
        GPIO_PIN_RESET,
        GPIO_PIN_SET,
        GPIO_MODE_IT_RISING,
        GPIO_MODE_IT_FALLING
      ),
      _spi(spi),
      _startMillis(0) {
}
    
// para utilitzar esta fucion vamos a tener que codificar el pin y el port en el primer parametero "pin"
// we have to be able to enable interrupt for atachInterrupt
void stm32RadioLibHal::pinMode(uint32_t pin, uint32_t mode) { 

    if(pin == RADIOLIB_NC) {
        return;
    }

    // recogemos puerto y pin:
    GPIO_TypeDef* port = getPort(pin);
    uint16_t pinMask = getPinMask(pin);

    /* GPIO Ports Clock Enable */
    enablePortClock(port);

    /*Configure GPIO pin Output Level */
    // Do I have to use HAL_GPIO_WritePin???

    /*Configure GPIO pin :  */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = pinMask;
    GPIO_InitStruct.Pull = GPIO_NOPULL; 
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Mode = mode;
    HAL_GPIO_Init(port, &GPIO_InitStruct);

    // What about this here?? Look into it:
    // HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
    // HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
    // HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    // to do
    
}

void stm32RadioLibHal::digitalWrite(uint32_t pin, uint32_t value) {
    if(pin == RADIOLIB_NC) {
        return;
    }
    GPIO_TypeDef* port = getPort(pin);
    uint16_t pinMask = getPinMask(pin);
    HAL_GPIO_WritePin(
        port,
        pinMask,
        (value == GpioLevelHigh) ? GPIO_PIN_SET : GPIO_PIN_RESET
    );
  
}

uint32_t stm32RadioLibHal::digitalRead(uint32_t pin) {
    if (pin == RADIOLIB_NC) {
        return GpioLevelLow;    // or simply 0
    }
    GPIO_TypeDef* port = getPort(pin);
    uint16_t pinMask = getPinMask(pin);
    return (HAL_GPIO_ReadPin(port, pinMask) == GPIO_PIN_SET) ? GpioLevelHigh : GpioLevelLow;
}


// verify this is the code generated when setting up an external interrupt with STM32CubeMX. Do it for for different lines
void stm32RadioLibHal::attachInterrupt(uint32_t interruptNum, void (*interruptCb)(void), uint32_t mode) {
    if (interruptNum == RADIOLIB_NC || interruptCb == nullptr) {
        return;
    }
    // tratamos interruptNum como un port+pin
    GPIO_TypeDef* port = getPort(interruptNum);
    uint16_t pinMask = getPinMask(interruptNum);
    int line = getExtiLineFromPinMask(pinMask); //extract exti line
    if (line < 0 || line > 15) { // validate line
        return;    // invalid pin
    }
    
    // what is this for??   
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    // configure GPIO
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin   = pinMask;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;          // Change if needed
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Mode  = mode;                 // Already a HAL mode from RadioLib (RISING/FALLING)
    HAL_GPIO_Init(port, &GPIO_InitStruct);

    // Store callback for this EXTI line
    _extiCallbacks[line] = interruptCb;

    // Enable appropriate NVIC IRQ
    IRQn_Type irqn;
    if (line <= 4) {
        static const IRQn_Type table[5] = {
            EXTI0_IRQn, EXTI1_IRQn, EXTI2_IRQn, EXTI3_IRQn, EXTI4_IRQn
        };
        irqn = table[line];
    } else if (line <= 9) {
        irqn = EXTI9_5_IRQn;
    } else {
        irqn = EXTI15_10_IRQn;
    }

    // set priority and enable
    HAL_NVIC_SetPriority(irqn, 5, 0);
    HAL_NVIC_EnableIRQ(irqn);
}

// __HAL_GPIO_EXTI_CLEAR_IT(pinMask);???
void stm32RadioLibHal::detachInterrupt(uint32_t interruptNum) {
    if (interruptNum == RADIOLIB_NC) {
        return;
    }

    GPIO_TypeDef* port = getPort(interruptNum);
    uint16_t pinMask   = getPinMask(interruptNum);
    int line = getExtiLineFromPinMask(pinMask);
    if (line < 0 || line > 15) {
        return;
    }

    _extiCallbacks[line] = nullptr;

#if defined(EXTI)
    EXTI->IMR  &= ~pinMask;   // mask interrupt
    EXTI->EMR  &= ~pinMask;   // mask event (optional)
#endif

    IRQn_Type irqn;

    if (line <= 4) {
        static const IRQn_Type table[5] = {
            EXTI0_IRQn, EXTI1_IRQn, EXTI2_IRQn, EXTI3_IRQn, EXTI4_IRQn
        };
        irqn = table[line];
        HAL_NVIC_DisableIRQ(irqn);
    }
    else {
        int groupStart, groupEnd;
        if (line <= 9) {
            irqn = EXTI9_5_IRQn;
            groupStart = 5;
            groupEnd   = 9;
        }
        else {
            irqn = EXTI15_10_IRQn;
            groupStart = 10;
            groupEnd   = 15;
        }
        bool anyUsed = false;
        for (int i = groupStart; i <= groupEnd; ++i) {
            if (_extiCallbacks[i] != nullptr) {
                anyUsed = true;
                break;
            }
        }
        if (!anyUsed) {
            HAL_NVIC_DisableIRQ(irqn);
        }
    }
}

void stm32RadioLibHal::delay(RadioLibTime_t ms) {
#if !defined(RADIOLIB_CLOCK_DRIFT_MS)
    HAL_Delay(ms);
#else
    HAL_Delay(ms * 1000 / (1000 + RADIOLIB_CLOCK_DRIFT_MS));
#endif
}

void stm32RadioLibHal::delayMicroseconds(RadioLibTime_t us) {
#if !defined(RADIOLIB_CLOCK_DRIFT_MS)
    RadioLibTime_t start = micros();
    while ((micros() - start) < us)
        ; // do nothing
#else
    RadioLibTime_t corrected = us * 1000 / (1000 + RADIOLIB_CLOCK_DRIFT_MS);
    RadioLibTime_t start = micros();
    while ((micros() - start) < corrected)
        ; // do nothing
#endif
}


RadioLibTime_t stm32RadioLibHal::millis() {
#if !defined(RADIOLIB_CLOCK_DRIFT_MS)
    return HAL_GetTick();
#else
    // Same correction RadioLib uses on Arduino
    return HAL_GetTick() * 1000 / (1000 + RADIOLIB_CLOCK_DRIFT_MS);
#endif
}



RadioLibTime_t stm32RadioLibHal::micros() {
#if !defined(RADIOLIB_CLOCK_DRIFT_MS)
    return __HAL_TIM_GET_COUNTER(&htim5); // hay que añadir header del hal para esto?
#else
    return __HAL_TIM_GET_COUNTER(&htim5) * 1000 / (1000 + RADIOLIB_CLOCK_DRIFT_MS);
#endif
}


long stm32RadioLibHal::pulseIn(uint32_t pin, uint32_t state, RadioLibTime_t timeout) {
    GPIO_TypeDef* port = getPort(pin);
    uint16_t pinMask = getPinMask(pin);

    uint32_t startMicros = micros();
    uint32_t timeoutMicros = startMicros + timeout

    uint8_t targetState = (state ? GPIO_PIN_SET : GPIO_PIN_RESET);

    // esperamos a que el pin salga de target state
    while (HAL_GPIO_ReadPin(port, pinMask) == targetState) {
        if (micros() > timeoutMicros) return 0;
        yield(); // allow FreeRTOS to run other tasks
    }

    // esperamos a que el pin vuelva a entrar en target state
    while (HAL_GPIO_ReadPin(port, pinMask) != targetState) {
        if (micros() > timeoutMicros) return 0;
        yield();
    }

    // medimos el tiempo que el pin se mantiene en target state
    uint32_t pulseStart = micros();
    while (HAL_GPIO_ReadPin(port, pinMask) == targetState) {
        if (micros() > timeoutMicros) return 0;
        yield();
    }
    return micros() - pulseStart;
}

void stm32RadioLibHal::spiBegin() {
    // No need for this in stm32. Init SPI at boot!!
}

void stm32RadioLibHal::spiBeginTransaction() {
    // No need for beginning transaction in stm32
}

void stm32RadioLibHal::spiTransfer(uint8_t* out, size_t len, uint8_t* in) { 
    for(size_t i = 0; i < len; i++) {  // for loop que imita Arduino HAL SPI transfer
        // ttransmitimos i recibimos 1 byte
        if(HAL_SPI_TransmitReceive(_spi, &out[i], &in[i], 1, HAL_MAX_DELAY) != HAL_OK) {
            in[i] = 0xFF;  // valor de error default
        }
    }
}

void stm32RadioLibHal::spiEndTransaction() {
    // No need for ending transaction in stm32
}

void stm32RadioLibHal::spiEnd() {
    // No need for ending spi for stm32. This is arduino specific
}

void stm32RadioLibHal::init() {
    // No need to initialize anything
}
void stm32RadioLibHal::term() {
    // No implementation needed for STM32
}

void stm32RadioLibHal::tone(uint32_t pin, unsigned int frequency, RadioLibTime_t duration) {

    if (pin == RADIOLIB_NC || frequency == 0) { //To-do: might have to protect frequency values a bit more. > than a certain value?
        return;
    }

    GPIO_TypeDef* port = getPort(pin);
    uint16_t pinMask = getPinMask(pin);

    // solo se puede usar PA0, PA5 o PA15 para salida de timer2 channel1
    if (!(port == GPIOA &&
         (pinMask == GPIO_PIN_0 ||
          pinMask == GPIO_PIN_5 ||
          pinMask == GPIO_PIN_15))) {
        return;  
    }

    enablePortClock(port);

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = pinMask;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(port, &GPIO_InitStruct);

    uint32_t timerClock = 80000000; // To-do: 80 MHz, sys clock is set to 80 MHz at the moment (nominal state). This will have to be changed when operational mode changes are implemented
    uint32_t arr = (timerClock / frequency) - 1;

    __HAL_TIM_SET_PRESCALER(&htim2, 0);
    __HAL_TIM_SET_AUTORELOAD(&htim2, arr);

    // arduino's tone uses 50% duty cycle
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, arr / 2);

    // this reloads parameter values immediately
    HAL_TIM_GenerateEvent(&htim2, TIM_EVENTSOURCE_UPDATE);

    // Start PWM output
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

    // Optional blocking with duration
    if (duration > 0) {
        delay(duration);
        noTone(pin);
    }
}


void stm32RadioLibHal::noTone(uint32_t pin) {
    if (pin == RADIOLIB_NC) {
        return;
    }
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);

    GPIO_TypeDef* port = getPort(pin);
    uint16_t pinMask = getPinMask(pin);

    // reseteamos pin como un ouput normal (para que device no lea floating values) 
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = pinMask;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(port, &GPIO_InitStruct);

    HAL_GPIO_WritePin(port, pinMask, GPIO_PIN_RESET);
}

void stm32RadioLibHal::yield() {
    // Allow task switching if scheduler is active
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        if (xPortIsInsideInterrupt()) {
            portYIELD_FROM_ISR(pdTRUE);
        } else {
            taskYIELD();
        }
    }
}

uint32_t stm32RadioLibHal::pinToInterrupt(uint32_t pin) {
    // trataremos los interruptpins como si fueran un pin normal, mediante codificación port+pin
    return pin;
}   


// ----- private functions ------
int stm32RadioLibHal::getExtiLineFromPinMask(uint16_t pinMask) {
    for (int line = 0; line < 16; ++line) {
        if (pinMask & (1U << line)) {
            return line;
        }
    }
    return -1; // invalid or multiple bits set
}


void stm32RadioLibHal::handleExtiCallback(uint16_t gpioPin) {
    int line = getExtiLineFromPinMask(gpioPin);
    if (line < 0 || line > 15) {
        return;
    }

    void (*cb)(void) = _extiCallbacks[line];
    if (cb != nullptr) {
        cb();
    }
}