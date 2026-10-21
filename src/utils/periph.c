/**
 * @file periph.c
 * @brief Peripheral handle definitions and initialization helpers.
 * @details
 * Defines the global STM32 HAL peripheral handles declared in periph.h and
 * centralizes board peripheral initialization. The module initializes GPIO,
 * TIM5, TIM2, SPI2, IWDG, UART4, RTC, ADC1, I2C3 and ADC2, and provides
 * reconfiguration for peripherals whose timing depends on the selected
 * system clock.
 * @author Guillermo O'Tuama Pascual
 * @date 2026-01-20
 */

#include "periph.h"

// TODO
/* Provided by main.c and stm32l4xx_hal_msp.c. */
void Error_Handler(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;
SPI_HandleTypeDef hspi2;
UART_HandleTypeDef huart4;
IWDG_HandleTypeDef hiwdg;
RTC_HandleTypeDef hrtc;
ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c3;
ADC_HandleTypeDef hadc2;

static uint32_t tim5_prescaler_for_freq(ClockFreq_t freq);
static uint32_t spi2_prescaler_for_freq(ClockFreq_t freq);
static uint32_t adc1_prescaler_for_freq(ClockFreq_t freq);
static uint32_t i2c3_timing_for_freq(ClockFreq_t freq);
static void periph_gpio_init(void);
static void periph_tim2_init(void);
static void periph_tim5_init(ClockFreq_t freq);
static void periph_spi2_init(ClockFreq_t freq);
static void periph_iwdg_init(void);
static void periph_uart4_init(void);
static void periph_rtc_init(void);
static void periph_adc1_init(ClockFreq_t freq);
static void periph_i2c3_init(ClockFreq_t freq);
static void periph_adc2_init(ClockFreq_t freq);

void periph_init_for_freq(ClockFreq_t freq)
{
    periph_gpio_init();
    periph_tim5_init(freq);
    periph_tim2_init();
    periph_spi2_init(freq);
    periph_iwdg_init();
    periph_uart4_init();
    periph_rtc_init();
    periph_adc1_init(freq);
    periph_i2c3_init(freq);
    periph_adc2_init(freq);
}

void periph_reconfigure_for_freq(ClockFreq_t freq)
{
    __HAL_TIM_SET_PRESCALER(&htim5, tim5_prescaler_for_freq(freq));
    HAL_TIM_GenerateEvent(&htim5, TIM_EVENTSOURCE_UPDATE);

    if (HAL_UART_Init(&huart4) != HAL_OK) {
        Error_Handler();
    }

    HAL_SPI_DeInit(&hspi2);
    hspi2.Init.BaudRatePrescaler = spi2_prescaler_for_freq(freq);
    if (HAL_SPI_Init(&hspi2) != HAL_OK) {
        Error_Handler();
    }

    /* ADC1 and ADC2 share the ADC clock and the common prescaler: take both
     * down before bringing either back up. */
    HAL_ADC_DeInit(&hadc2);
    HAL_ADC_DeInit(&hadc1);
    hadc1.Init.ClockPrescaler = adc1_prescaler_for_freq(freq);
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) {
        Error_Handler();
    }

    hadc2.Init.ClockPrescaler = adc1_prescaler_for_freq(freq);
    if (HAL_ADC_Init(&hadc2) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) {
        Error_Handler();
    }

    HAL_I2C_DeInit(&hi2c3);
    hi2c3.Init.Timing = i2c3_timing_for_freq(freq);
    if (HAL_I2C_Init(&hi2c3) != HAL_OK) {
        Error_Handler();
    }
}

void periph_delay_us(uint32_t us)
{
    uint32_t start = __HAL_TIM_GET_COUNTER(&htim5);

    while ((__HAL_TIM_GET_COUNTER(&htim5) - start) < us) {
    }
}

/**
 * @brief Select the TIM5 prescaler for the configured system clock.
 *
 * TIM5 is used as a microsecond timebase, so the prescaler is chosen to keep
 * the timer counter running at 1 MHz for each supported system clock.
 *
 * @param freq System clock selection.
 * @return TIM5 prescaler value.
 */
static uint32_t tim5_prescaler_for_freq(ClockFreq_t freq)
{
    switch (freq) {
        case CLK_FREQ_80MHZ: return 79u; /* 80 MHz / (79 + 1) = 1 MHz */
        case CLK_FREQ_8MHZ:  return 7u;  /* 8 MHz / (7 + 1) = 1 MHz */
        case CLK_FREQ_2MHZ:  return 1u;  /* 2 MHz / (1 + 1) = 1 MHz */
        default:             return 79u;
    }
}

/**
 * @brief Select the SPI2 baud-rate prescaler for the configured system clock.
 *
 * @param freq System clock selection.
 * @return SPI2 baud-rate prescaler value.
 */
static uint32_t spi2_prescaler_for_freq(ClockFreq_t freq)
{
    switch (freq) {
        case CLK_FREQ_80MHZ: return SPI_BAUDRATEPRESCALER_8; /* 10 MHz */
        case CLK_FREQ_8MHZ:  return SPI_BAUDRATEPRESCALER_2; /* 4 MHz  */
        case CLK_FREQ_2MHZ:  return SPI_BAUDRATEPRESCALER_2; /* 1 MHz  */
        default:             return SPI_BAUDRATEPRESCALER_8;
    }
}

/**
 * @brief Select the ADC1 clock prescaler for the configured system clock.
 *
 * @param freq System clock selection.
 * @return ADC1 clock prescaler value.
 */
static uint32_t adc1_prescaler_for_freq(ClockFreq_t freq)
{
    switch (freq) {
        case CLK_FREQ_80MHZ: return ADC_CLOCK_SYNC_PCLK_DIV4; /* 20 MHz */
        case CLK_FREQ_8MHZ:  return ADC_CLOCK_SYNC_PCLK_DIV1; /* 8 MHz  */
        case CLK_FREQ_2MHZ:  return ADC_CLOCK_SYNC_PCLK_DIV1; /* 2 MHz  */
        default:             return ADC_CLOCK_SYNC_PCLK_DIV4;
    }
}

/**
 * @brief Select the I2C3 TIMINGR value for the configured system clock.
 *
 * I2C3 is clocked from PCLK1, which equals SYSCLK in every clock
 * configuration. Fast mode (400 kHz) where the clock allows it, standard
 * mode (≈100 kHz) at 2 MHz. Values from RM0351 §37.4.10 (8 MHz table) and
 * CubeMX (80 MHz); the 2 MHz value is derived with the same constraints
 * (PRESC = 0, SCLL = 10, SCLH = 8, SDADEL = 0, SCLDEL = 3 clock periods).
 *
 * @param freq System clock selection.
 * @return I2C3 TIMINGR value.
 */
static uint32_t i2c3_timing_for_freq(ClockFreq_t freq)
{
    switch (freq) {
        case CLK_FREQ_80MHZ: return 0x00702991u; /* 400 kHz */
        case CLK_FREQ_8MHZ:  return 0x00310309u; /* 400 kHz */
        case CLK_FREQ_2MHZ:  return 0x00200709u; /* ~100 kHz */
        default:             return 0x00702991u;
    }
}

/**
  * @brief GPIO Initialization Function
  * @details Enables the GPIO port clocks required by board peripherals and
  * configures the ADCS board control outputs (magnetorquer drivers asleep,
  * sun-sensor mux on channel S1).
  * Radio control and interrupt pins are configured later by the RadioLib HAL.
  */
static void periph_gpio_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* Drive the outputs low before enabling them: DRV8214 nSLEEP stays low */
    HAL_GPIO_WritePin(ADCS_MTQ_EN_PORT, ADCS_MTQ_EN_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ADCS_SEL_PH0_PORT, ADCS_SEL_PH0_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ADCS_SEL_PH1_PORT, ADCS_SEL_PH1_PIN | ADCS_SEL_PH2_PIN, GPIO_PIN_RESET);

    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pin = ADCS_MTQ_EN_PIN | ADCS_SEL_PH1_PIN | ADCS_SEL_PH2_PIN; /* all on GPIOA */
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = ADCS_SEL_PH0_PIN;
    HAL_GPIO_Init(ADCS_SEL_PH0_PORT, &gpio);

     /* Radio GPIO pins (PC9/NRST, PA8/BUSY, PA10/DIO1, PB12/NSS) are
     configured by RadioLib through the HAL — do not touch them here. */
}

/**
  * @brief TIM2 Initialization Function
  * @details Configured for PWM generation, used for tone generation in stm32_radiolib_hal.cpp.
  * @note It will probably not be needed for SX1262 operation but it was added for completeness.
  * If not used, it should be removed.
  * @todo Remove if not used.
  */
static void periph_tim2_init(void)
{
    TIM_MasterConfigTypeDef master_config = {0};
    TIM_OC_InitTypeDef output_compare_config = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0xFFFFFFFFu;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }

    master_config.MasterOutputTrigger = TIM_TRGO_RESET;
    master_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &master_config) != HAL_OK) {
        Error_Handler();
    }

    output_compare_config.OCMode = TIM_OCMODE_PWM1;
    output_compare_config.Pulse = 0;
    output_compare_config.OCPolarity = TIM_OCPOLARITY_HIGH;
    output_compare_config.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &output_compare_config, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }

    HAL_TIM_MspPostInit(&htim2);
}

/**
  * @brief TIM5 Initialization Function
  * @details TIM5 is used as a microsecond timebase. The prescaler is selected
  * from the current system clock configuration so the timer counter runs at
  * 1 MHz for each supported clock frequency.
  * @param freq Current system clock selection.
  */
static void periph_tim5_init(ClockFreq_t freq)
{
    TIM_ClockConfigTypeDef clock_source_config = {0};
    TIM_MasterConfigTypeDef master_config = {0};

    htim5.Instance = TIM5;
    htim5.Init.Prescaler = tim5_prescaler_for_freq(freq);
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = 0xFFFFFFFFu;
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim5) != HAL_OK) {
        Error_Handler();
    }

    clock_source_config.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim5, &clock_source_config) != HAL_OK) {
        Error_Handler();
    }

    master_config.MasterOutputTrigger = TIM_TRGO_RESET;
    master_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &master_config) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_TIM_Base_Start(&htim5) != HAL_OK) {
        Error_Handler();
    }
}

/**
* @brief SPI2 Initialization Function
* @details Initializes SPI2 as an SPI master (full-duplex) for communication with the SX1262.
*  - 8-bit frames
*  - Most significant bit (MSB) first
*  - Clock polarity low, clock phase 1st edge (SPI mode 0)
*  - Software NSS management
*  - NSS pulse mode disabled
*  - Baud rate prescaler selected from the current system clock
* @param freq Current system clock selection.
*/
static void periph_spi2_init(ClockFreq_t freq)
{
    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_MASTER;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi2.Init.NSS = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = spi2_prescaler_for_freq(freq);
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial = 7;
    hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    if (HAL_SPI_Init(&hspi2) != HAL_OK) {
        Error_Handler();
    }
}


/**
  * @brief IWDG Initialization Function
  * @details The watchdog is currently set to the maximum prescaler and reload value for debugging purposes.
  * This results in a countdown period of approximately 30-35 seconds.
  */
static void periph_iwdg_init(void)
{
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
    hiwdg.Init.Window = 4095;
    hiwdg.Init.Reload = 4095;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief UART4 Initialization Function
  * @details UART4 is configured for debug output using printf() at 115200 baud.
  */
static void periph_uart4_init(void)
{
    huart4.Instance = UART4;
    huart4.Init.BaudRate = 115200;
    huart4.Init.WordLength = UART_WORDLENGTH_8B;
    huart4.Init.StopBits = UART_STOPBITS_1;
    huart4.Init.Parity = UART_PARITY_NONE;
    huart4.Init.Mode = UART_MODE_TX_RX;
    huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart4.Init.OverSampling = UART_OVERSAMPLING_16;
    huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart4) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief RTC Initialization Function
  * @details Initializes the RTC in 24-hour mode and preserves the stored
  * date/time across resets when the backup register flag is already set.
  * If the flag is missing, default time/date fields are written and the backup
  * flag is set.
  */
static void periph_rtc_init(void)
{
    RTC_TimeTypeDef time = {0};
    RTC_DateTypeDef date = {0};

    hrtc.Instance = RTC;
    hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
    hrtc.Init.AsynchPrediv = 127;
    hrtc.Init.SynchPrediv = 255;
    hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
    hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
    hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
    if (HAL_RTC_Init(&hrtc) != HAL_OK) {
        Error_Handler();
    }

    /* Preserve RTC date/time across resets once the backup flag is set. */
    if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) != 0x32F2u) {
        time.Hours = 0x0;
        time.Minutes = 0x0;
        time.Seconds = 0x0;
        time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
        time.StoreOperation = RTC_STOREOPERATION_RESET;
        if (HAL_RTC_SetTime(&hrtc, &time, RTC_FORMAT_BCD) != HAL_OK) {
            Error_Handler();
        }

        date.WeekDay = RTC_WEEKDAY_MONDAY;
        date.Month = RTC_MONTH_JANUARY;
        date.Date = 0x1;
        date.Year = 0x0;
        if (HAL_RTC_SetDate(&hrtc, &date, RTC_FORMAT_BCD) != HAL_OK) {
            Error_Handler();
        }

        HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, 0x32F2u);
    }
}

/**
  * @brief ADC1 Initialization Function
  * @details Configures ADC1 for single software-triggered conversions used by
  * the internal temperature-sensor readout. The ADC clock prescaler is selected
  * from the current system clock configuration, and the channel is configured
  * by the code that performs each conversion.
  * - 12-bit resolution
  * - Single conversion mode
  * - Software trigger
  * - Calibration started in single-ended mode
  * @param freq Current system clock selection.
  */
static void periph_adc1_init(ClockFreq_t freq)
{
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = adc1_prescaler_for_freq(freq);
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    hadc1.Init.OversamplingMode = DISABLE;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief I2C3 Initialization Function
  * @details I2C3 (PC0 = SCL, PC1 = SDA) is the ADCS bus: MMC5983MA,
  * IIM-42652, the DRV8214 magnetorquer drivers and the face-board TCN75A
  * sensors. 7-bit addressing, analog filter on, no digital filter; the
  * 4.7 kΩ pull-ups are on the OBC board.
  * @param freq Current system clock selection.
  */
static void periph_i2c3_init(ClockFreq_t freq)
{
    hi2c3.Instance = I2C3;
    hi2c3.Init.Timing = i2c3_timing_for_freq(freq);
    hi2c3.Init.OwnAddress1 = 0;
    hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c3.Init.OwnAddress2 = 0;
    hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c3) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief ADC2 Initialization Function
  * @details ADC2 samples the ADCS sun-sensor mux output ADC_PH (PC5,
  * ADC12_IN14). It is kept separate from ADC1 so the sun sensors never
  * contend with the MCU temperature readout. Same configuration as ADC1;
  * the clock prescaler is common to both.
  * @param freq Current system clock selection.
  */
static void periph_adc2_init(ClockFreq_t freq)
{
    hadc2.Instance = ADC2;
    hadc2.Init.ClockPrescaler = adc1_prescaler_for_freq(freq);
    hadc2.Init.Resolution = ADC_RESOLUTION_12B;
    hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc2.Init.LowPowerAutoWait = DISABLE;
    hadc2.Init.ContinuousConvMode = DISABLE;
    hadc2.Init.NbrOfConversion = 1;
    hadc2.Init.DiscontinuousConvMode = DISABLE;
    hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc2.Init.DMAContinuousRequests = DISABLE;
    hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    hadc2.Init.OversamplingMode = DISABLE;
    if (HAL_ADC_Init(&hadc2) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) {
        Error_Handler();
    }
}
