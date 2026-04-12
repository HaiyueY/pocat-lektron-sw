/**
 * @file clock.c
 * @brief Dynamic system clock frequency switching for power management.
 *
 * Switches between 80 MHz (HSI+PLL), 8 MHz (MSI Range 7), and 2 MHz
 * (MSI Range 5) based on OBC state. Reconfigures TIM5, UART2, SPI2,
 * and ADC1 after each switch.
 */

#include "clock.h"
#include "stm32l4xx_hal.h"
#include "periph.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

static ClockFreq_t current_freq = CLK_FREQ_80MHZ;

static ClockFreq_t state_to_freq(ObcState_t state);
static bool switch_to_80mhz(void);
static bool switch_to_msi(uint32_t msi_range, uint32_t flash_latency);
static void reconfigure_peripherals(ClockFreq_t freq);


bool clock_switch_for_state(ObcState_t state)
{
    ClockFreq_t target = state_to_freq(state);

    if (target == current_freq) {
        return true;
    }

    bool ok;
    
    // revisar esta parte....
    vTaskSuspendAll();

    if (target == CLK_FREQ_80MHZ) {
        ok = switch_to_80mhz();
    } else {
        uint32_t msi_range = (target == CLK_FREQ_8MHZ)
            ? RCC_MSIRANGE_7   // 8 MHz 
            : RCC_MSIRANGE_5;  // 2 MHz
        uint32_t latency = (target == CLK_FREQ_8MHZ)
            ? FLASH_LATENCY_1 // 8MHz at flash latency 1
            : FLASH_LATENCY_0;  // 2 MHz
        ok = switch_to_msi(msi_range, latency);
    }

    if (ok) {
        reconfigure_peripherals(target);
        current_freq = target;
    }

    // revisar esta parte....
    xTaskResumeAll();
    return ok;
}


static ClockFreq_t state_to_freq(ObcState_t state)
{
    switch (state) {
        case SUNSAFE:   return CLK_FREQ_8MHZ;
        case SURVIVAL:  return CLK_FREQ_2MHZ;
        default:        return CLK_FREQ_80MHZ;  // NOMINAL, CONTINGENCY
    }
}


static bool switch_to_80mhz(void)
{
    /* Step 1: Exit Low-Power Run mode if active (required before raising voltage/frequency) */
    if (__HAL_PWR_GET_FLAG(PWR_FLAG_REGLPF)) {
        if (HAL_PWREx_DisableLowPowerRunMode() != HAL_OK) {
            return false;
        }
    }

    /* Step 2: Voltage scaling to Range 1 BEFORE increasing frequency */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
        return false;
    }

    /* Step 3: Enable HSI and PLL */
    RCC_OscInitTypeDef osc = {0};
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    osc.PLL.PLLM = 1;
    osc.PLL.PLLN = 10;
    osc.PLL.PLLP = RCC_PLLP_DIV7;
    osc.PLL.PLLQ = RCC_PLLQ_DIV2;
    osc.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        return false;
    }

    /* Step 4: Switch SYSCLK to PLL */
    RCC_ClkInitTypeDef clk = {0};
    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_4) != HAL_OK) {
        return false;
    }

    /* Step 5: Optionally disable MSI to save power */
    RCC_OscInitTypeDef msi_off = {0};
    msi_off.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    msi_off.MSIState = RCC_MSI_OFF;
    msi_off.PLL.PLLState = RCC_PLL_NONE;
    HAL_RCC_OscConfig(&msi_off);  /* Non-critical if it fails */

    return true;
}


static bool switch_to_msi(uint32_t msi_range, uint32_t flash_latency)
{
    /* Step 1: Exit Low-Power Run mode if active (e.g. switching from 2 MHz to 8 MHz) */
    if (__HAL_PWR_GET_FLAG(PWR_FLAG_REGLPF)) {
        if (HAL_PWREx_DisableLowPowerRunMode() != HAL_OK) {
            return false;
        }
    }

    /* Step 2: Enable MSI at target range */
    RCC_OscInitTypeDef osc = {0};
    osc.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    osc.MSIState = RCC_MSI_ON;
    osc.MSIClockRange = msi_range;
    osc.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
    osc.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        return false;
    }

    /* Step 3: Switch SYSCLK to MSI */
    RCC_ClkInitTypeDef clk = {0};
    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, flash_latency) != HAL_OK) {
        return false;
    }

    // Disable PLL and HSI at the same time with HAL_RCC_OscConfig. PLL might have to be disabled before HSI but we'll leave it like this for now
    RCC_OscInitTypeDef pll_off = {0};
    pll_off.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    pll_off.HSIState = RCC_HSI_OFF; // HSI clock deactivation
    pll_off.PLL.PLLState = RCC_PLL_OFF; // PLL deactivation
    HAL_RCC_OscConfig(&pll_off);  /* Non-critical if it fails */

    /* Step 5: Voltage scaling to Range 2 AFTER decreasing frequency */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) != HAL_OK) {
        return false;
    }

    /* Step 6: Enter Low-Power Run mode at 2 MHz for maximum power savings.
     * At 8 MHz we stay in normal Run mode (LP Run is only valid up to 2 MHz). */
    if (msi_range == RCC_MSIRANGE_5) {
        HAL_PWREx_EnableLowPowerRunMode();
    }

    return true;
}

// todo: error handling 
static void reconfigure_peripherals(ClockFreq_t freq)
{
    //TIM5
    uint32_t tim5_psc;
    switch (freq) {
        case CLK_FREQ_80MHZ: tim5_psc = 79; break;  // 80 MHz / (79 + 1) = 1 MHz
        case CLK_FREQ_8MHZ:  tim5_psc = 7;  break;  // 8 MHz / (7 + 1) = 1 MHz
        case CLK_FREQ_2MHZ:  tim5_psc = 1;  break;  // 2 MHz / (1 + 1) = 1 MHz
        default:             tim5_psc = 79;  break;
    }
    __HAL_TIM_SET_PRESCALER(&htim5, tim5_psc);
    HAL_TIM_GenerateEvent(&htim5, TIM_EVENTSOURCE_UPDATE); // Apply new prescaler immediately by generating an update event

    // UART2: re-init recalculates BRR from current PCLK1
    HAL_UART_Init(&huart2);

    // SPI2
    uint32_t spi_psc;
    switch (freq) {
        case CLK_FREQ_80MHZ: spi_psc = SPI_BAUDRATEPRESCALER_8;  break;  /* 10 MHz */
        case CLK_FREQ_8MHZ:  spi_psc = SPI_BAUDRATEPRESCALER_2;  break;  /* 4 MHz  */
        case CLK_FREQ_2MHZ:  spi_psc = SPI_BAUDRATEPRESCALER_2;  break;  /* 1 MHz  */
        default:             spi_psc = SPI_BAUDRATEPRESCALER_8;  break;
    }
    HAL_SPI_DeInit(&hspi2);
    hspi2.Init.BaudRatePrescaler = spi_psc;
    HAL_SPI_Init(&hspi2);

    // ADC1
    uint32_t adc_psc;
    switch (freq) {
        case CLK_FREQ_80MHZ: adc_psc = ADC_CLOCK_SYNC_PCLK_DIV4; break;  /* 20 MHz */
        case CLK_FREQ_8MHZ:  adc_psc = ADC_CLOCK_SYNC_PCLK_DIV1; break;  /* 8 MHz  */
        case CLK_FREQ_2MHZ:  adc_psc = ADC_CLOCK_SYNC_PCLK_DIV1; break;  /* 2 MHz  */
        default:             adc_psc = ADC_CLOCK_SYNC_PCLK_DIV4; break;
    }
    HAL_ADC_DeInit(&hadc1);
    hadc1.Init.ClockPrescaler = adc_psc;
    HAL_ADC_Init(&hadc1);
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
}
