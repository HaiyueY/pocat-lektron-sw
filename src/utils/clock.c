/**
 * @file clock.c
 * @brief Dynamic system clock frequency switching for power management.
 *
 * Switches between 80 MHz (HSI+PLL), 8 MHz (MSI Range 7), and 2 MHz
 * (MSI Range 5) based on OBC state. After runtime switches, peripherals that
 * derive timings from the system clock are reconfigured through periph.c.
 */

#include "clock.h"
#include "periph.h"
#include "stm32l4xx_hal.h"
#include <stdio.h>

static ClockFreq_t current_freq = CLK_FREQ_80MHZ;
static bool clock_initialized = false;

static ClockFreq_t state_to_freq(ObcState_t state);
static bool systemclock_config_for_freq(ClockFreq_t target);
static bool switch_to_hsi(void);
static bool switch_to_msi(uint32_t msi_range, uint32_t flash_latency);
static bool systemclock_config_hsi(void);
static bool systemclock_config_msi(uint32_t msi_range, uint32_t flash_latency);

bool systemclock_init_for_state(ObcState_t state)
{
    ClockFreq_t target = state_to_freq(state);
    bool ok;

    if (clock_initialized && target == current_freq) {
        return true;
    }

    ok = systemclock_config_for_freq(target);

    if (!ok) {
        printf("System clock config failed\r\n");
        return false;
    }

    current_freq = target;
    clock_initialized = true;
    return true;
}

bool clock_switch_for_state(ObcState_t state)
{
    ClockFreq_t target = state_to_freq(state);
    bool ok;

    if (!clock_initialized) {
        return false;
    }

    if (target == current_freq) {
        return true;
    }

    if (target == CLK_FREQ_80MHZ) {
        ok = switch_to_hsi();
    } else {
        uint32_t msi_range = (target == CLK_FREQ_8MHZ) ? RCC_MSIRANGE_7 : RCC_MSIRANGE_5;
        uint32_t latency = (target == CLK_FREQ_8MHZ) ? FLASH_LATENCY_1 : FLASH_LATENCY_0;
        ok = switch_to_msi(msi_range, latency);
    }

    if (!ok) {
        printf("Clock switch failed\r\n");
        return false;
    }

    periph_reconfigure_for_freq(target);
    current_freq = target;
    return true;
}

ClockFreq_t clock_get_current(void)
{
    return current_freq;
}

static ClockFreq_t state_to_freq(ObcState_t state)
{
    switch (state) {
        case SUNSAFE:  return CLK_FREQ_8MHZ;
        case SURVIVAL: return CLK_FREQ_2MHZ;
        default:       return CLK_FREQ_80MHZ;
    }
}

static bool systemclock_config_for_freq(ClockFreq_t target)
{
    if (target == CLK_FREQ_80MHZ) {
        return systemclock_config_hsi();
    }

    uint32_t msi_range = (target == CLK_FREQ_8MHZ) ? RCC_MSIRANGE_7 : RCC_MSIRANGE_5;
    uint32_t latency = (target == CLK_FREQ_8MHZ) ? FLASH_LATENCY_1 : FLASH_LATENCY_0;
    return systemclock_config_msi(msi_range, latency);
}

static bool switch_to_hsi(void)
{
    // Step 1: Exit Low-Power Run mode if active (required before raising voltage/frequency)
    if (__HAL_PWR_GET_FLAG(PWR_FLAG_REGLPF)) {
        if (HAL_PWREx_DisableLowPowerRunMode() != HAL_OK) {
            return false;
        }
    }

    // Step 2: Voltage scaling to Range 1 BEFORE increasing frequency
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
        return false;
    }

    // Step 3: Enable HSI and PLL
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

    // Step 4: Switch SYSCLK to PLL
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

    // Step 5: Optionally disable MSI to save power
    RCC_OscInitTypeDef msi_off = {0};
    msi_off.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    msi_off.MSIState = RCC_MSI_OFF;
    msi_off.PLL.PLLState = RCC_PLL_NONE;
    HAL_RCC_OscConfig(&msi_off);

    return true;
}

static bool switch_to_msi(uint32_t msi_range, uint32_t flash_latency)
{
    // Step 1: Exit Low-Power Run mode if active (e.g. switching from 2 MHz to 8 MHz)
    if (__HAL_PWR_GET_FLAG(PWR_FLAG_REGLPF)) {
        if (HAL_PWREx_DisableLowPowerRunMode() != HAL_OK) {
            return false;
        }
    }

    // Step 2: Enable MSI at target range
    RCC_OscInitTypeDef osc = {0};
    osc.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    osc.MSIState = RCC_MSI_ON;
    osc.MSIClockRange = msi_range;
    osc.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
    osc.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        return false;
    }

    // Step 3: Switch SYSCLK to MSI
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
    HAL_RCC_OscConfig(&pll_off);

    // Step 5: Voltage scaling to Range 2 AFTER decreasing frequency
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) != HAL_OK) {
        return false;
    }

    /* Step 6: Enter Low-Power run mode at 2 MHz for maximum power savings.
     * At 8 MHz we stay in normal Run mode (Low-Power run mode is only valid up to 2 MHz). */
    if (msi_range == RCC_MSIRANGE_5) {
        HAL_PWREx_EnableLowPowerRunMode();
    }

    return true;
}

// only called after restarting satellite
static bool systemclock_config_hsi(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    // Configure the main internal regulator output voltage
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
        return false;
    }

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.LSIState = RCC_LSI_ON;
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

    // Initializes the CPU, AHB and APB buses clocks 
    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_4) != HAL_OK) {
        return false;
    }

    return true;
}

// only called after restarting satellite
static bool systemclock_config_msi(uint32_t msi_range, uint32_t flash_latency)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    // Configure the main internal regulator output voltage
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) != HAL_OK) {
        return false;
    }

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    osc.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_MSI;
    osc.LSIState = RCC_LSI_ON;
    osc.MSIState = RCC_MSI_ON;
    osc.MSICalibrationValue = 0;
    osc.MSIClockRange = msi_range;
    osc.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        return false;
    }

    // Initializes the CPU, AHB and APB buses clocks
    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, flash_latency) != HAL_OK) {
        return false;
    }

    // enter low-power run mode
    if (msi_range == RCC_MSIRANGE_5) {
        HAL_PWREx_EnableLowPowerRunMode();
    }

    return true;
}
