#include "board.h"

#include <math.h>

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi2_tx;
UART_HandleTypeDef hsbus_uart;
#if BOARD_HAS_BATTERY_VOLTAGE
static ADC_HandleTypeDef hadc_battery;
#endif

#define DFU_REQUEST_ADDRESS 0x2003FFF0U
#define DFU_REQUEST_MAGIC 0x44554634U
#define SYSTEM_MEMORY_ADDRESS 0x1FF00000U
#define STATUS_LED_PATTERN_PERIOD_US 1000000U
#define STATUS_LED_FLASH_US 80000U
#define STATUS_LED_SECOND_FLASH_US 160000U
#define STATUS_LED_CALIBRATION_TOGGLE_US 100000U
#define BUZZER_PATTERN_PERIOD_US 400000U
#define BUZZER_BEEP_US 100000U
#define BUZZER_SECOND_BEEP_US 150000U

static void jump_to_system_bootloader(void) __attribute__((noreturn));

#include "foxeer_uart.h"

static void jump_to_system_bootloader(void)
{
    const uint32_t boot_stack =
        *(volatile uint32_t *)SYSTEM_MEMORY_ADDRESS;
    const uint32_t boot_entry =
        *(volatile uint32_t *)(SYSTEM_MEMORY_ADDRESS + 4U);
    void (*const bootloader)(void) = (void (*)(void))boot_entry;

    /*
     * This runs immediately after a real MCU reset. Keep PRIMASK clear:
     * the ROM USB DFU bootloader needs interrupts during enumeration.
     * Keep the interrupt state compatible with the STM32F7 ROM bootloader.
     */
    SCB->VTOR = SYSTEM_MEMORY_ADDRESS;
    __DSB();
    __ISB();
    __set_MSP(boot_stack);
    bootloader();
    while (1) {
    }
}

static void clock_init(void)
{
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    RCC_OscInitTypeDef osc = {0};
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    /* 8 MHz / 8 * 432 / 2 = 216 MHz; PLLQ / 9 = exact 48 MHz USB. */
    osc.PLL.PLLM = 8U;
    osc.PLL.PLLN = 432U;
    osc.PLL.PLLP = RCC_PLLP_DIV2;
    osc.PLL.PLLQ = 9U;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) board_fatal_error();
    if (HAL_PWREx_EnableOverDrive() != HAL_OK) board_fatal_error();
    RCC_ClkInitTypeDef clk = {0};
    clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV4;
    clk.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_7) != HAL_OK) board_fatal_error();
    RCC_PeriphCLKInitTypeDef periph = {0};
    periph.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
    periph.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&periph) != HAL_OK) board_fatal_error();
}

static void gpio_init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    /* Keep every ESC signal low before changing the pins to timer outputs. */
    HAL_GPIO_WritePin(MOTOR_1_PORT, MOTOR_1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_2_PORT, MOTOR_2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_3_PORT, MOTOR_3_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_4_PORT, MOTOR_4_PIN, GPIO_PIN_RESET);
    gpio.Pin = MOTOR_1_PIN;
    HAL_GPIO_Init(MOTOR_1_PORT, &gpio);
    gpio.Pin = MOTOR_2_PIN;
    HAL_GPIO_Init(MOTOR_2_PORT, &gpio);
    gpio.Pin = MOTOR_3_PIN;
    HAL_GPIO_Init(MOTOR_3_PORT, &gpio);
    gpio.Pin = MOTOR_4_PIN;
    HAL_GPIO_Init(MOTOR_4_PORT, &gpio);
    gpio.Pin = STATUS_LED_PIN;
    HAL_GPIO_Init(STATUS_LED_PORT, &gpio);
    board_status_led_set(false);
#if BOARD_HAS_SBUS_INVERTER_CONTROL
    gpio.Pin = SBUS_INVERTER_PIN;
    HAL_GPIO_Init(SBUS_INVERTER_PORT, &gpio);
    HAL_GPIO_WritePin(SBUS_INVERTER_PORT,
                      SBUS_INVERTER_PIN,
                      SBUS_INVERTER_ENABLE_LEVEL);
#endif
    gpio.Pin = IMU_CS_PIN;
    HAL_GPIO_Init(IMU_CS_PORT, &gpio);
    HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_SET);

    gpio.Pin = BUZZER_PIN;
#if BOARD_BUZZER_OUTPUT_OPEN_DRAIN
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_PULLUP;
#else
    /* Betaflight specifies a push-pull active-low beeper on F722 V4. */
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
#endif
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BUZZER_PORT, &gpio);
    board_buzzer_set(false);
}
static void spi1_init(void)
{
    __HAL_RCC_SPI1_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &gpio);

    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        board_fatal_error();
    }
}

static void storage_spi_init(void)
{
    __HAL_RCC_SPI2_CLK_ENABLE();
    __HAL_RCC_SPI3_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {.Pin=GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15,
        .Mode=GPIO_MODE_AF_PP, .Pull=GPIO_NOPULL,
        .Speed=GPIO_SPEED_FREQ_VERY_HIGH, .Alternate=GPIO_AF5_SPI2};
    HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin = DATAFLASH_CS_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_WritePin(DATAFLASH_CS_PORT, DATAFLASH_CS_PIN, GPIO_PIN_SET);
    HAL_GPIO_Init(DATAFLASH_CS_PORT, &gpio);
    hspi2.Instance = SPI2;
    hspi2.Init = hspi1.Init;
    hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    if (HAL_SPI_Init(&hspi2) != HAL_OK) board_fatal_error();

    /* SPI3 MOSI is PB5, not PC12: PC12 remains available for UART5. */
    gpio.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &gpio);
    gpio.Pin = GPIO_PIN_5;
    HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin = MAX7456_CS_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_WritePin(MAX7456_CS_PORT, MAX7456_CS_PIN, GPIO_PIN_SET);
    HAL_GPIO_Init(MAX7456_CS_PORT, &gpio);
    hspi3.Instance = SPI3;
    hspi3.Init = hspi1.Init;
    hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    if (HAL_SPI_Init(&hspi3) != HAL_OK) board_fatal_error();
}

static void battery_adc_init(void)
{
#if BOARD_HAS_BATTERY_VOLTAGE
    __HAL_RCC_ADC3_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = BATTERY_ADC_PIN;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BATTERY_ADC_PORT, &gpio);

    gpio.Pin = CURRENT_ADC_PIN;
    HAL_GPIO_Init(CURRENT_ADC_PORT, &gpio);
    hadc_battery.Instance = ADC3;
    hadc_battery.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc_battery.Init.Resolution = ADC_RESOLUTION_12B;
    hadc_battery.Init.ScanConvMode = DISABLE;
    hadc_battery.Init.ContinuousConvMode = DISABLE;
    hadc_battery.Init.DiscontinuousConvMode = DISABLE;
    hadc_battery.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc_battery.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc_battery.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc_battery.Init.NbrOfConversion = 1U;
    hadc_battery.Init.DMAContinuousRequests = DISABLE;
    hadc_battery.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc_battery) != HAL_OK) board_fatal_error();

    ADC_ChannelConfTypeDef channel = {0};
    channel.Channel = BATTERY_ADC_CHANNEL;
    channel.Rank = 1U;
    channel.SamplingTime = ADC_SAMPLETIME_84CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc_battery, &channel) != HAL_OK) {
        board_fatal_error();
    }
#endif
}

void board_init(void)
{
    SCB_EnableICache();
    /* SRAM DMA buffers and USB data remain coherent with D-cache disabled. */
    HAL_Init();
    clock_init();
    gpio_init();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void board_peripherals_init(void)
{
    spi1_init();
    storage_spi_init();
    if (!board_receiver_uart_configure(true, 1U)) board_fatal_error();
    battery_adc_init();
}

void board_imu_select(uint8_t candidate) { (void)candidate; }
GPIO_TypeDef *board_imu_cs_port(void) { return IMU_CS_PORT; }
uint16_t board_imu_cs_pin(void) { return IMU_CS_PIN; }

static uint32_t battery_adc_total;
static uint32_t battery_adc_next_sample_us;
static uint8_t battery_adc_samples;
static bool battery_adc_pending;
static uint32_t battery_adc_started_us;
static uint32_t battery_adc_errors;
static float battery_voltage_filtered;
static float battery_voltage_multiplier = 1.0f;
#if BOARD_HAS_CURRENT
static bool current_adc_pending;
static float battery_current_filtered;
static uint32_t current_adc_raw, current_adc_count, current_adc_last_us;
#endif

uint32_t board_micros(void)
{
    uint32_t tick_before, tick_after, systick_value;
    do {
        tick_before = HAL_GetTick();
        systick_value = SysTick->VAL;
        tick_after = HAL_GetTick();
    } while (tick_before != tick_after);
    const uint32_t counts_per_ms = SysTick->LOAD + 1U;
    return tick_before * 1000U +
        (uint32_t)(((counts_per_ms - systick_value) * 1000U) /
                   counts_per_ms);
}

void board_battery_set_multiplier(float multiplier)
{
    if (!isfinite(multiplier) || multiplier < 0.5f || multiplier > 1.5f) return;
    if (battery_voltage_filtered > 0.0f)
        battery_voltage_filtered *= multiplier / battery_voltage_multiplier;
    battery_voltage_multiplier = multiplier;
}

void board_battery_update(void)
{
    if (!battery_adc_pending) {
        const uint32_t now = board_micros();
        if ((int32_t)(now - battery_adc_next_sample_us) < 0) return;
        battery_adc_next_sample_us = now + 25000U;
#if BOARD_HAS_CURRENT
        ADC_ChannelConfTypeDef channel = {0};
        channel.Channel = current_adc_pending ? CURRENT_ADC_CHANNEL : BATTERY_ADC_CHANNEL;
        channel.Rank = 1U;
        channel.SamplingTime = ADC_SAMPLETIME_84CYCLES;
        if (HAL_ADC_ConfigChannel(&hadc_battery, &channel) != HAL_OK) {
            ++battery_adc_errors;
            return;
        }
#endif
        if (HAL_ADC_Start(&hadc_battery) == HAL_OK) {
            battery_adc_pending = true;
            battery_adc_started_us = now;
        } else ++battery_adc_errors;
        return;
    }
    if (HAL_ADC_PollForConversion(&hadc_battery, 0U) != HAL_OK) {
        /* A missed completion must not leave voltage/current frozen forever. */
        if ((uint32_t)(board_micros() - battery_adc_started_us) >= 10000U) {
            HAL_ADC_Stop(&hadc_battery);
            battery_adc_pending = false;
            ++battery_adc_errors;
        }
        return;
    }
    const uint32_t sample = HAL_ADC_GetValue(&hadc_battery);
    HAL_ADC_Stop(&hadc_battery);
    battery_adc_pending = false;
#if BOARD_HAS_CURRENT
    if (current_adc_pending) {
        const uint32_t now = board_micros();
        const uint32_t elapsed_us = now - current_adc_last_us;
        current_adc_raw = sample;
        ++current_adc_count;
        current_adc_last_us = now;
        const float millivolts = (float)sample * 3300.0f / 4095.0f;
        const float measured = millivolts * 10.0f / CURRENT_METER_SCALE;
        /* Filter every fresh current sample (nominally 20 Hz). The former
         * eight-sample average plus 0.15 smoothing hid short throttle bursts.
         * A 100 ms PT1 time constant settles in approximately 400 ms, while
         * elapsed time keeps recovery and timer wraparound well behaved. */
        const float elapsed_s = (float)elapsed_us * 0.000001f;
        const float alpha = elapsed_s / (0.100f + elapsed_s);
        battery_current_filtered = current_adc_count == 1U ? measured :
            battery_current_filtered + alpha * (measured - battery_current_filtered);
        current_adc_pending = false;
        return;
    }
    current_adc_pending = true;
#endif
    battery_adc_total += sample;
    if (++battery_adc_samples < 8U) return;
    const float measured = ((float)battery_adc_total / 8.0f) * 3.3f *
        BATTERY_VOLTAGE_DIVIDER * battery_voltage_multiplier / 4095.0f;
    battery_voltage_filtered = battery_voltage_filtered <= 0.0f ? measured
        : battery_voltage_filtered * 0.85f + measured * 0.15f;
    battery_adc_total = 0U;
    battery_adc_samples = 0U;
}

float board_battery_voltage(void) { return battery_voltage_filtered; }

float board_battery_current(void)
{
#if BOARD_HAS_CURRENT
    return battery_current_filtered;
#else
    return 0.0f;
#endif
}

void board_status_led_set(bool enabled)
{
    HAL_GPIO_WritePin(STATUS_LED_PORT, STATUS_LED_PIN,
                      enabled ? STATUS_LED_ACTIVE_LEVEL
                              : (STATUS_LED_ACTIVE_LEVEL == GPIO_PIN_SET
                                     ? GPIO_PIN_RESET
                                     : GPIO_PIN_SET));
}

void board_status_led_update(bool receiver_signal_valid,
                             bool gyro_calibration_active)
{
    const uint32_t now_us = board_micros();
    if (gyro_calibration_active) {
        board_status_led_set(
            ((now_us / STATUS_LED_CALIBRATION_TOGGLE_US) & 1U) == 0U);
        return;
    }

    const uint32_t phase = now_us % STATUS_LED_PATTERN_PERIOD_US;
    const bool first_flash = phase < STATUS_LED_FLASH_US;
    const bool second_flash = receiver_signal_valid &&
        phase >= STATUS_LED_SECOND_FLASH_US &&
        phase < STATUS_LED_SECOND_FLASH_US + STATUS_LED_FLASH_US;
    board_status_led_set(first_flash || second_flash);
}

void board_buzzer_update(bool requested)
{
    static bool was_requested;
    static uint32_t request_started_us;

    if (!requested) {
        was_requested = false;
        board_buzzer_set(false);
        return;
    }

    const uint32_t now_us = board_micros();
    if (!was_requested) {
        was_requested = true;
        request_started_us = now_us;
    }

    const uint32_t phase =
        (uint32_t)(now_us - request_started_us) %
        BUZZER_PATTERN_PERIOD_US;
    const bool sounding = phase < BUZZER_BEEP_US ||
        (phase >= BUZZER_SECOND_BEEP_US &&
         phase < BUZZER_SECOND_BEEP_US + BUZZER_BEEP_US);
    board_buzzer_set(sounding);
}

void board_check_dfu_request(void)
{
    volatile uint32_t *const request =
        (volatile uint32_t *)DFU_REQUEST_ADDRESS;
    if (*request == DFU_REQUEST_MAGIC) {
        *request = 0U;
        __DSB();
        jump_to_system_bootloader();
    }
}

void board_enter_dfu(void)
{
    volatile uint32_t *const request =
        (volatile uint32_t *)DFU_REQUEST_ADDRESS;
    *request = DFU_REQUEST_MAGIC;
    __DSB();
    NVIC_SystemReset();
    while (1) {
    }
}

void board_fatal_error(void)
{
    __disable_irq();
    while (1) {
        STATUS_LED_PORT->ODR ^= STATUS_LED_PIN;
        for (volatile uint32_t i = 0; i < 2000000U; ++i) {
            __NOP();
        }
    }
}

void board_buzzer_set(bool enabled)
{
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN,
        enabled ? BUZZER_ACTIVE_LEVEL : GPIO_PIN_SET);
}
