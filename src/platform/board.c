#include "board.h"

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi2_tx;
UART_HandleTypeDef hsbus_uart;
#if BOARD_HAS_BATTERY_VOLTAGE
static ADC_HandleTypeDef hadc1;
#endif

#define DFU_REQUEST_ADDRESS 0x2001FFF0U
#define DFU_REQUEST_MAGIC 0x44554634U
#define SYSTEM_MEMORY_ADDRESS 0x1FFF0000U
#define STATUS_LED_PATTERN_PERIOD_US 1000000U
#define STATUS_LED_FLASH_US 80000U
#define STATUS_LED_SECOND_FLASH_US 160000U
#define STATUS_LED_CALIBRATION_TOGGLE_US 100000U
#define BUZZER_PATTERN_PERIOD_US 500000U
#define BUZZER_BEEP_US 70000U
#define BUZZER_SECOND_BEEP_US 120000U

static void jump_to_system_bootloader(void) __attribute__((noreturn));

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
     * This mirrors Betaflight's STM32F4 bootloader-request path.
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
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM = 8;
#if BOARD_CORE_CLOCK_HZ == 168000000U
    osc.PLL.PLLN = 336;
#else
    osc.PLL.PLLN = 192;
#endif
    osc.PLL.PLLP = RCC_PLLP_DIV2;
#if BOARD_CORE_CLOCK_HZ == 168000000U
    osc.PLL.PLLQ = 7;
#else
    osc.PLL.PLLQ = 4;
#endif
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        /*
         * Keep USB diagnostics available even on board revisions where the
         * external oscillator is absent or differs from the reference target.
         * The target-specific PLL values preserve both the core clock and the
         * exact 48 MHz USB clock.
         */
        osc = (RCC_OscInitTypeDef){0};
        osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
        osc.HSIState = RCC_HSI_ON;
        osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
        osc.PLL.PLLState = RCC_PLL_ON;
        osc.PLL.PLLSource = RCC_PLLSOURCE_HSI;
        osc.PLL.PLLM = 16;
#if BOARD_CORE_CLOCK_HZ == 168000000U
        osc.PLL.PLLN = 336;
#else
        osc.PLL.PLLN = 192;
#endif
        osc.PLL.PLLP = RCC_PLLP_DIV2;
#if BOARD_CORE_CLOCK_HZ == 168000000U
        osc.PLL.PLLQ = 7;
#else
        osc.PLL.PLLQ = 4;
#endif
        if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
            board_fatal_error();
        }
    }

    clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
#if BOARD_CORE_CLOCK_HZ == 168000000U
    clk.APB1CLKDivider = RCC_HCLK_DIV4;
    clk.APB2CLKDivider = RCC_HCLK_DIV2;
    const uint32_t flash_latency = FLASH_LATENCY_5;
#else
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    const uint32_t flash_latency = FLASH_LATENCY_3;
#endif
    if (HAL_RCC_ClockConfig(&clk, flash_latency) != HAL_OK) {
        board_fatal_error();
    }
}

static void gpio_init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
#if defined(BOARD_FLYWOOF405NANO)
    __HAL_RCC_GPIOD_CLK_ENABLE();
#endif

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
    /* Push-pull is used by CLRacingF4 and the Flywoo BZ- sink circuit. */
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

static void osd_spi_init(void)
{
#if BOARD_HAS_OSD
#if defined(BOARD_CLRACINGF4)
    __HAL_RCC_SPI3_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &gpio);

    hspi3.Instance = SPI3;
#else
    __HAL_RCC_SPI2_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &gpio);

    hspi2.Instance = SPI2;
#endif

    gpio.Pin = MAX7456_CS_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(MAX7456_CS_PORT, &gpio);
    HAL_GPIO_WritePin(MAX7456_CS_PORT, MAX7456_CS_PIN, GPIO_PIN_SET);

    OSD_SPI_HANDLE.Init.Mode = SPI_MODE_MASTER;
    OSD_SPI_HANDLE.Init.Direction = SPI_DIRECTION_2LINES;
    OSD_SPI_HANDLE.Init.DataSize = SPI_DATASIZE_8BIT;
    OSD_SPI_HANDLE.Init.CLKPolarity = SPI_POLARITY_HIGH;
    OSD_SPI_HANDLE.Init.CLKPhase = SPI_PHASE_2EDGE;
    OSD_SPI_HANDLE.Init.NSS = SPI_NSS_SOFT;
    OSD_SPI_HANDLE.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    OSD_SPI_HANDLE.Init.FirstBit = SPI_FIRSTBIT_MSB;
    OSD_SPI_HANDLE.Init.TIMode = SPI_TIMODE_DISABLE;
    OSD_SPI_HANDLE.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    if (HAL_SPI_Init(&OSD_SPI_HANDLE) != HAL_OK) board_fatal_error();
#endif
}

static void dataflash_spi_init(void)
{
#if BOARD_HAS_DATAFLASH
    __HAL_RCC_SPI3_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &gpio);

    gpio.Pin = DATAFLASH_CS_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(DATAFLASH_CS_PORT, &gpio);
    HAL_GPIO_WritePin(DATAFLASH_CS_PORT, DATAFLASH_CS_PIN, GPIO_PIN_SET);

    DATAFLASH_SPI_HANDLE.Instance = SPI3;
    DATAFLASH_SPI_HANDLE.Init.Mode = SPI_MODE_MASTER;
    DATAFLASH_SPI_HANDLE.Init.Direction = SPI_DIRECTION_2LINES;
    DATAFLASH_SPI_HANDLE.Init.DataSize = SPI_DATASIZE_8BIT;
    DATAFLASH_SPI_HANDLE.Init.CLKPolarity = SPI_POLARITY_LOW;
    DATAFLASH_SPI_HANDLE.Init.CLKPhase = SPI_PHASE_1EDGE;
    DATAFLASH_SPI_HANDLE.Init.NSS = SPI_NSS_SOFT;
    DATAFLASH_SPI_HANDLE.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    DATAFLASH_SPI_HANDLE.Init.FirstBit = SPI_FIRSTBIT_MSB;
    DATAFLASH_SPI_HANDLE.Init.TIMode = SPI_TIMODE_DISABLE;
    DATAFLASH_SPI_HANDLE.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    if (HAL_SPI_Init(&DATAFLASH_SPI_HANDLE) != HAL_OK) board_fatal_error();
#endif
}

static void sdcard_spi_init(void)
{
#if BOARD_HAS_SDCARD
    __HAL_RCC_SPI2_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin = SDCARD_CS_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(SDCARD_CS_PORT, &gpio);
    HAL_GPIO_WritePin(SDCARD_CS_PORT, SDCARD_CS_PIN, GPIO_PIN_SET);

    gpio.Pin = SDCARD_DETECT_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(SDCARD_DETECT_PORT, &gpio);

    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_MASTER;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi2.Init.NSS = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    if (HAL_SPI_Init(&hspi2) != HAL_OK) board_fatal_error();

    hdma_spi2_tx.Instance = DMA1_Stream4;
    hdma_spi2_tx.Init.Channel = DMA_CHANNEL_0;
    hdma_spi2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi2_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi2_tx.Init.Mode = DMA_NORMAL;
    hdma_spi2_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_spi2_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi2_tx) != HAL_OK) board_fatal_error();
    __HAL_LINKDMA(&hspi2, hdmatx, hdma_spi2_tx);
    HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 7U, 0U);
    HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
#endif
}

static void sbus_uart_init(void)
{
    SBUS_UART_CLOCK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = SBUS_RX_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = SBUS_RX_AF;
    HAL_GPIO_Init(SBUS_RX_PORT, &gpio);

    hsbus_uart.Instance = SBUS_UART_INSTANCE;
    hsbus_uart.Init.BaudRate = 100000;
    hsbus_uart.Init.WordLength = UART_WORDLENGTH_9B;
    hsbus_uart.Init.StopBits = UART_STOPBITS_2;
    hsbus_uart.Init.Parity = UART_PARITY_EVEN;
    hsbus_uart.Init.Mode = UART_MODE_RX;
    hsbus_uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    hsbus_uart.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&hsbus_uart) != HAL_OK) {
        board_fatal_error();
    }
    HAL_NVIC_SetPriority(SBUS_UART_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(SBUS_UART_IRQn);
}

static void battery_adc_init(void)
{
#if BOARD_HAS_BATTERY_VOLTAGE
    __HAL_RCC_ADC1_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = BATTERY_ADC_PIN;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BATTERY_ADC_PORT, &gpio);

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1U;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) board_fatal_error();

    ADC_ChannelConfTypeDef channel = {0};
    channel.Channel = BATTERY_ADC_CHANNEL;
    channel.Rank = 1U;
    channel.SamplingTime = ADC_SAMPLETIME_84CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &channel) != HAL_OK) {
        board_fatal_error();
    }
#endif
}

void board_init(void)
{
    HAL_Init();
    clock_init();
    gpio_init();
    spi1_init();
    osd_spi_init();
    dataflash_spi_init();
    sdcard_spi_init();
    sbus_uart_init();
    battery_adc_init();

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

float board_battery_voltage(void)
{
#if BOARD_HAS_BATTERY_VOLTAGE
    uint32_t total = 0U;
    for (uint8_t i = 0U; i < 8U; ++i) {
        if (HAL_ADC_Start(&hadc1) != HAL_OK ||
            HAL_ADC_PollForConversion(&hadc1, 100U) != HAL_OK) {
            HAL_ADC_Stop(&hadc1);
            return 0.0f;
        }
        total += HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
    }
    const float measured =
        ((float)total / 8.0f) * 3.3f *
        BATTERY_VOLTAGE_DIVIDER / 4095.0f;
    static float filtered;
    filtered = filtered <= 0.0f ? measured : filtered * 0.85f + measured * 0.15f;
    return filtered;
#else
    return 0.0f;
#endif
}

uint32_t board_micros(void)
{
    /*
     * Do not derive this value by dividing DWT->CYCCNT directly.  At 96 MHz
     * that 32-bit counter wraps every 44.7 seconds; division before unsigned
     * subtraction breaks all timeout calculations at each CPU-cycle wrap.
     *
     * SysTick supplies a true 32-bit microsecond time base: HAL_GetTick()
     * provides the milliseconds and the down-counter provides the fraction.
     * Read the millisecond counter on both sides so a concurrent SysTick IRQ
     * cannot combine values from two different milliseconds.
     */
    uint32_t tick_before;
    uint32_t tick_after;
    uint32_t systick_value;
    do {
        tick_before = HAL_GetTick();
        systick_value = SysTick->VAL;
        tick_after = HAL_GetTick();
    } while (tick_before != tick_after);

    const uint32_t counts_per_ms = SysTick->LOAD + 1U;
    const uint32_t elapsed_counts = counts_per_ms - systick_value;
    const uint32_t fractional_us =
        (uint32_t)(((uint64_t)elapsed_counts * 1000U) / counts_per_ms);
    return tick_before * 1000U + fractional_us;
}

void board_buzzer_set(bool enabled)
{
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN,
                      enabled ? BUZZER_ACTIVE_LEVEL
                              : (BUZZER_ACTIVE_LEVEL == GPIO_PIN_SET
                                     ? GPIO_PIN_RESET
                                     : GPIO_PIN_SET));
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
#if BOARD_BUZZER_REQUIRES_TONE
    if (sounding) {
        /*
         * PB4 drives a passive beeper on CLRacingF4.  This function runs at
         * the 8 kHz control-loop rate, so toggling once per call produces the
         * highest possible square wave here: 4 kHz, close to its most audible
         * range.
         */
        HAL_GPIO_TogglePin(BUZZER_PORT, BUZZER_PIN);
    } else {
        board_buzzer_set(false);
    }
#else
    /* An active buzzer only needs the sounding envelope. */
    board_buzzer_set(sounding);
#endif
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
