#include "board.h"

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
UART_HandleTypeDef hsbus_uart;
#if BOARD_HAS_BATTERY_VOLTAGE
static ADC_HandleTypeDef hadc1;
#endif

#define DFU_REQUEST_ADDRESS 0x2001FFF0U
#define DFU_REQUEST_MAGIC 0x44554634U
#define SYSTEM_MEMORY_ADDRESS 0x1FFF0000U

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
#if defined(BOARD_CLRACINGF4)
    osc.PLL.PLLN = 336;
#else
    osc.PLL.PLLN = 192;
#endif
    osc.PLL.PLLP = RCC_PLLP_DIV2;
#if defined(BOARD_CLRACINGF4)
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
#if defined(BOARD_CLRACINGF4)
        osc.PLL.PLLN = 336;
#else
        osc.PLL.PLLN = 192;
#endif
        osc.PLL.PLLP = RCC_PLLP_DIV2;
#if defined(BOARD_CLRACINGF4)
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
#if defined(BOARD_CLRACINGF4)
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

    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

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
    gpio.Pin = SBUS_INVERTER_PIN;
    HAL_GPIO_Init(SBUS_INVERTER_PORT, &gpio);
    gpio.Pin = MPU6000_CS_PIN;
    HAL_GPIO_Init(MPU6000_CS_PORT, &gpio);
    HAL_GPIO_WritePin(MPU6000_CS_PORT, MPU6000_CS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SBUS_INVERTER_PORT,
                      SBUS_INVERTER_PIN,
                      SBUS_INVERTER_ENABLE_LEVEL);

    gpio.Pin = BUZZER_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BUZZER_PORT, &gpio);
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);
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

static void spi2_init(void)
{
#if BOARD_HAS_OSD
    __HAL_RCC_SPI2_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin = MAX7456_CS_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(MAX7456_CS_PORT, &gpio);
    HAL_GPIO_WritePin(MAX7456_CS_PORT, MAX7456_CS_PIN, GPIO_PIN_SET);

    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_MASTER;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi2.Init.NSS = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    if (HAL_SPI_Init(&hspi2) != HAL_OK) board_fatal_error();
#endif
}

static void sbus_uart_init(void)
{
    __HAL_RCC_USART1_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_10;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &gpio);

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
    gpio.Pin = GPIO_PIN_0;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);

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
    channel.Channel = ADC_CHANNEL_0;
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
    spi2_init();
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
    /* Betaflight MAMBAF411 default: PA0 VBAT divider, scale 110 (11:1). */
    const float measured = ((float)total / 8.0f) * 3.3f * 11.0f / 4095.0f;
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
