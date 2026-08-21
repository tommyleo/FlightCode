#include "board.h"

#include <math.h>

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi2_tx;
UART_HandleTypeDef hsbus_uart;
static ADC_HandleTypeDef hadc1;

#define DFU_REQUEST_ADDRESS 0x2001FFF0U
#define DFU_REQUEST_MAGIC 0x44554634U
#define SYSTEM_MEMORY_ADDRESS 0x1FF09800U
#define STATUS_LED_PATTERN_PERIOD_US 1000000U
#define STATUS_LED_FLASH_US 80000U
#define STATUS_LED_SECOND_FLASH_US 160000U
#define STATUS_LED_CALIBRATION_TOGGLE_US 100000U
#define BUZZER_PATTERN_PERIOD_US 500000U
#define BUZZER_BEEP_US 70000U
#define BUZZER_SECOND_BEEP_US 120000U

static GPIO_TypeDef *active_imu_cs_port = IMU_PRIMARY_CS_PORT;
static uint16_t active_imu_cs_pin = IMU_PRIMARY_CS_PIN;

void board_imu_select(uint8_t candidate)
{
    active_imu_cs_port = candidate == 0U ? IMU_PRIMARY_CS_PORT : IMU_ALT_CS_PORT;
    active_imu_cs_pin = candidate == 0U ? IMU_PRIMARY_CS_PIN : IMU_ALT_CS_PIN;
}

GPIO_TypeDef *board_imu_cs_port(void) { return active_imu_cs_port; }
uint16_t board_imu_cs_pin(void) { return active_imu_cs_pin; }

static void clock_init(void)
{
    if (HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY) != HAL_OK) board_fatal_error();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
    }

    RCC_OscInitTypeDef osc = {0};
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI48;
    osc.HSEState = RCC_HSE_ON;
    osc.HSI48State = RCC_HSI48_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM = 4U;
    osc.PLL.PLLN = 240U;
    osc.PLL.PLLP = 2U;
    osc.PLL.PLLQ = 20U;
    osc.PLL.PLLR = 2U;
    osc.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
    osc.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    osc.PLL.PLLFRACN = 0U;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) board_fatal_error();

    RCC_ClkInitTypeDef clk = {0};
    clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                    RCC_CLOCKTYPE_D1PCLK1 | RCC_CLOCKTYPE_PCLK1 |
                    RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.SYSCLKDivider = RCC_SYSCLK_DIV1;
    clk.AHBCLKDivider = RCC_HCLK_DIV2;
    clk.APB3CLKDivider = RCC_APB3_DIV2;
    clk.APB1CLKDivider = RCC_APB1_DIV2;
    clk.APB2CLKDivider = RCC_APB2_DIV2;
    clk.APB4CLKDivider = RCC_APB4_DIV2;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_4) != HAL_OK) board_fatal_error();

    RCC_PeriphCLKInitTypeDef periph = {0};
    periph.PeriphClockSelection = RCC_PERIPHCLK_USB;
    periph.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
    if (HAL_RCCEx_PeriphCLKConfig(&periph) != HAL_OK) board_fatal_error();
}

static void gpio_init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {
        .Mode = GPIO_MODE_OUTPUT_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
    };
    HAL_GPIO_WritePin(GPIOC, MOTOR_1_PIN | MOTOR_2_PIN |
                             MOTOR_3_PIN | MOTOR_4_PIN, GPIO_PIN_RESET);
    gpio.Pin = MOTOR_1_PIN | MOTOR_2_PIN | MOTOR_3_PIN | MOTOR_4_PIN;
    HAL_GPIO_Init(GPIOC, &gpio);

    gpio.Pin = STATUS_LED_PIN;
    HAL_GPIO_Init(STATUS_LED_PORT, &gpio);
    board_status_led_set(false);

    gpio.Pin = IMU_PRIMARY_CS_PIN | IMU_ALT_CS_PIN;
    HAL_GPIO_Init(GPIOB, &gpio);
    HAL_GPIO_WritePin(GPIOB, IMU_PRIMARY_CS_PIN | IMU_ALT_CS_PIN, GPIO_PIN_SET);

    gpio.Pin = BUZZER_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BUZZER_PORT, &gpio);
    board_buzzer_set(false);
}

static void spi_init(void)
{
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_SPI2_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {
        .Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7,
        .Mode = GPIO_MODE_AF_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
        .Alternate = GPIO_AF5_SPI1,
    };
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
    hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
    hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
    hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
    hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
    hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
    hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) board_fatal_error();

    gpio.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    gpio.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin = DATAFLASH_CS_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(DATAFLASH_CS_PORT, &gpio);
    HAL_GPIO_WritePin(DATAFLASH_CS_PORT, DATAFLASH_CS_PIN, GPIO_PIN_SET);

    hspi2 = (SPI_HandleTypeDef){0};
    hspi2.Instance = SPI2;
    hspi2.Init = hspi1.Init;
    hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    if (HAL_SPI_Init(&hspi2) != HAL_OK) board_fatal_error();
}

bool board_receiver_uart_configure(bool crsf)
{
    if (hsbus_uart.Instance != NULL) {
        HAL_NVIC_DisableIRQ(hsbus_uart.Instance == CRSF_UART_INSTANCE
                            ? CRSF_UART_IRQn : SBUS_UART_IRQn);
        (void)HAL_UART_DeInit(&hsbus_uart);
    }
    GPIO_InitTypeDef gpio = {
        .Mode = GPIO_MODE_AF_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
    };
    if (crsf) {
        CRSF_UART_CLOCK_ENABLE();
        gpio.Pin = CRSF_RX_PIN;
        gpio.Alternate = CRSF_RX_AF;
        HAL_GPIO_Init(CRSF_RX_PORT, &gpio);
        hsbus_uart.Instance = CRSF_UART_INSTANCE;
        hsbus_uart.Init.BaudRate = 420000U;
        hsbus_uart.Init.WordLength = UART_WORDLENGTH_8B;
        hsbus_uart.Init.StopBits = UART_STOPBITS_1;
        hsbus_uart.Init.Parity = UART_PARITY_NONE;
        hsbus_uart.Init.Mode = UART_MODE_RX;
        hsbus_uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
        hsbus_uart.Init.OverSampling = UART_OVERSAMPLING_16;
        hsbus_uart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
        if (HAL_UART_Init(&hsbus_uart) != HAL_OK) return false;
        HAL_NVIC_SetPriority(CRSF_UART_IRQn, 5U, 0U);
        HAL_NVIC_EnableIRQ(CRSF_UART_IRQn);
        return true;
    }
    SBUS_UART_CLOCK_ENABLE();
    gpio.Pin = SBUS_RX_PIN;
    gpio.Alternate = SBUS_RX_AF;
    HAL_GPIO_Init(SBUS_RX_PORT, &gpio);
    hsbus_uart.Instance = SBUS_UART_INSTANCE;
    hsbus_uart.Init.BaudRate = 100000U;
    hsbus_uart.Init.WordLength = UART_WORDLENGTH_9B;
    hsbus_uart.Init.StopBits = UART_STOPBITS_2;
    hsbus_uart.Init.Parity = UART_PARITY_EVEN;
    hsbus_uart.Init.Mode = UART_MODE_RX;
    hsbus_uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    hsbus_uart.Init.OverSampling = UART_OVERSAMPLING_16;
    hsbus_uart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    if (HAL_UART_Init(&hsbus_uart) != HAL_OK) return false;
    HAL_NVIC_SetPriority(SBUS_UART_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(SBUS_UART_IRQn);
    return true;
}

static void battery_adc_init(void)
{
    __HAL_RCC_ADC12_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {
        .Pin = BATTERY_ADC_PIN,
        .Mode = GPIO_MODE_ANALOG,
        .Pull = GPIO_NOPULL,
    };
    HAL_GPIO_Init(BATTERY_ADC_PORT, &gpio);
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV8;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.NbrOfConversion = 1U;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
    hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
    hadc1.Init.OversamplingMode = DISABLE;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) board_fatal_error();
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
        board_fatal_error();
    ADC_ChannelConfTypeDef channel = {0};
    channel.Channel = BATTERY_ADC_CHANNEL;
    channel.Rank = ADC_REGULAR_RANK_1;
    channel.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;
    channel.SingleDiff = ADC_SINGLE_ENDED;
    channel.OffsetNumber = ADC_OFFSET_NONE;
    channel.Offset = 0U;
    if (HAL_ADC_ConfigChannel(&hadc1, &channel) != HAL_OK) board_fatal_error();
}

void board_init(void)
{
    HAL_Init();
    clock_init();
    gpio_init();
    spi_init();
    if (!board_receiver_uart_configure(true)) board_fatal_error();
    battery_adc_init();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint32_t battery_adc_total;
static uint32_t battery_adc_next_sample_us;
static uint8_t battery_adc_samples;
static bool battery_adc_pending;
static float battery_voltage_filtered;
static float battery_voltage_multiplier = 1.0f;

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
        (uint32_t)(((uint64_t)(counts_per_ms - systick_value) * 1000U) /
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
        if (HAL_ADC_Start(&hadc1) == HAL_OK) {
            battery_adc_pending = true;
            battery_adc_next_sample_us = now + 25000U;
        }
        return;
    }
    if (HAL_ADC_PollForConversion(&hadc1, 0U) != HAL_OK) return;
    battery_adc_total += HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    battery_adc_pending = false;
    if (++battery_adc_samples < 8U) return;
    const float measured = ((float)battery_adc_total / 8.0f) * 3.3f *
        BATTERY_VOLTAGE_DIVIDER * battery_voltage_multiplier / 4095.0f;
    battery_voltage_filtered = battery_voltage_filtered <= 0.0f ? measured
        : battery_voltage_filtered * 0.85f + measured * 0.15f;
    battery_adc_total = 0U;
    battery_adc_samples = 0U;
}

float board_battery_voltage(void) { return battery_voltage_filtered; }

void board_buzzer_set(bool enabled)
{
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN,
        enabled ? BUZZER_ACTIVE_LEVEL : GPIO_PIN_SET);
}

void board_status_led_set(bool enabled)
{
    HAL_GPIO_WritePin(STATUS_LED_PORT, STATUS_LED_PIN,
        enabled ? STATUS_LED_ACTIVE_LEVEL : GPIO_PIN_SET);
}

void board_status_led_update(bool receiver_valid, bool calibrating)
{
    const uint32_t now = board_micros();
    if (calibrating) {
        board_status_led_set(((now / STATUS_LED_CALIBRATION_TOGGLE_US) & 1U) == 0U);
        return;
    }
    const uint32_t phase = now % STATUS_LED_PATTERN_PERIOD_US;
    board_status_led_set(phase < STATUS_LED_FLASH_US ||
        (receiver_valid && phase >= STATUS_LED_SECOND_FLASH_US &&
         phase < STATUS_LED_SECOND_FLASH_US + STATUS_LED_FLASH_US));
}

void board_buzzer_update(bool requested)
{
    static bool was_requested;
    static uint32_t started;
    if (!requested) {
        was_requested = false;
        board_buzzer_set(false);
        return;
    }
    if (!was_requested) { was_requested = true; started = board_micros(); }
    const uint32_t phase = (board_micros() - started) % BUZZER_PATTERN_PERIOD_US;
    board_buzzer_set(phase < BUZZER_BEEP_US ||
        (phase >= BUZZER_SECOND_BEEP_US &&
         phase < BUZZER_SECOND_BEEP_US + BUZZER_BEEP_US));
}

static void jump_to_system_bootloader(void) __attribute__((noreturn));
static void jump_to_system_bootloader(void)
{
    const uint32_t stack = *(volatile uint32_t *)SYSTEM_MEMORY_ADDRESS;
    const uint32_t entry = *(volatile uint32_t *)(SYSTEM_MEMORY_ADDRESS + 4U);
    __disable_irq();
    HAL_RCC_DeInit();
    SysTick->CTRL = 0U;
    SCB_DisableDCache();
    SCB_DisableICache();
    SCB->VTOR = SYSTEM_MEMORY_ADDRESS;
    __set_MSP(stack);
    ((void (*)(void))entry)();
    while (1) {
    }
}

void board_check_dfu_request(void)
{
    volatile uint32_t *request = (volatile uint32_t *)DFU_REQUEST_ADDRESS;
    if (*request == DFU_REQUEST_MAGIC) {
        *request = 0U;
        jump_to_system_bootloader();
    }
}

void board_enter_dfu(void)
{
    *(volatile uint32_t *)DFU_REQUEST_ADDRESS = DFU_REQUEST_MAGIC;
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
        for (volatile uint32_t i = 0U; i < 8000000U; ++i) __NOP();
    }
}
