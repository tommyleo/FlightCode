#include "board.h"

#include <math.h>

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi2_tx;
UART_HandleTypeDef hsbus_uart;
static ADC_HandleTypeDef hadc_battery;

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

#if defined(BOARD_FOXEERH743)
#include "foxeer_uart.h"
#else
bool board_uart_half_duplex_init(uint8_t port, uint32_t baud_rate, uint32_t stop_bits,
                                 UART_HandleTypeDef *handle)
{
    USART_TypeDef *instance;
    GPIO_TypeDef *gpio_port;
    uint16_t gpio_pin;
    uint32_t gpio_af;
    if (port == 1U) {
        instance = USART1; gpio_port = GPIOA; gpio_pin = GPIO_PIN_9;
        gpio_af = GPIO_AF7_USART1; __HAL_RCC_USART1_CLK_ENABLE();
    } else if (port == 2U) {
        instance = USART2; gpio_port = GPIOA; gpio_pin = GPIO_PIN_2;
        gpio_af = GPIO_AF7_USART2; __HAL_RCC_USART2_CLK_ENABLE();
    } else if (port == 4U) {
        instance = UART4; gpio_port = GPIOA; gpio_pin = GPIO_PIN_0;
        gpio_af = GPIO_AF8_UART4; __HAL_RCC_UART4_CLK_ENABLE();
#if defined(BOARD_SEQUREH7V2)
    } else if (port == 6U) {
        instance = USART6; gpio_port = GPIOC; gpio_pin = GPIO_PIN_6;
        gpio_af = GPIO_AF7_USART6; __HAL_RCC_USART6_CLK_ENABLE();
    } else if (port == 7U) {
        instance = UART7; gpio_port = GPIOE; gpio_pin = GPIO_PIN_8;
        gpio_af = GPIO_AF7_UART7; __HAL_RCC_UART7_CLK_ENABLE();
#endif
    } else {
        return false;
    }

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = gpio_pin; gpio.Mode = GPIO_MODE_AF_OD;
    gpio.Pull = GPIO_PULLUP; gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = gpio_af;
    HAL_GPIO_Init(gpio_port, &gpio);

    *handle = (UART_HandleTypeDef){0};
    handle->Instance = instance; handle->Init.BaudRate = baud_rate;
    handle->Init.WordLength = UART_WORDLENGTH_8B;
    handle->Init.StopBits = stop_bits; handle->Init.Parity = UART_PARITY_NONE;
    handle->Init.Mode = UART_MODE_TX_RX; handle->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    handle->Init.OverSampling = UART_OVERSAMPLING_16;
    return HAL_HalfDuplex_Init(handle) == HAL_OK;
}

static bool board_uart_serial_init(uint8_t port, uint32_t baud_rate,
                        UART_HandleTypeDef *handle, bool receive)
{
    USART_TypeDef *instance = NULL;
    GPIO_TypeDef *gpio_port = NULL;
    uint16_t gpio_pin = 0U;
    uint32_t gpio_af = 0U;
    if (port == 1U) {
#if defined(BOARD_SEQUREH7V2)
        instance = USART1; gpio_port = GPIOA; gpio_pin = GPIO_PIN_9;
#else
        instance = USART1; gpio_port = GPIOB; gpio_pin = GPIO_PIN_6;
#endif
        gpio_af = GPIO_AF7_USART1; __HAL_RCC_USART1_CLK_ENABLE();
    } else if (port == 2U) {
        instance = USART2; gpio_port = GPIOA; gpio_pin = GPIO_PIN_2;
        gpio_af = GPIO_AF7_USART2; __HAL_RCC_USART2_CLK_ENABLE();
    } else if (port == 4U) {
        instance = UART4; gpio_port = GPIOA; gpio_pin = GPIO_PIN_0;
        gpio_af = GPIO_AF8_UART4; __HAL_RCC_UART4_CLK_ENABLE();
    } else if (port == 5U) {
        instance = UART5; gpio_port = GPIOC; gpio_pin = GPIO_PIN_12;
        gpio_af = GPIO_AF8_UART5; __HAL_RCC_UART5_CLK_ENABLE();
#if defined(BOARD_SEQUREH7V2)
    } else if (port == 6U) {
        instance = USART6; gpio_port = GPIOC; gpio_pin = GPIO_PIN_6;
        gpio_af = GPIO_AF7_USART6; __HAL_RCC_USART6_CLK_ENABLE();
    } else if (port == 7U) {
        instance = UART7; gpio_port = GPIOE; gpio_pin = GPIO_PIN_8;
        gpio_af = GPIO_AF7_UART7; __HAL_RCC_UART7_CLK_ENABLE();
#endif
    }
    if (instance == NULL) return false;

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = gpio_pin; gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL; gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = gpio_af;
    HAL_GPIO_Init(gpio_port, &gpio);

    if (receive) {
        GPIO_TypeDef *rx_port = gpio_port;
        uint16_t rx_pin = (uint16_t)(gpio_pin << 1);
        if (port == 5U) { rx_port = GPIOD; rx_pin = GPIO_PIN_2; }
        if (port == 7U) rx_pin = GPIO_PIN_7;
        gpio.Pin = rx_pin;
        HAL_GPIO_Init(rx_port, &gpio);
    }
    *handle = (UART_HandleTypeDef){0};
    handle->Instance = instance; handle->Init.BaudRate = baud_rate;
    handle->Init.WordLength = UART_WORDLENGTH_8B;
    handle->Init.StopBits = UART_STOPBITS_1; handle->Init.Parity = UART_PARITY_NONE;
    handle->Init.Mode = receive ? UART_MODE_TX_RX : UART_MODE_TX; handle->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    handle->Init.OverSampling = UART_OVERSAMPLING_16;
    return HAL_UART_Init(handle) == HAL_OK;
}

bool board_uart_tx_init(uint8_t port, uint32_t baud, UART_HandleTypeDef *handle)
{ return board_uart_serial_init(port, baud, handle, false); }
bool board_uart_tx_rx_init(uint8_t port, uint32_t baud, UART_HandleTypeDef *handle)
{ return board_uart_serial_init(port, baud, handle, true); }

#endif

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
#if defined(BOARD_SEQUREH7V2) || defined(BOARD_FOXEERH743)
    /* VOS1/400 MHz also supports STM32H743 silicon revision Y. VOS0 is only
     * available on revision V and later. */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
#else
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
#endif
    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
    }

#if defined(BOARD_SEQUREH7V2) || defined(BOARD_FOXEERH743)
    /* Apply this workaround on STM32H743/H750 before enabling
     * HSE.  On affected silicon HSERDY can otherwise take many seconds (or
     * time out altogether), which leaves the board apparently dead before
     * USB is ever initialized.  PH0/PH1 are the HSE oscillator pins. */
    __HAL_RCC_GPIOH_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);
    GPIO_InitTypeDef hse_gpio = {0};
    hse_gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    hse_gpio.Mode = GPIO_MODE_OUTPUT_PP;
    hse_gpio.Pull = GPIO_NOPULL;
    hse_gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOH, &hse_gpio);
#endif

    RCC_OscInitTypeDef osc = {0};
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE |
                         RCC_OSCILLATORTYPE_HSI48;
    osc.HSEState = RCC_HSE_ON;
    osc.HSI48State = RCC_HSI48_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLM = 4U;
#if defined(BOARD_SEQUREH7V2) || defined(BOARD_FOXEERH743)
    /* 8 MHz HSE / 4 * 400 / 2 = 400 MHz SYSCLK. */
    osc.PLL.PLLN = 400U;
#else
    /* 8 MHz HSE / 4 * 480 / 2 = 480 MHz SYSCLK. */
    osc.PLL.PLLN = 480U;
#endif
    osc.PLL.PLLP = 2U;
    osc.PLL.PLLQ = 8U;
    osc.PLL.PLLR = 5U;
    osc.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
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
    if (HAL_RCC_ClockConfig(&clk,
#if defined(BOARD_SEQUREH7V2) || defined(BOARD_FOXEERH743)
                            FLASH_LATENCY_2
#else
                            FLASH_LATENCY_4
#endif
                            ) != HAL_OK) board_fatal_error();

    /* Enable the H7 I/O compensation cell before bringing up
     * high-speed peripherals, including USB. */
    __HAL_RCC_CSI_ENABLE();
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    HAL_EnableCompensationCell();
    HAL_Delay(10U);

    RCC_PeriphCLKInitTypeDef periph = {0};
    periph.PeriphClockSelection = RCC_PERIPHCLK_USB;
#if defined(BOARD_SEQUREH7V2) || defined(BOARD_FOXEERH743)
    periph.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
#else
    /* Generate USB's exact 48 MHz from a dedicated PLL instead of relying on
     * the free-running HSI48 oscillator. */
    periph.PLL3.PLL3M = 4U;
    periph.PLL3.PLL3N = 96U;
    periph.PLL3.PLL3P = 2U;
    periph.PLL3.PLL3Q = 4U;
    periph.PLL3.PLL3R = 2U;
    periph.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_1;
    periph.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
    periph.PLL3.PLL3FRACN = 0U;
    periph.UsbClockSelection = RCC_USBCLKSOURCE_PLL3;
#endif
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
    HAL_GPIO_WritePin(MOTOR_1_PORT, MOTOR_1_PIN | MOTOR_2_PIN |
                             MOTOR_3_PIN | MOTOR_4_PIN, GPIO_PIN_RESET);
    gpio.Pin = MOTOR_1_PIN | MOTOR_2_PIN | MOTOR_3_PIN | MOTOR_4_PIN;
    HAL_GPIO_Init(MOTOR_1_PORT, &gpio);

    gpio.Pin = STATUS_LED_PIN;
    HAL_GPIO_Init(STATUS_LED_PORT, &gpio);
    board_status_led_set(false);

    gpio.Pin = IMU_PRIMARY_CS_PIN | IMU_ALT_CS_PIN;
    HAL_GPIO_Init(GPIOB, &gpio);
    HAL_GPIO_WritePin(GPIOB, IMU_PRIMARY_CS_PIN | IMU_ALT_CS_PIN, GPIO_PIN_SET);

    gpio.Pin = BUZZER_PIN;
#if BOARD_BUZZER_OUTPUT_OPEN_DRAIN
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_PULLUP;
#else
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
#endif
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BUZZER_PORT, &gpio);
    board_buzzer_set(false);
}

static void spi_init(void)
{
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_SPI2_CLK_ENABLE();
#if defined(BOARD_SEQUREH7V2) || defined(BOARD_FOXEERH743)
    __HAL_RCC_SPI3_CLK_ENABLE();
#endif

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

#if defined(BOARD_SEQUREH7V2) || defined(BOARD_FOXEERH743)
    gpio.Pin = MAX7456_CS_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Alternate = 0U;
    HAL_GPIO_Init(MAX7456_CS_PORT, &gpio);
    HAL_GPIO_WritePin(MAX7456_CS_PORT, MAX7456_CS_PIN, GPIO_PIN_SET);
#endif

    gpio.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &gpio);
#if !defined(BOARD_SEQUREH7V2) && !defined(BOARD_FOXEERH743)
    gpio.Pin = DATAFLASH_CS_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(DATAFLASH_CS_PORT, &gpio);
    HAL_GPIO_WritePin(DATAFLASH_CS_PORT, DATAFLASH_CS_PIN, GPIO_PIN_SET);
#endif

    hspi2 = (SPI_HandleTypeDef){0};
    hspi2.Instance = SPI2;
    hspi2.Init = hspi1.Init;
    hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    if (HAL_SPI_Init(&hspi2) != HAL_OK) board_fatal_error();
#if defined(BOARD_SEQUREH7V2) || defined(BOARD_FOXEERH743)
    gpio.Pin = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &gpio);
    gpio.Pin = DATAFLASH_CS_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Alternate = 0U;
    HAL_GPIO_Init(DATAFLASH_CS_PORT, &gpio);
    HAL_GPIO_WritePin(DATAFLASH_CS_PORT, DATAFLASH_CS_PIN, GPIO_PIN_SET);
    hspi3 = (SPI_HandleTypeDef){0};
    hspi3.Instance = SPI3;
    hspi3.Init = hspi2.Init;
    if (HAL_SPI_Init(&hspi3) != HAL_OK) board_fatal_error();
#endif
}

#if defined(BOARD_SEQUREH7V2)
static bool sequre_receiver_uart(uint8_t port, USART_TypeDef **instance,
                                 GPIO_TypeDef **gpio_port, uint16_t *pin,
                                 uint32_t *af, IRQn_Type *irq)
{
    switch (port) {
    case 1U:
        *instance = USART1; *gpio_port = GPIOA; *pin = GPIO_PIN_10;
        *af = GPIO_AF7_USART1; *irq = USART1_IRQn;
        __HAL_RCC_USART1_CLK_ENABLE(); return true;
    case 2U:
        *instance = USART2; *gpio_port = GPIOA; *pin = GPIO_PIN_3;
        *af = GPIO_AF7_USART2; *irq = USART2_IRQn;
        __HAL_RCC_USART2_CLK_ENABLE(); return true;
    case 4U:
        *instance = UART4; *gpio_port = GPIOA; *pin = GPIO_PIN_1;
        *af = GPIO_AF8_UART4; *irq = UART4_IRQn;
        __HAL_RCC_UART4_CLK_ENABLE(); return true;
    case 6U:
        *instance = USART6; *gpio_port = GPIOC; *pin = GPIO_PIN_7;
        *af = GPIO_AF7_USART6; *irq = USART6_IRQn;
        __HAL_RCC_USART6_CLK_ENABLE(); return true;
    case 7U:
        *instance = UART7; *gpio_port = GPIOE; *pin = GPIO_PIN_7;
        *af = GPIO_AF7_UART7; *irq = UART7_IRQn;
        __HAL_RCC_UART7_CLK_ENABLE(); return true;
    default:
        return false;
    }
}
#endif

#if !defined(BOARD_FOXEERH743)
#include "receiver_uart_tx.h"

bool board_receiver_uart_transmit(const uint8_t *bytes, uint16_t length)
{
#if defined(BOARD_SEQUREH7V2)
    GPIO_TypeDef *port = GPIOA;
    uint16_t pin;
    uint32_t af;
    if (hsbus_uart.Instance == USART1) { pin=GPIO_PIN_9; af=GPIO_AF7_USART1; }
    else if (hsbus_uart.Instance == USART2) { pin=GPIO_PIN_2; af=GPIO_AF7_USART2; }
    else if (hsbus_uart.Instance == UART4) { pin=GPIO_PIN_0; af=GPIO_AF8_UART4; }
    else if (hsbus_uart.Instance == USART6) { port=GPIOC; pin=GPIO_PIN_6; af=GPIO_AF7_USART6; }
    else if (hsbus_uart.Instance == UART7) { port=GPIOE; pin=GPIO_PIN_8; af=GPIO_AF7_UART7; }
    else return false;
    return receiver_uart_transmit(port, pin, af, bytes, length);
#else
    if (hsbus_uart.Instance != CRSF_UART_INSTANCE) return false;
    return receiver_uart_transmit(GPIOB, GPIO_PIN_6, GPIO_AF7_USART1,
                                  bytes, length);
#endif
}

bool board_receiver_uart_configure(bool crsf, uint8_t port)
{
#if defined(BOARD_SEQUREH7V2) || defined(BOARD_FOXEERH743)
    USART_TypeDef *instance = NULL;
    GPIO_TypeDef *gpio_port = NULL;
    uint16_t pin = 0U;
    uint32_t af = 0U;
    IRQn_Type irq = USART1_IRQn;
    if (!sequre_receiver_uart(port, &instance, &gpio_port, &pin, &af, &irq))
        return false;
    if (hsbus_uart.Instance != NULL) {
        const IRQn_Type old_irq = hsbus_uart.Instance == USART1 ? USART1_IRQn :
            hsbus_uart.Instance == USART2 ? USART2_IRQn :
            hsbus_uart.Instance == UART4 ? UART4_IRQn :
            hsbus_uart.Instance == USART6 ? USART6_IRQn : UART7_IRQn;
        HAL_NVIC_DisableIRQ(old_irq);
        (void)HAL_UART_DeInit(&hsbus_uart);
    }
    hsbus_uart = (UART_HandleTypeDef){0};
    GPIO_InitTypeDef gpio = {
        .Pin = pin, .Mode = GPIO_MODE_AF_PP, .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH, .Alternate = af,
    };
    HAL_GPIO_Init(gpio_port, &gpio);
    hsbus_uart.Instance = instance;
    hsbus_uart.Init.BaudRate = crsf ? 420000U : 100000U;
    hsbus_uart.Init.WordLength = crsf ? UART_WORDLENGTH_8B : UART_WORDLENGTH_9B;
    hsbus_uart.Init.StopBits = crsf ? UART_STOPBITS_1 : UART_STOPBITS_2;
    hsbus_uart.Init.Parity = crsf ? UART_PARITY_NONE : UART_PARITY_EVEN;
    hsbus_uart.Init.Mode = UART_MODE_RX;
    hsbus_uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    hsbus_uart.Init.OverSampling = UART_OVERSAMPLING_16;
    hsbus_uart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    if (!crsf) {
        hsbus_uart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXINVERT_INIT;
        hsbus_uart.AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;
    }
    if (HAL_UART_Init(&hsbus_uart) != HAL_OK) return false;
    HAL_NVIC_SetPriority(irq, 5U, 0U);
    HAL_NVIC_EnableIRQ(irq);
    return true;
#else
    (void)port;
    if (hsbus_uart.Instance != NULL) {
        HAL_NVIC_DisableIRQ(hsbus_uart.Instance == CRSF_UART_INSTANCE
                            ? CRSF_UART_IRQn : SBUS_UART_IRQn);
        (void)HAL_UART_DeInit(&hsbus_uart);
    }
    hsbus_uart = (UART_HandleTypeDef){0};
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
#if defined(BOARD_SEQUREH7V2) || defined(BOARD_FOXEERH743)
    hsbus_uart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXINVERT_INIT;
    hsbus_uart.AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;
#endif
    if (HAL_UART_Init(&hsbus_uart) != HAL_OK) return false;
    HAL_NVIC_SetPriority(SBUS_UART_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(SBUS_UART_IRQn);
    return true;
#endif
}

#endif

static void battery_adc_init(void)
{
#if defined(BOARD_SEQUREH7V2) || defined(BOARD_FOXEERH743)
    /* H743 LQFP100 exposes PC2_C/PC3_C, directly connected to ADC3
     * INP0/INP1. ADC1 INP12/INP13 are different internal inputs. */
    __HAL_RCC_ADC3_CLK_ENABLE();
#else
    __HAL_RCC_ADC12_CLK_ENABLE();
#endif
    GPIO_InitTypeDef gpio = {
        .Pin = BATTERY_ADC_PIN,
        .Mode = GPIO_MODE_ANALOG,
        .Pull = GPIO_NOPULL,
    };
    HAL_GPIO_Init(BATTERY_ADC_PORT, &gpio);
#if BOARD_HAS_CURRENT
    gpio.Pin = CURRENT_ADC_PIN;
    HAL_GPIO_Init(CURRENT_ADC_PORT, &gpio);
#endif
#if defined(BOARD_SEQUREH7V2) || defined(BOARD_FOXEERH743)
    hadc_battery.Instance = ADC3;
#else
    hadc_battery.Instance = ADC1;
#endif
    /* Use synchronous H743 ADC clocking.  The previous asynchronous mode
     * had no RCC ADC source configured, so calibration ran without a clock
     * and eventually timed out. */
    hadc_battery.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc_battery.Init.Resolution = ADC_RESOLUTION_12B;
    hadc_battery.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc_battery.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc_battery.Init.LowPowerAutoWait = DISABLE;
    hadc_battery.Init.ContinuousConvMode = DISABLE;
    hadc_battery.Init.NbrOfConversion = 1U;
    hadc_battery.Init.DiscontinuousConvMode = DISABLE;
    hadc_battery.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc_battery.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc_battery.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
    hadc_battery.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    hadc_battery.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
    hadc_battery.Init.OversamplingMode = DISABLE;
    if (HAL_ADC_Init(&hadc_battery) != HAL_OK) board_fatal_error();
    if (HAL_ADCEx_Calibration_Start(&hadc_battery, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
        board_fatal_error();
    ADC_ChannelConfTypeDef channel = {0};
    channel.Channel = BATTERY_ADC_CHANNEL;
    channel.Rank = ADC_REGULAR_RANK_1;
    channel.SamplingTime = ADC_SAMPLETIME_387CYCLES_5;
    channel.SingleDiff = ADC_SINGLE_ENDED;
    channel.OffsetNumber = ADC_OFFSET_NONE;
    channel.Offset = 0U;
    if (HAL_ADC_ConfigChannel(&hadc_battery, &channel) != HAL_OK) board_fatal_error();
}

void board_init(void)
{
    HAL_Init();
    clock_init();
    gpio_init();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void board_peripherals_init(void)
{
    spi_init();
    if (!board_receiver_uart_configure(true, 1U)) board_fatal_error();
    battery_adc_init();
}

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
        channel.Rank = ADC_REGULAR_RANK_1;
        channel.SamplingTime = ADC_SAMPLETIME_387CYCLES_5;
        channel.SingleDiff = ADC_SINGLE_ENDED;
        channel.OffsetNumber = ADC_OFFSET_NONE;
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
#if defined(BOARD_SEQUREH7V2)
void board_current_adc_diagnostics(board_current_adc_diagnostics_t *d)
{
    d->raw = current_adc_raw;
    d->samples = current_adc_count;
    d->age_ms = current_adc_count ?
        (uint32_t)(board_micros() - current_adc_last_us) / 1000U : UINT32_MAX;
    d->errors = battery_adc_errors;
}
#endif
float board_battery_current(void)
{
#if BOARD_HAS_CURRENT
    return battery_current_filtered;
#else
    return 0.0f;
#endif
}

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

    /* This path runs immediately after NVIC_SystemReset(), before board_init()
     * changes clocks or starts peripherals.  Leave PRIMASK clear: the STM32H7
     * ROM USB DFU bootloader needs interrupts in order to enumerate. */
    SCB->VTOR = SYSTEM_MEMORY_ADDRESS;
    __DSB();
    __ISB();
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
