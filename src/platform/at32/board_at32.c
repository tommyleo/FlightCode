#include "board.h"
#include "usb_cdc.h"

SPI_HandleTypeDef hspi1, hspi2, hspi3, hspi4;
DMA_HandleTypeDef hdma_spi2_tx;
UART_HandleTypeDef hsbus_uart;
static float voltage, current, voltage_multiplier = 1.0f;
static uint8_t adc_channel;
static bool adc_ready;
#define DFU_REQUEST_ADDRESS 0x2001FFF0U
#define DFU_REQUEST_MAGIC 0x41544446U
#define SYSTEM_MEMORY_ADDRESS 0x1FFF0000U

static void clock_init(void)
{
    crm_reset();
    crm_periph_clock_enable(CRM_PWC_PERIPH_CLOCK, TRUE);
    pwc_ldo_output_voltage_set(PWC_LDO_OUTPUT_1V3);
    flash_clock_divider_set(FLASH_CLOCK_DIV_3);
    crm_clock_source_enable(CRM_CLOCK_SOURCE_HEXT, TRUE);
    if (crm_hext_stable_wait() == ERROR) board_fatal_error();
    crm_pll_config(CRM_PLL_SOURCE_HEXT, 144, 1, CRM_PLL_FR_4);
    crm_clock_source_enable(CRM_CLOCK_SOURCE_PLL, TRUE);
    uint32_t tries = 2000000U;
    while (crm_flag_get(CRM_PLL_STABLE_FLAG) == RESET)
        if (--tries == 0U) board_fatal_error();
    crm_ahb_div_set(CRM_AHB_DIV_1);
    crm_apb1_div_set(CRM_APB1_DIV_2);
    crm_apb2_div_set(CRM_APB2_DIV_2);
    crm_auto_step_mode_enable(TRUE);
    crm_sysclk_switch(CRM_SCLK_PLL);
    tries = 2000000U;
    while (crm_sysclk_switch_status_get() != CRM_SCLK_PLL)
        if (--tries == 0U) board_fatal_error();
    crm_auto_step_mode_enable(FALSE);
    system_core_clock_update();
    crm_usb_clock_source_select(CRM_USB_CLOCK_SOURCE_PLL);
    crm_usb_clock_div_set(CRM_USB_DIV_6);
}

static void spi_bus_init(SPI_HandleTypeDef *h, spi_type *instance,
                         uint32_t prescaler, uint32_t polarity, uint32_t phase)
{
    h->Instance = instance;
    h->Init.BaudRatePrescaler = prescaler;
    h->Init.CLKPolarity = polarity;
    h->Init.CLKPhase = phase;
    HAL_SPI_Init(h);
}

static void adc_init(void)
{
    crm_periph_clock_enable(CRM_ADC1_PERIPH_CLOCK, TRUE);
    at32_gpio_config(GPIOA, GPIO_PIN_2 | GPIO_PIN_3, GPIO_MODE_ANALOG, GPIO_PULL_NONE, GPIO_MUX_0);
    adc_common_config_type common;
    adc_common_default_para_init(&common);
    common.div = ADC_HCLK_DIV_8;
    adc_common_config(&common);
    adc_base_config_type base;
    adc_base_default_para_init(&base);
    base.ordinary_channel_length = 1;
    adc_base_config(ADC1, &base);
    adc_ordinary_conversion_trigger_set(ADC1, ADC_ORDINARY_TRIG_TMR1CH1, ADC_ORDINARY_TRIG_EDGE_NONE);
    adc_enable(ADC1, TRUE);
    uint32_t start = board_micros();
    while (adc_flag_get(ADC1, ADC_RDY_FLAG) == RESET)
        if ((uint32_t)(board_micros() - start) > 10000U) return;
    adc_calibration_init(ADC1);
    start = board_micros();
    while (adc_calibration_init_status_get(ADC1))
        if ((uint32_t)(board_micros() - start) > 10000U) return;
    adc_calibration_start(ADC1);
    start = board_micros();
    while (adc_calibration_status_get(ADC1))
        if ((uint32_t)(board_micros() - start) > 10000U) return;
    adc_ordinary_channel_set(ADC1, ADC_CHANNEL_2, 1, ADC_SAMPLETIME_92_5);
    adc_ordinary_software_trigger_enable(ADC1, TRUE);
    adc_ready = true;
}

void board_init(void)
{
    clock_init();
    NVIC_SetPriorityGrouping(3U);
    SCB->VTOR = FLASH_BASE;
    __DSB(); __ISB();
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_TMR5_PERIPH_CLOCK, TRUE);
    tmr_32_bit_function_enable(TMR5, TRUE);
    tmr_base_init(TMR5, UINT32_MAX, BOARD_CORE_CLOCK_HZ / 1000000U - 1U);
    tmr_counter_enable(TMR5, TRUE);
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    SysTick_Config(SystemCoreClock / 1000U);
    NVIC_SetPriority(SysTick_IRQn, 15U);

    /* Set chip selects high before selecting output mode. */
    gpio_bits_set(GPIOA, GPIO_PIN_4 | GPIO_PIN_8);
    gpio_bits_set(GPIOC, GPIO_PIN_14 | GPIO_PIN_15);
    at32_gpio_config(GPIOA, GPIO_PIN_4 | GPIO_PIN_8, GPIO_MODE_OUTPUT, GPIO_PULL_NONE, GPIO_MUX_0);
    at32_gpio_config(GPIOC, GPIO_PIN_14 | GPIO_PIN_15, GPIO_MODE_OUTPUT, GPIO_PULL_NONE, GPIO_MUX_0);
    /* PINIO config 129 is push-pull inverted: low enables the 10V VTX rail. */
    gpio_bits_reset(GPIOC, GPIO_PIN_13);
    at32_gpio_config(GPIOC, GPIO_PIN_13, GPIO_MODE_OUTPUT, GPIO_PULL_NONE, GPIO_MUX_0);
    /* The ICM runs from its internal clock; leave optional CLKIN undriven. */
    at32_gpio_config(GPIOB, GPIO_PIN_11 | GPIO_PIN_12, GPIO_MODE_INPUT, GPIO_PULL_DOWN, GPIO_MUX_0);

    crm_periph_clock_enable(CRM_SPI1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_SPI2_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_SPI4_PERIPH_CLOCK, TRUE);
    at32_gpio_config(GPIOA, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_MODE_MUX, GPIO_PULL_NONE, GPIO_MUX_5);
    at32_gpio_config(GPIOB, GPIO_PIN_10 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_MODE_MUX, GPIO_PULL_NONE, GPIO_MUX_5);
    at32_gpio_config(GPIOB, GPIO_PIN_13 | GPIO_PIN_8 | GPIO_PIN_9, GPIO_MODE_MUX, GPIO_PULL_NONE, GPIO_MUX_6);
    spi_bus_init(&hspi1, SPI1, SPI_BAUDRATEPRESCALER_128, SPI_POLARITY_HIGH, SPI_PHASE_2EDGE);
    spi_bus_init(&hspi2, SPI2, SPI_BAUDRATEPRESCALER_8, SPI_POLARITY_LOW, SPI_PHASE_1EDGE);
    spi_bus_init(&hspi4, SPI4, SPI_BAUDRATEPRESCALER_64, SPI_POLARITY_LOW, SPI_PHASE_1EDGE);
    adc_init();
}

uint32_t board_micros(void) { return tmr_counter_value_get(TMR5); }
void board_status_led_set(bool on)
{ HAL_GPIO_WritePin(STATUS_LED_PORT, STATUS_LED_PIN, on ? STATUS_LED_ACTIVE_LEVEL : GPIO_PIN_SET); }
void board_status_led_update(bool signal, bool calibrating)
{
    const uint32_t t = board_micros();
    board_status_led_set(calibrating ? (t / 100000U) % 2U == 0U :
                        signal ? true : t % 1000000U < 80000U);
}
void board_buzzer_set(bool on) { (void)on; }
void board_buzzer_update(bool on) { (void)on; }
void board_battery_set_multiplier(float m) { voltage_multiplier = m; }
float board_battery_voltage(void) { return voltage * voltage_multiplier; }
float board_battery_current(void) { return current; }
void board_battery_update(void)
{
    if (!adc_ready || adc_flag_get(ADC1, ADC_OCCE_FLAG) == RESET) return;
    const float pin_volts = adc_ordinary_conversion_data_get(ADC1) * (3.3f / 4095.0f);
    adc_flag_clear(ADC1, ADC_OCCE_FLAG);
    if (adc_channel == 0) {
        const float v = pin_volts * BATTERY_VOLTAGE_DIVIDER;
        voltage = voltage == 0 ? v : voltage + 0.05f * (v - voltage);
    } else {
        /* Betaflight current scale is expressed in 0.1 mV per ampere. */
        const float a = pin_volts * 10000.0f / CURRENT_METER_SCALE;
        current += 0.05f * (a - current);
    }
    adc_channel ^= 1U;
    adc_ordinary_channel_set(ADC1, adc_channel ? ADC_CHANNEL_3 : ADC_CHANNEL_2, 1, ADC_SAMPLETIME_92_5);
    adc_ordinary_software_trigger_enable(ADC1, TRUE);
}

bool board_receiver_uart_configure(bool crsf, uint8_t port)
{
    /* UART1 is physically wired to the built-in receiver. No SBUS inverter. */
    if (!crsf || (port != 0 && port != 1)) return false;
    NVIC_DisableIRQ(USART1_IRQn);
    crm_periph_clock_enable(CRM_USART1_PERIPH_CLOCK, TRUE);
    usart_enable(USART1, FALSE);
    at32_gpio_config(GPIOB, GPIO_PIN_7, GPIO_MODE_MUX, GPIO_PULL_UP, GPIO_MUX_7);
    /* PA15 is enabled temporarily when sending a CRSF bind command. */
    at32_gpio_config(GPIOA, GPIO_PIN_15, GPIO_MODE_INPUT, GPIO_PULL_NONE, GPIO_MUX_0);
    hsbus_uart.Instance = USART1;
    hsbus_uart.rxRemaining = 0;
    usart_init(USART1, 420000U, USART_DATA_8BITS, USART_STOP_1_BIT);
    usart_parity_selection_config(USART1, USART_PARITY_NONE);
    usart_receiver_enable(USART1, TRUE);
    usart_transmitter_enable(USART1, FALSE);
    usart_enable(USART1, TRUE);
    at32_uart_clear_errors(&hsbus_uart);
    NVIC_SetPriority(USART1_IRQn, 3U);
    NVIC_ClearPendingIRQ(USART1_IRQn);
    NVIC_EnableIRQ(USART1_IRQn);
    return true;
}
bool board_receiver_uart_transmit(const uint8_t *bytes, uint16_t length)
{
    if (hsbus_uart.Instance != USART1) return false;
    at32_gpio_config(GPIOA, GPIO_PIN_15, GPIO_MODE_MUX, GPIO_PULL_NONE, GPIO_MUX_7);
    usart_transmitter_enable(USART1, TRUE);
    const bool sent = HAL_UART_Transmit(&hsbus_uart, bytes, length, 5U) == HAL_OK;
    usart_transmitter_enable(USART1, FALSE);
    at32_gpio_config(GPIOA, GPIO_PIN_15, GPIO_MODE_INPUT, GPIO_PULL_NONE, GPIO_MUX_0);
    return sent;
}

void USART1_IRQHandler(void) { at32_uart_irq(&hsbus_uart); }

static bool uart_tx_config(uint8_t port, uint32_t baud, uint32_t stop,
                            UART_HandleTypeDef *h, bool half_duplex)
{
    if (port == 5) {
        crm_periph_clock_enable(CRM_UART5_PERIPH_CLOCK, TRUE);
        h->Instance = UART5;
        at32_gpio_config(GPIOB, GPIO_PIN_6, GPIO_MODE_MUX, GPIO_PULL_UP, GPIO_MUX_8);
    } else if (port == 7) {
        crm_periph_clock_enable(CRM_UART7_PERIPH_CLOCK, TRUE);
        h->Instance = UART7;
        at32_gpio_config(GPIOB, GPIO_PIN_4, GPIO_MODE_MUX, GPIO_PULL_UP, GPIO_MUX_8);
    } else return false;
    usart_enable(h->Instance, FALSE);
    usart_init(h->Instance, baud, USART_DATA_8BITS, (usart_stop_bit_num_type)stop);
    usart_parity_selection_config(h->Instance, USART_PARITY_NONE);
    usart_single_line_halfduplex_select(h->Instance, half_duplex ? TRUE : FALSE);
    usart_receiver_enable(h->Instance, FALSE);
    usart_transmitter_enable(h->Instance, TRUE);
    usart_enable(h->Instance, TRUE);
    return true;
}
bool board_uart_tx_init(uint8_t port, uint32_t baud, UART_HandleTypeDef *h)
{ return uart_tx_config(port, baud, UART_STOPBITS_1, h, false); }
bool board_uart_tx_rx_init(uint8_t port, uint32_t baud, UART_HandleTypeDef *h)
{
    if (!uart_tx_config(port, baud, UART_STOPBITS_1, h, false)) return false;
    /* UART5: PB6 TX / PB5 RX; UART7: PB4 TX / PB3 RX. */
    at32_gpio_config(GPIOB, port == 5U ? GPIO_PIN_5 : GPIO_PIN_3,
                     GPIO_MODE_MUX, GPIO_PULL_UP, GPIO_MUX_8);
    usart_receiver_enable(h->Instance, TRUE);
    return true;
}
bool board_uart_half_duplex_init(uint8_t port, uint32_t baud, uint32_t stop, UART_HandleTypeDef *h)
{ return uart_tx_config(port, baud, stop, h, true); }
void board_imu_select(uint8_t candidate) { (void)candidate; }
GPIO_TypeDef *board_imu_cs_port(void) { return IMU_CS_PORT; }
uint16_t board_imu_cs_pin(void) { return IMU_CS_PIN; }

void board_check_dfu_request(void)
{
    volatile uint32_t *request = (volatile uint32_t *)DFU_REQUEST_ADDRESS;
    if (*request != DFU_REQUEST_MAGIC) return;
    *request = 0;
    const uint32_t stack = *(volatile uint32_t *)SYSTEM_MEMORY_ADDRESS;
    const uint32_t entry = *(volatile uint32_t *)(SYSTEM_MEMORY_ADDRESS + 4U);
    SCB->VTOR = SYSTEM_MEMORY_ADDRESS;
    __DSB(); __ISB();
    /* Reset path runs before board_init; the ROM needs interrupts enabled. */
    __asm volatile("msr msp, %0\n bx %1" : : "r"(stack), "r"(entry) : "memory");
    while (1) {}
}
void board_enter_dfu(void)
{
    *(volatile uint32_t *)DFU_REQUEST_ADDRESS = DFU_REQUEST_MAGIC;
    __DSB();
    NVIC_SystemReset();
    while (1) {}
}
void board_fatal_error(void)
{
    __disable_irq();
    /* Clock failures can occur before GPIO initialization; leave motors idle. */
    while (1) __NOP();
}
