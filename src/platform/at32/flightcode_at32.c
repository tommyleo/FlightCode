#include "flightcode_at32.h"
#include "board.h"

static volatile uint32_t ticks;
void SysTick_Handler(void) { ++ticks; }
uint32_t HAL_GetTick(void) { return ticks; }
void HAL_Delay(uint32_t ms)
{
    const uint32_t start = ticks;
    while ((uint32_t)(ticks - start) < ms) __NOP();
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pins, GPIO_PinState level)
{
    gpio_bits_write(port, pins, level == GPIO_PIN_SET ? TRUE : FALSE);
}

void at32_gpio_config(gpio_type *port, uint16_t pins, gpio_mode_type mode,
                      gpio_pull_type pull, gpio_mux_sel_type mux)
{
    gpio_init_type cfg;
    gpio_default_para_init(&cfg);
    cfg.gpio_pins = pins;
    cfg.gpio_mode = mode;
    cfg.gpio_pull = pull;
    cfg.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init(port, &cfg);
    if (mode == GPIO_MODE_MUX) {
        for (uint8_t pin = 0; pin < 16; ++pin)
            if (pins & (1U << pin))
                gpio_pin_mux_config(port, (gpio_pins_source_type)pin, mux);
    }
}

HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *h)
{
    spi_init_type cfg;
    spi_enable(h->Instance, FALSE);
    spi_default_para_init(&cfg);
    cfg.master_slave_mode = SPI_MODE_MASTER;
    cfg.transmission_mode = SPI_TRANSMIT_FULL_DUPLEX;
    cfg.mclk_freq_division = (spi_mclk_freq_div_type)h->Init.BaudRatePrescaler;
    cfg.clock_polarity = (spi_clock_polarity_type)h->Init.CLKPolarity;
    cfg.clock_phase = (spi_clock_phase_type)h->Init.CLKPhase;
    cfg.cs_mode_selection = SPI_CS_SOFTWARE_MODE;
    cfg.frame_bit_num = SPI_FRAME_8BIT;
    spi_init(h->Instance, &cfg);
    spi_software_cs_internal_level_set(h->Instance, SPI_SWCS_INTERNAL_LEVEL_HIGHT);
    spi_enable(h->Instance, TRUE);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_DeInit(SPI_HandleTypeDef *h)
{ spi_enable(h->Instance, FALSE); return HAL_OK; }

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *h, const uint8_t *tx,
                                         uint8_t *rx, uint16_t n, uint32_t ms)
{
    const uint32_t start = board_micros();
    const uint32_t timeout = ms * 1000U;
    for (uint16_t i = 0; i < n; ++i) {
        while (spi_i2s_flag_get(h->Instance, SPI_I2S_TDBE_FLAG) == RESET)
            if ((uint32_t)(board_micros() - start) >= timeout) return HAL_TIMEOUT;
        spi_i2s_data_transmit(h->Instance, tx ? tx[i] : 0xFFU);
        while (spi_i2s_flag_get(h->Instance, SPI_I2S_RDBF_FLAG) == RESET)
            if ((uint32_t)(board_micros() - start) >= timeout) return HAL_TIMEOUT;
        const uint8_t byte = (uint8_t)spi_i2s_data_receive(h->Instance);
        if (rx) rx[i] = byte;
    }
    while (spi_i2s_flag_get(h->Instance, SPI_I2S_BF_FLAG) != RESET)
        if ((uint32_t)(board_micros() - start) >= timeout) return HAL_TIMEOUT;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *h, const uint8_t *tx, uint16_t n, uint32_t ms)
{ return HAL_SPI_TransmitReceive(h, tx, NULL, n, ms); }
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *h, uint8_t *rx, uint16_t n, uint32_t ms)
{ return HAL_SPI_TransmitReceive(h, NULL, rx, n, ms); }

void at32_uart_clear_errors(UART_HandleTypeDef *h)
{
    /* AT32 clears PE/FE/NE/ORE by reading STS followed by DT. */
    volatile uint32_t status = h->Instance->sts;
    volatile uint32_t data = h->Instance->dt;
    (void)status; (void)data;
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *h)
{
    usart_interrupt_enable(h->Instance, USART_RDBF_INT, FALSE);
    h->rxRemaining = 0;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *h, uint8_t *rx, uint16_t n)
{
    if (h->rxRemaining) return HAL_BUSY;
    h->rx = rx; h->rxRemaining = n;
    usart_interrupt_enable(h->Instance, USART_RDBF_INT, TRUE);
    return HAL_OK;
}
void at32_uart_irq(UART_HandleTypeDef *h)
{
    const uint32_t status = h->Instance->sts;
    if (status & (USART_PERR_FLAG | USART_FERR_FLAG | USART_NERR_FLAG | USART_ROERR_FLAG)) {
        at32_uart_clear_errors(h);
        HAL_UART_AbortReceive(h);
        HAL_UART_ErrorCallback(h);
        return;
    }
    if ((status & USART_RDBF_FLAG) && h->rxRemaining) {
        *h->rx++ = (uint8_t)usart_data_receive(h->Instance);
        if (--h->rxRemaining == 0) {
            usart_interrupt_enable(h->Instance, USART_RDBF_INT, FALSE);
            HAL_UART_RxCpltCallback(h);
        }
    }
}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h, const uint8_t *tx, uint16_t n, uint32_t ms)
{
    const uint32_t start = HAL_GetTick();
    for (uint16_t i = 0; i < n; ++i) {
        while (usart_flag_get(h->Instance, USART_TDBE_FLAG) == RESET)
            if ((uint32_t)(HAL_GetTick() - start) >= ms) return HAL_TIMEOUT;
        usart_data_transmit(h->Instance, tx[i]);
    }
    while (usart_flag_get(h->Instance, USART_TDC_FLAG) == RESET)
        if ((uint32_t)(HAL_GetTick() - start) >= ms) return HAL_TIMEOUT;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_HalfDuplex_EnableReceiver(UART_HandleTypeDef *h)
{
    usart_transmitter_enable(h->Instance, FALSE);
    usart_receiver_enable(h->Instance, TRUE);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_HalfDuplex_EnableTransmitter(UART_HandleTypeDef *h)
{
    usart_receiver_enable(h->Instance, FALSE);
    usart_transmitter_enable(h->Instance, TRUE);
    return HAL_OK;
}

void HAL_FLASH_Unlock(void) { flash_unlock(); }
void HAL_FLASH_Lock(void) { flash_lock(); }
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *e, uint32_t *error)
{
    /* Never allow the shared storage API to erase the firmware or USD options. */
    if (e->Sector != SETTINGS_ADDRESS || e->NbSectors != 1U) {
        *error = e->Sector; return HAL_ERROR;
    }
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    const flash_status_type result = flash_sector_erase(e->Sector);
    __set_PRIMASK(mask);
    *error = result == FLASH_OPERATE_DONE ? UINT32_MAX : e->Sector;
    return result == FLASH_OPERATE_DONE ? HAL_OK : HAL_ERROR;
}
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t type, uint32_t address, uint32_t data)
{
    (void)type;
    if (address < SETTINGS_ADDRESS || address > SETTINGS_ADDRESS + 2048U - 4U || (address & 3U))
        return HAL_ERROR;
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    const flash_status_type result = flash_word_program(address, data);
    __set_PRIMASK(mask);
    return result == FLASH_OPERATE_DONE && *(volatile uint32_t *)address == data ? HAL_OK : HAL_ERROR;
}
