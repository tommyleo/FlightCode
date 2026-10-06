#pragma once

/* Enable only the receiver TX pin while sending a command. RX and its
 * interrupt remain active; the TX pin returns to input afterwards. */
static bool receiver_uart_transmit(GPIO_TypeDef *port, uint16_t pin,
                                  uint32_t af, const uint8_t *bytes,
                                  uint16_t length)
{
    GPIO_InitTypeDef gpio = {
        .Pin = pin, .Mode = GPIO_MODE_AF_PP, .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH, .Alternate = af,
    };
    HAL_GPIO_Init(port, &gpio);
    SET_BIT(hsbus_uart.Instance->CR1, USART_CR1_TE);
    const bool sent = HAL_UART_Transmit(&hsbus_uart, (uint8_t *)bytes,
                                       length, 5U) == HAL_OK;
    CLEAR_BIT(hsbus_uart.Instance->CR1, USART_CR1_TE);
    gpio.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(port, &gpio);
    return sent;
}
