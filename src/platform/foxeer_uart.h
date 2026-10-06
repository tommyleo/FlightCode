#pragma once

/* Shared STM32F7/H7 Foxeer UART routing. Keep RX, TX and reported availability
 * together so receiver/VTX selection cannot claim a nonexistent pad. */
typedef struct {
    USART_TypeDef *instance;
    GPIO_TypeDef *rx_port, *tx_port;
    uint16_t rx_pin, tx_pin;
    uint32_t af;
    IRQn_Type irq;
} foxeer_uart_t;

static bool foxeer_uart(uint8_t port, foxeer_uart_t *u)
{
    switch (port) {
    case 1U:
#if defined(BOARD_FOXEERF722V4)
        *u = (foxeer_uart_t){USART1, GPIOB, GPIOB, GPIO_PIN_7, GPIO_PIN_6,
                             GPIO_AF7_USART1, USART1_IRQn};
#else
        *u = (foxeer_uart_t){USART1, GPIOA, GPIOA, GPIO_PIN_10, GPIO_PIN_9,
                             GPIO_AF7_USART1, USART1_IRQn};
#endif
        __HAL_RCC_USART1_CLK_ENABLE(); break;
    case 2U:
        *u = (foxeer_uart_t){USART2, GPIOA, GPIOA, GPIO_PIN_3, GPIO_PIN_2,
                             GPIO_AF7_USART2, USART2_IRQn};
        __HAL_RCC_USART2_CLK_ENABLE(); break;
    case 3U:
        *u = (foxeer_uart_t){USART3, GPIOB, GPIOB, GPIO_PIN_11, GPIO_PIN_10,
                             GPIO_AF7_USART3, USART3_IRQn};
        __HAL_RCC_USART3_CLK_ENABLE(); break;
    case 4U:
        *u = (foxeer_uart_t){UART4, GPIOA, GPIOA, GPIO_PIN_1, GPIO_PIN_0,
                             GPIO_AF8_UART4, UART4_IRQn};
        __HAL_RCC_UART4_CLK_ENABLE(); break;
#if defined(BOARD_FOXEERF722V4)
    case 5U:
        *u = (foxeer_uart_t){UART5, GPIOD, GPIOC, GPIO_PIN_2, GPIO_PIN_12,
                             GPIO_AF8_UART5, UART5_IRQn};
        __HAL_RCC_UART5_CLK_ENABLE(); break;
#endif
    case 6U:
        *u = (foxeer_uart_t){USART6, GPIOC, GPIOC, GPIO_PIN_7, GPIO_PIN_6,
#if defined(PLATFORM_STM32F7)
                             GPIO_AF8_USART6, USART6_IRQn};
#else
                             GPIO_AF7_USART6, USART6_IRQn};
#endif
        __HAL_RCC_USART6_CLK_ENABLE(); break;
#if defined(BOARD_FOXEERH743)
    case 7U:
        *u = (foxeer_uart_t){UART7, GPIOE, GPIOE, GPIO_PIN_7, GPIO_PIN_8,
                             GPIO_AF7_UART7, UART7_IRQn};
        __HAL_RCC_UART7_CLK_ENABLE(); break;
    case 8U:
        *u = (foxeer_uart_t){UART8, GPIOE, GPIOE, GPIO_PIN_0, GPIO_PIN_1,
                             GPIO_AF8_UART8, UART8_IRQn};
        __HAL_RCC_UART8_CLK_ENABLE(); break;
#endif
    default: return false;
    }
    return true;
}

static void foxeer_uart_settings(UART_HandleTypeDef *handle, USART_TypeDef *instance,
                                 uint32_t baud, uint32_t mode, uint32_t stop)
{
    *handle = (UART_HandleTypeDef){0};
    handle->Instance = instance;
    handle->Init.BaudRate = baud;
    handle->Init.WordLength = UART_WORDLENGTH_8B;
    handle->Init.StopBits = stop;
    handle->Init.Parity = UART_PARITY_NONE;
    handle->Init.Mode = mode;
    handle->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    handle->Init.OverSampling = UART_OVERSAMPLING_16;
    handle->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
}

bool board_uart_half_duplex_init(uint8_t port, uint32_t baud, uint32_t stop,
                                 UART_HandleTypeDef *handle)
{
    foxeer_uart_t u;
    if (!foxeer_uart(port, &u)) return false;
    GPIO_InitTypeDef gpio = {.Pin=u.tx_pin, .Mode=GPIO_MODE_AF_OD,
        .Pull=GPIO_PULLUP, .Speed=GPIO_SPEED_FREQ_VERY_HIGH, .Alternate=u.af};
    HAL_GPIO_Init(u.tx_port, &gpio);
    foxeer_uart_settings(handle, u.instance, baud, UART_MODE_TX_RX, stop);
    return HAL_HalfDuplex_Init(handle) == HAL_OK;
}

bool board_uart_tx_init(uint8_t port, uint32_t baud, UART_HandleTypeDef *handle)
{
    foxeer_uart_t u;
    if (!foxeer_uart(port, &u)) return false;
    GPIO_InitTypeDef gpio = {.Pin=u.tx_pin, .Mode=GPIO_MODE_AF_PP,
        .Pull=GPIO_NOPULL, .Speed=GPIO_SPEED_FREQ_VERY_HIGH, .Alternate=u.af};
    HAL_GPIO_Init(u.tx_port, &gpio);
    foxeer_uart_settings(handle, u.instance, baud, UART_MODE_TX, UART_STOPBITS_1);
    return HAL_UART_Init(handle) == HAL_OK;
}

bool board_uart_tx_rx_init(uint8_t port, uint32_t baud, UART_HandleTypeDef *handle)
{
    foxeer_uart_t u;
    if (!foxeer_uart(port, &u)) return false;
    GPIO_InitTypeDef gpio = {.Pin=u.tx_pin, .Mode=GPIO_MODE_AF_PP,
        .Pull=GPIO_NOPULL, .Speed=GPIO_SPEED_FREQ_VERY_HIGH, .Alternate=u.af};
    HAL_GPIO_Init(u.tx_port, &gpio);
    gpio.Pin=u.rx_pin;
    HAL_GPIO_Init(u.rx_port, &gpio);
    foxeer_uart_settings(handle, u.instance, baud, UART_MODE_TX_RX, UART_STOPBITS_1);
    return HAL_UART_Init(handle) == HAL_OK;
}

bool board_receiver_uart_configure(bool crsf, uint8_t port)
{
    foxeer_uart_t u;
    if (!foxeer_uart(port, &u)) return false;
    if (hsbus_uart.Instance != NULL) {
        for (uint8_t p=1U; p<=8U; ++p) {
            foxeer_uart_t old;
            if (foxeer_uart(p, &old) && old.instance == hsbus_uart.Instance) {
                HAL_NVIC_DisableIRQ(old.irq);
                HAL_NVIC_ClearPendingIRQ(old.irq);
                break;
            }
        }
        (void)HAL_UART_DeInit(&hsbus_uart);
    }
    GPIO_InitTypeDef gpio = {.Pin=u.rx_pin, .Mode=GPIO_MODE_AF_PP,
        .Pull=GPIO_NOPULL, .Speed=GPIO_SPEED_FREQ_VERY_HIGH, .Alternate=u.af};
    HAL_GPIO_Init(u.rx_port, &gpio);
    foxeer_uart_settings(&hsbus_uart, u.instance, crsf ? 420000U : 100000U,
                         UART_MODE_RX, crsf ? UART_STOPBITS_1 : UART_STOPBITS_2);
    if (!crsf) {
        hsbus_uart.Init.WordLength = UART_WORDLENGTH_9B;
        hsbus_uart.Init.Parity = UART_PARITY_EVEN;
        hsbus_uart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXINVERT_INIT;
        hsbus_uart.AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;
    }
    if (HAL_UART_Init(&hsbus_uart) != HAL_OK) return false;
    HAL_NVIC_SetPriority(u.irq, 5U, 0U);
    HAL_NVIC_EnableIRQ(u.irq);
    return true;
}

#include "receiver_uart_tx.h"

bool board_receiver_uart_transmit(const uint8_t *bytes, uint16_t length)
{
    for (uint8_t p=1U; p<=8U; ++p) {
        foxeer_uart_t u;
        if (foxeer_uart(p, &u) && u.instance == hsbus_uart.Instance)
            return receiver_uart_transmit(u.tx_port, u.tx_pin, u.af,
                                          bytes, length);
    }
    return false;
}
