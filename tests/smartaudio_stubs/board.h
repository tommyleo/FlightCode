#pragma once
#include <stdbool.h>
#include <stdint.h>
typedef struct { uint32_t DR, TDR, RDR; } USART_TypeDef;
typedef struct { USART_TypeDef *Instance; } UART_HandleTypeDef;
#define UART_STOPBITS_2 2U
#define UART_FLAG_TXE 1U
#define UART_FLAG_TC 2U
#define UART_FLAG_RXNE 3U
#define UART_FLAG_ORE 4U
#define UART_FLAG_FE 5U
#define UART_FLAG_NE 6U
#define RESET 0U
#define HAL_OK 0U
uint32_t HAL_GetTick(void);
bool board_uart_half_duplex_init(uint8_t, uint32_t, uint32_t, UART_HandleTypeDef *);
int HAL_HalfDuplex_EnableTransmitter(UART_HandleTypeDef *);
int HAL_HalfDuplex_EnableReceiver(UART_HandleTypeDef *);
unsigned test_uart_flag(unsigned flag);
#define __HAL_UART_GET_FLAG(handle, flag) test_uart_flag(flag)
#define __HAL_UART_CLEAR_OREFLAG(handle) ((void)(handle))
#define __HAL_UART_CLEAR_FEFLAG(handle) ((void)(handle))
#define __HAL_UART_CLEAR_NEFLAG(handle) ((void)(handle))
#define __HAL_UART_FLUSH_DRREGISTER(handle) ((void)(handle))
