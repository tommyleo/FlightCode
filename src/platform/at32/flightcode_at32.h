#pragma once
/* Small API adapter for shared FlightCode drivers; all hardware operations
 * below use Artery peripherals, never STM32 register definitions. */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "at32f435_437.h"
#define SystemCoreClock system_core_clock
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef gpio_type GPIO_TypeDef;
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
#define GPIO_PIN_0 GPIO_PINS_0
#define GPIO_PIN_1 GPIO_PINS_1
#define GPIO_PIN_2 GPIO_PINS_2
#define GPIO_PIN_3 GPIO_PINS_3
#define GPIO_PIN_4 GPIO_PINS_4
#define GPIO_PIN_5 GPIO_PINS_5
#define GPIO_PIN_6 GPIO_PINS_6
#define GPIO_PIN_7 GPIO_PINS_7
#define GPIO_PIN_8 GPIO_PINS_8
#define GPIO_PIN_9 GPIO_PINS_9
#define GPIO_PIN_10 GPIO_PINS_10
#define GPIO_PIN_11 GPIO_PINS_11
#define GPIO_PIN_12 GPIO_PINS_12
#define GPIO_PIN_13 GPIO_PINS_13
#define GPIO_PIN_14 GPIO_PINS_14
#define GPIO_PIN_15 GPIO_PINS_15
typedef struct {
    spi_type *Instance;
    struct { uint32_t BaudRatePrescaler, CLKPolarity, CLKPhase; } Init;
} SPI_HandleTypeDef;
#define SPI_BAUDRATEPRESCALER_8 SPI_MCLK_DIV_8
#define SPI_BAUDRATEPRESCALER_16 SPI_MCLK_DIV_16
#define SPI_BAUDRATEPRESCALER_32 SPI_MCLK_DIV_32
#define SPI_BAUDRATEPRESCALER_64 SPI_MCLK_DIV_64
#define SPI_BAUDRATEPRESCALER_128 SPI_MCLK_DIV_128
#define SPI_POLARITY_LOW SPI_CLOCK_POLARITY_LOW
#define SPI_POLARITY_HIGH SPI_CLOCK_POLARITY_HIGH
#define SPI_PHASE_1EDGE SPI_CLOCK_PHASE_1EDGE
#define SPI_PHASE_2EDGE SPI_CLOCK_PHASE_2EDGE
typedef struct {
    usart_type *Instance;
    uint8_t *rx;
    uint16_t rxRemaining;
} UART_HandleTypeDef;
/* This target has no SD-card DMA handle. Kept for board.h's shared declaration. */
typedef struct { uint8_t unused; } DMA_HandleTypeDef;
#define UART_STOPBITS_1 USART_STOP_1_BIT
#define UART_STOPBITS_2 USART_STOP_2_BIT
#define UART_FLAG_TXE USART_TDBE_FLAG
#define UART_FLAG_TC USART_TDC_FLAG
#define UART_FLAG_RXNE USART_RDBF_FLAG
#define UART_FLAG_ORE USART_ROERR_FLAG
#define UART_FLAG_FE USART_FERR_FLAG
#define UART_FLAG_NE USART_NERR_FLAG
#define __HAL_UART_GET_FLAG(h, f) usart_flag_get((h)->Instance, (f))
void at32_uart_clear_errors(UART_HandleTypeDef *h);
#define __HAL_UART_CLEAR_PEFLAG(h) at32_uart_clear_errors(h)
#define __HAL_UART_CLEAR_FEFLAG(h) at32_uart_clear_errors(h)
#define __HAL_UART_CLEAR_NEFLAG(h) at32_uart_clear_errors(h)
#define __HAL_UART_CLEAR_OREFLAG(h) at32_uart_clear_errors(h)
#define __HAL_UART_FLUSH_DRREGISTER(h) at32_uart_clear_errors(h)
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pins, GPIO_PinState level);
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *h);
HAL_StatusTypeDef HAL_SPI_DeInit(SPI_HandleTypeDef *h);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *h, const uint8_t *tx,
                                         uint8_t *rx, uint16_t n, uint32_t ms);
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *h, const uint8_t *tx, uint16_t n, uint32_t ms);
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *h, uint8_t *rx, uint16_t n, uint32_t ms);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *h, uint8_t *rx, uint16_t n);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *h);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h, const uint8_t *tx, uint16_t n, uint32_t ms);
HAL_StatusTypeDef HAL_HalfDuplex_EnableReceiver(UART_HandleTypeDef *h);
HAL_StatusTypeDef HAL_HalfDuplex_EnableTransmitter(UART_HandleTypeDef *h);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *h);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *h);
void at32_uart_irq(UART_HandleTypeDef *h);
/* The shared settings code passes the address of a reserved 2 KiB AT32 sector. */
typedef struct { uint32_t TypeErase, Sector, NbSectors, VoltageRange; } FLASH_EraseInitTypeDef;
#define FLASH_TYPEERASE_SECTORS 0U
#define FLASH_TYPEPROGRAM_WORD 0U
#define FLASH_VOLTAGE_RANGE_3 0U
void HAL_FLASH_Unlock(void);
void HAL_FLASH_Lock(void);
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *e, uint32_t *error);
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t type, uint32_t address, uint32_t data);
void at32_gpio_config(gpio_type *port, uint16_t pins, gpio_mode_type mode,
                      gpio_pull_type pull, gpio_mux_sel_type mux);
