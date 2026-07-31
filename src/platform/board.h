#pragma once

#include <stdbool.h>

#include "stm32f4xx_hal.h"

#if defined(BOARD_MAMBAF411)
#define BOARD_NAME "MAMBAF411"
// Official Betaflight DIAT/MAMBAF411 target mapping.
#define MOTOR_1_PORT GPIOB
#define MOTOR_1_PIN GPIO_PIN_3
#define MOTOR_2_PORT GPIOB
#define MOTOR_2_PIN GPIO_PIN_4
#define MOTOR_3_PORT GPIOB
#define MOTOR_3_PIN GPIO_PIN_6
#define MOTOR_4_PORT GPIOB
#define MOTOR_4_PIN GPIO_PIN_7

#define STATUS_LED_PORT GPIOC
#define STATUS_LED_PIN GPIO_PIN_13

#define BUZZER_PORT GPIOB
#define BUZZER_PIN GPIO_PIN_2
#define BUZZER_ACTIVE_LEVEL GPIO_PIN_RESET

#define SBUS_INVERTER_PORT GPIOB
#define SBUS_INVERTER_PIN GPIO_PIN_10
#define SBUS_INVERTER_ENABLE_LEVEL GPIO_PIN_RESET

#define SETTINGS_ADDRESS 0x08060000U
#define SETTINGS_FLASH_SECTOR FLASH_SECTOR_7
#define FLIGHT_LOG_ADDRESS 0x08040000U
#define FLIGHT_LOG_FLASH_SECTOR FLASH_SECTOR_6
#define SBUS_UART_INSTANCE USART1
#define SBUS_UART_IRQn USART1_IRQn

#define MPU6000_CS_PORT GPIOA
#define MPU6000_CS_PIN GPIO_PIN_4
#define BOARD_CORE_CLOCK_HZ 96000000U

#elif defined(BOARD_CLRACINGF4)
#define BOARD_NAME "CLRACINGF4"
// Official Betaflight CLRA/CLRACINGF4 target mapping.
#define MOTOR_1_PORT GPIOB
#define MOTOR_1_PIN GPIO_PIN_0
#define MOTOR_2_PORT GPIOB
#define MOTOR_2_PIN GPIO_PIN_1
#define MOTOR_3_PORT GPIOA
#define MOTOR_3_PIN GPIO_PIN_3
#define MOTOR_4_PORT GPIOA
#define MOTOR_4_PIN GPIO_PIN_2

#define STATUS_LED_PORT GPIOB
#define STATUS_LED_PIN GPIO_PIN_5

#define BUZZER_PORT GPIOB
#define BUZZER_PIN GPIO_PIN_4
#define BUZZER_ACTIVE_LEVEL GPIO_PIN_RESET

#define SBUS_INVERTER_PORT GPIOC
#define SBUS_INVERTER_PIN GPIO_PIN_0
#define SBUS_INVERTER_ENABLE_LEVEL GPIO_PIN_RESET

#define SETTINGS_ADDRESS 0x080E0000U
#define SETTINGS_FLASH_SECTOR FLASH_SECTOR_11
#define FLIGHT_LOG_ADDRESS 0x080C0000U
#define FLIGHT_LOG_FLASH_SECTOR FLASH_SECTOR_10
#define SBUS_UART_INSTANCE USART1
#define SBUS_UART_IRQn USART1_IRQn

#define MPU6000_CS_PORT GPIOA
#define MPU6000_CS_PIN GPIO_PIN_4
#define BOARD_CORE_CLOCK_HZ 168000000U
#else
#error "No supported board selected"
#endif

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef hsbus_uart;

void board_init(void);
void board_fatal_error(void);
uint32_t board_micros(void);
void board_buzzer_set(bool enabled);
void board_check_dfu_request(void);
void board_enter_dfu(void) __attribute__((noreturn));
