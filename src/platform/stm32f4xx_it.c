#include "board.h"

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern UART_HandleTypeDef hsbus_uart;
extern DMA_HandleTypeDef hdma_spi2_tx;

void SysTick_Handler(void)
{
    HAL_IncTick();
}

void HardFault_Handler(void)
{
    while (1) {
    }
}

void OTG_FS_IRQHandler(void)
{
    HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
}

void SBUS_UART_IRQ_HANDLER(void)
{
    HAL_UART_IRQHandler(&hsbus_uart);
}

#if defined(BOARD_SEQUREH7V2)
void USART2_IRQHandler(void) { HAL_UART_IRQHandler(&hsbus_uart); }
void UART4_IRQHandler(void) { HAL_UART_IRQHandler(&hsbus_uart); }
void USART6_IRQHandler(void) { HAL_UART_IRQHandler(&hsbus_uart); }
void UART7_IRQHandler(void) { HAL_UART_IRQHandler(&hsbus_uart); }
void UART8_IRQHandler(void) { HAL_UART_IRQHandler(&hsbus_uart); }
#endif

#if BOARD_HAS_CRSF && !defined(BOARD_RECEIVER_UART_SHARED)
void CRSF_UART_IRQ_HANDLER(void)
{
    HAL_UART_IRQHandler(&hsbus_uart);
}
#endif

void DMA1_Stream4_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_spi2_tx);
}
