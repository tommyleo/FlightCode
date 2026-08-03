#include "stm32f4xx_hal.h"

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

void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&hsbus_uart);
}

void DMA1_Stream4_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_spi2_tx);
}
