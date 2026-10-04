#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
typedef unsigned IRQn_Type;
typedef struct {unsigned id;} USART_TypeDef,GPIO_TypeDef;
static USART_TypeDef uart[9];static GPIO_TypeDef gp[5];
#define USART1 (&uart[1])
#define USART2 (&uart[2])
#define USART3 (&uart[3])
#define UART4 (&uart[4])
#define UART5 (&uart[5])
#define USART6 (&uart[6])
#define UART7 (&uart[7])
#define UART8 (&uart[8])
#define GPIOA (&gp[0])
#define GPIOB (&gp[1])
#define GPIOC (&gp[2])
#define GPIOD (&gp[3])
#define GPIOE (&gp[4])
#define GPIO_PIN_0 1U
#define GPIO_PIN_1 2U
#define GPIO_PIN_2 4U
#define GPIO_PIN_3 8U
#define GPIO_PIN_6 64U
#define GPIO_PIN_7 128U
#define GPIO_PIN_8 256U
#define GPIO_PIN_9 512U
#define GPIO_PIN_10 1024U
#define GPIO_PIN_11 2048U
#define GPIO_PIN_12 4096U
#define GPIO_AF7_USART1 7U
#define GPIO_AF7_USART2 7U
#define GPIO_AF7_USART3 7U
#define GPIO_AF8_UART4 8U
#define GPIO_AF8_UART5 8U
#define GPIO_AF7_USART6 7U
#define GPIO_AF8_USART6 8U
#define GPIO_AF7_UART7 7U
#define GPIO_AF8_UART8 8U
#define USART1_IRQn 1U
#define USART2_IRQn 2U
#define USART3_IRQn 3U
#define UART4_IRQn 4U
#define UART5_IRQn 5U
#define USART6_IRQn 6U
#define UART7_IRQn 7U
#define UART8_IRQn 8U
#define __HAL_RCC_USART1_CLK_ENABLE() ((void)0)
#define __HAL_RCC_USART2_CLK_ENABLE() ((void)0)
#define __HAL_RCC_USART3_CLK_ENABLE() ((void)0)
#define __HAL_RCC_UART4_CLK_ENABLE() ((void)0)
#define __HAL_RCC_UART5_CLK_ENABLE() ((void)0)
#define __HAL_RCC_USART6_CLK_ENABLE() ((void)0)
#define __HAL_RCC_UART7_CLK_ENABLE() ((void)0)
#define __HAL_RCC_UART8_CLK_ENABLE() ((void)0)
typedef struct {uint32_t Pin,Mode,Pull,Speed,Alternate;} GPIO_InitTypeDef;
#define GPIO_MODE_AF_OD 1U
#define GPIO_MODE_AF_PP 2U
#define GPIO_PULLUP 1U
#define GPIO_NOPULL 0U
#define GPIO_SPEED_FREQ_VERY_HIGH 3U
typedef struct {USART_TypeDef *Instance;struct {
    uint32_t BaudRate,WordLength,StopBits,Parity,Mode,HwFlowCtl,OverSampling,OneBitSampling;
} Init;struct {uint32_t AdvFeatureInit,RxPinLevelInvert;} AdvancedInit;} UART_HandleTypeDef;
#define UART_WORDLENGTH_8B 8U
#define UART_WORDLENGTH_9B 9U
#define UART_PARITY_NONE 0U
#define UART_PARITY_EVEN 2U
#define UART_MODE_TX_RX 3U
#define UART_MODE_TX 1U
#define UART_MODE_RX 2U
#define UART_HWCONTROL_NONE 0U
#define UART_OVERSAMPLING_16 16U
#define UART_ONE_BIT_SAMPLE_DISABLE 0U
#define UART_STOPBITS_1 1U
#define UART_STOPBITS_2 2U
#define UART_ADVFEATURE_RXINVERT_INIT 1U
#define UART_ADVFEATURE_RXINV_ENABLE 1U
#define HAL_OK 0U
static UART_HandleTypeDef hsbus_uart;
static unsigned enabled_irq,disabled_irq,pad,af;
static GPIO_TypeDef *pad_port;
static void HAL_GPIO_Init(GPIO_TypeDef *p,GPIO_InitTypeDef *g){pad=g->Pin;af=g->Alternate;pad_port=p;}
static unsigned HAL_UART_Init(UART_HandleTypeDef *h){(void)h;return HAL_OK;}
static unsigned HAL_HalfDuplex_Init(UART_HandleTypeDef *h){(void)h;return HAL_OK;}
static unsigned HAL_UART_DeInit(UART_HandleTypeDef *h){(void)h;return HAL_OK;}
static void HAL_NVIC_DisableIRQ(IRQn_Type i){disabled_irq=i;}
static void HAL_NVIC_ClearPendingIRQ(IRQn_Type i){(void)i;}
static void HAL_NVIC_SetPriority(IRQn_Type i,unsigned p,unsigned sub){(void)i;assert(p==5U);(void)sub;}
static void HAL_NVIC_EnableIRQ(IRQn_Type i){enabled_irq=i;}
#include "foxeer_uart.h"

int main(void)
{
    assert(!board_receiver_uart_configure(true,0U));
    assert(!board_receiver_uart_configure(true,9U));
    for(unsigned p=1;p<=8;p++){
        foxeer_uart_t u;
#if defined(BOARD_FOXEERF722V4)
        bool valid=p<=6;
#else
        bool valid=p!=5;
#endif
        assert(foxeer_uart(p,&u)==valid);
        assert(((BOARD_UART_MASK&(1U<<p))!=0U)==valid);
        if(!valid){assert(!board_receiver_uart_configure(false,p));continue;}
        assert(board_receiver_uart_configure(false,p));
        assert(hsbus_uart.Instance==u.instance&&enabled_irq==p);
        assert(hsbus_uart.Init.BaudRate==100000&&hsbus_uart.Init.WordLength==9&&hsbus_uart.Init.StopBits==2);
        assert(hsbus_uart.AdvancedInit.RxPinLevelInvert==1U&&pad==u.rx_pin&&pad_port==u.rx_port&&af==u.af);
        assert(board_receiver_uart_configure(true,p));assert(disabled_irq==p);
        assert(hsbus_uart.Init.BaudRate==420000&&hsbus_uart.Init.WordLength==8&&hsbus_uart.Init.StopBits==1);
        assert(hsbus_uart.AdvancedInit.RxPinLevelInvert==0U);
        UART_HandleTypeDef vtx;
        assert(board_uart_tx_init(p,115200,&vtx));assert(pad==u.tx_pin&&pad_port==u.tx_port);
        assert(board_uart_tx_rx_init(p,115200,&vtx));
        assert(pad==u.rx_pin&&pad_port==u.rx_port&&vtx.Init.Mode==UART_MODE_TX_RX);
        assert(board_uart_half_duplex_init(p,4800,2U,&vtx));assert(vtx.Init.StopBits==2U);
    }
    foxeer_uart_t u;assert(foxeer_uart(1U,&u));
#if defined(BOARD_FOXEERF722V4)
    assert(u.rx_port==GPIOB&&u.rx_pin==GPIO_PIN_7&&u.tx_pin==GPIO_PIN_6);
#else
    assert(u.rx_port==GPIOA&&u.rx_pin==GPIO_PIN_10&&u.tx_pin==GPIO_PIN_9);
#endif
    assert(foxeer_uart(6U,&u));
#if defined(BOARD_FOXEERF722V4)
    assert(u.af==8U);
#else
    assert(u.af==7U);
#endif
    puts("Foxeer UART routing, available-port mask, SBUS inversion, CRSF and switching passed");
}
