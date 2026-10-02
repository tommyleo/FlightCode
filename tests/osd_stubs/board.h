#pragma once
#include <stdint.h>
#define BOARD_HAS_OSD 1
#define BOARD_HAS_CURRENT 0
#define BOARD_OSD_SHARES_DATAFLASH_SPI 0
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1
#define HAL_OK 0
#define SPI_POLARITY_LOW 0
#define SPI_POLARITY_HIGH 1
#define SPI_PHASE_1EDGE 0
#define SPI_PHASE_2EDGE 1
#define SPI_BAUDRATEPRESCALER_8 8
#define SPI_BAUDRATEPRESCALER_64 64
typedef int HAL_StatusTypeDef;
static struct { struct { int CLKPolarity,CLKPhase,BaudRatePrescaler; } Init; } mock_spi;
#define OSD_SPI_HANDLE mock_spi
#define MAX7456_CS_PORT 0
#define MAX7456_CS_PIN 0
static uint8_t status=2, vm0, dmah, dmal; static unsigned writes;
static inline void HAL_GPIO_WritePin(int p,int n,int v){(void)p;(void)n;(void)v;}
static inline void HAL_Delay(unsigned t){(void)t;}
static inline int HAL_SPI_DeInit(void*p){(void)p;return 0;}
static inline int HAL_SPI_Init(void*p){(void)p;return 0;}
static inline int HAL_SPI_Transmit(void*p,const uint8_t*b,unsigned n,unsigned t){(void)p;(void)b;(void)n;(void)t;return 0;}
static inline int HAL_SPI_TransmitReceive(void*p,uint8_t*tx,uint8_t*rx,unsigned n,unsigned t){
(void)p;(void)n;(void)t;
if(tx[0]==0x8c)rx[1]=0x1b;else if(tx[0]==0xa0)rx[1]=status;else rx[1]=0;
if(tx[0]==0)vm0=tx[1];
if(tx[0]==5)dmah=tx[1];
if(tx[0]==6)dmal=tx[1];
if(tx[0]==7)++writes;
return 0;
}
