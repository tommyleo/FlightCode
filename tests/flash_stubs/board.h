#pragma once
#include <stdint.h>
#include <string.h>
#define BOARD_HAS_DATAFLASH 1
#define HAL_OK 0
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1
#define DATAFLASH_CS_PORT 0
#define DATAFLASH_CS_PIN 0
static int fake_spi;
#define DATAFLASH_SPI_HANDLE fake_spi
static uint8_t fake_id[3],last_command;
static unsigned erase_program_commands;
static uint32_t fake_time;
static void HAL_GPIO_WritePin(int p,int pin,int level){(void)p;(void)pin;(void)level;}
static void HAL_Delay(unsigned ms){fake_time+=ms*1000;}
static uint32_t board_micros(void){return ++fake_time;}
static int HAL_SPI_Transmit(void *h,const uint8_t *tx,uint16_t n,unsigned timeout){
(void)h;(void)n;(void)timeout;last_command=tx[0];
if(tx[0]==0x02||tx[0]==0xd8)erase_program_commands++;
return HAL_OK;
}
static int HAL_SPI_Receive(void *h,void *rx,uint16_t n,unsigned timeout){
(void)h;(void)timeout;
if(last_command==0x9f){memcpy(rx,fake_id,n);}else{memset(rx,0xff,n);}
return HAL_OK;
}
static int HAL_SPI_TransmitReceive(void *h,const uint8_t *tx,uint8_t *rx,uint16_t n,unsigned timeout){
(void)h;(void)n;(void)timeout;(void)tx;rx[0]=rx[1]=0;return HAL_OK;
}
