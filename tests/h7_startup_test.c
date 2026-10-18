#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#define FLASH_BASE 0x08000000U
#define SCB_ICSR_PENDSTCLR_Msk (1U<<25)
#define SCB_ICSR_PENDSVCLR_Msk (1U<<27)
#define CoreDebug_DEMCR_TRCENA_Msk (1U<<24)
#define DWT_CTRL_CYCCNTENA_Msk 1U
typedef struct {uint32_t CTRL,LOAD,VAL;} Tick;
typedef struct {uint32_t ICER[8],ICPR[8];} Interrupts;
typedef struct {uint32_t ICSR,VTOR;} Core;
typedef struct {uint32_t DEMCR;} Debug;
typedef struct {uint32_t CYCCNT,CTRL;} Cycle;
static Tick tick;static Interrupts irqs;static Core core;static Debug debug;static Cycle cycle;
#define SysTick (&tick)
#define NVIC (&irqs)
#define SCB (&core)
#define CoreDebug (&debug)
#define DWT (&cycle)
static unsigned primask,phase,dsb,isb;
static void __disable_irq(void){primask=1;}
static void __DSB(void){++dsb;}
static void __ISB(void){++isb;}
static void __enable_irq(void){assert(core.VTOR==FLASH_BASE&&dsb&&isb);primask=0;}
static void HAL_Init(void){
 assert(phase++==0&&primask==0&&core.VTOR==FLASH_BASE);
 assert(tick.CTRL==0&&tick.LOAD==0&&tick.VAL==0);
 for(unsigned i=0;i<8;i++)assert(irqs.ICER[i]==UINT32_MAX&&irqs.ICPR[i]==UINT32_MAX);
 assert(core.ICSR==(SCB_ICSR_PENDSTCLR_Msk|SCB_ICSR_PENDSVCLR_Msk));
}
static void clock_init(void){assert(phase++==1);}
static void gpio_init(void){assert(phase++==2);}
#include "h7_startup_under_test.inc"
int main(void){
 /* Simulate a ROM jump with interrupts masked, vectors in system memory,
  * and an already running bootloader SysTick. */
 core.VTOR=0x1FF09800U;tick.CTRL=7;tick.LOAD=1000;tick.VAL=500;primask=1;
 board_init();assert(phase==3&&primask==0&&cycle.CYCCNT==0&&cycle.CTRL==1);
 /* The same entry sequence must work after a normal cold reset. */
 phase=dsb=isb=0;core.VTOR=0;tick.CTRL=0;primask=0;
 board_init();assert(phase==3&&core.VTOR==FLASH_BASE);
 puts("PASS actual H7 startup: ROM vectors, inherited IRQ mask/pending state, SysTick cleanup and cold reset");
}
