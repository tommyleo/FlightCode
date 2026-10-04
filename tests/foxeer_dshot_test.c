#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "dshot.h"

/* Exercise the production F7 driver against register mocks, including DMA
 * channel ordering, frame contents, busy handling and ESC passthrough. */
typedef struct {uint32_t CR1,PSC,ARR,CCR1,CCR2,CCR3,CCR4,CCMR1,CCMR2,CCER,
    BDTR,RCR,DCR,EGR,SR,DIER,CNT,DMAR;} TIM_TypeDef;
typedef struct {uint32_t CR,PAR,FCR,M0AR,NDTR;} DMA_Stream_TypeDef;
static TIM_TypeDef tim1,tim8;
static DMA_Stream_TypeDef stream5,stream1;
static struct {uint32_t HIFCR,LIFCR;} dma2;
static struct {uint32_t CFGR;} rcc;
#define TIM1 (&tim1)
#define TIM8 (&tim8)
#define DMA2_Stream5 (&stream5)
#define DMA2_Stream1 (&stream1)
#define DMA2 (&dma2)
#define RCC (&rcc)
#define RCC_CFGR_PPRE2 7U
#define RCC_CFGR_PPRE2_DIV1 0U
#define DMA_SxCR_EN 1U
#define DMA_MEMORY_TO_PERIPH (1U<<6)
#define DMA_MINC_ENABLE (1U<<10)
#define DMA_PDATAALIGN_HALFWORD (1U<<11)
#define DMA_MDATAALIGN_HALFWORD (1U<<13)
#define DMA_NORMAL 0U
#define DMA_PRIORITY_VERY_HIGH (3U<<16)
#define DMA_FIFOMODE_DISABLE 0U
#define DMA_CHANNEL_6 (6U<<25)
#define DMA_CHANNEL_7 (7U<<25)
#define TIM_CCMR1_OC1M_Pos 4U
#define TIM_CCMR1_OC1PE (1U<<3)
#define TIM_CCMR1_OC2M_Pos 12U
#define TIM_CCMR1_OC2PE (1U<<11)
#define TIM_CCER_CC1E 1U
#define TIM_CCER_CC2E (1U<<4)
#define TIM_CCER_CC3E (1U<<8)
#define TIM_CCER_CC4E (1U<<12)
#define TIM_BDTR_MOE (1U<<15)
#define TIM_DCR_DBA_Pos 0U
#define TIM_DCR_DBL_Pos 8U
#define TIM_EGR_UG 1U
#define TIM_CR1_ARPE (1U<<7)
#define TIM_CR1_CEN 1U
#define TIM_DIER_UDE (1U<<8)
#define DMA_HIFCR_CFEIF5 1U
#define DMA_HIFCR_CDMEIF5 2U
#define DMA_HIFCR_CTEIF5 4U
#define DMA_HIFCR_CHTIF5 8U
#define DMA_HIFCR_CTCIF5 16U
#define DMA_LIFCR_CFEIF1 1U
#define DMA_LIFCR_CDMEIF1 2U
#define DMA_LIFCR_CTEIF1 4U
#define DMA_LIFCR_CHTIF1 8U
#define DMA_LIFCR_CTCIF1 16U
typedef struct {int id;} GPIO_TypeDef;
static GPIO_TypeDef gpioa={0},gpioc={1};
#define GPIOA (&gpioa)
#define GPIOC (&gpioc)
#define MOTOR_1_PORT GPIOA
#define MOTOR_2_PORT GPIOA
#define MOTOR_3_PORT GPIOC
#define MOTOR_4_PORT GPIOC
#define MOTOR_1_PIN (1U<<9)
#define MOTOR_2_PIN (1U<<8)
#define MOTOR_3_PIN (1U<<9)
#define MOTOR_4_PIN (1U<<8)
typedef struct {uint32_t Pin,Mode,Pull,Speed,Alternate;} GPIO_InitTypeDef;
#define GPIO_MODE_AF_PP 1U
#define GPIO_MODE_INPUT 2U
#define GPIO_MODE_OUTPUT_PP 3U
#define GPIO_PULLDOWN 1U
#define GPIO_PULLUP 2U
#define GPIO_SPEED_FREQ_VERY_HIGH 3U
#define GPIO_AF1_TIM1 1U
#define GPIO_AF3_TIM8 3U
#define GPIO_PIN_SET 1U
#define GPIO_PIN_RESET 0U
static uint32_t gpio_modes[2],gpio_af[2],pins[2];
static void HAL_GPIO_Init(GPIO_TypeDef *p,GPIO_InitTypeDef *g){gpio_modes[p->id]=g->Mode;gpio_af[p->id]=g->Alternate;}
static void HAL_GPIO_WritePin(GPIO_TypeDef *p,uint16_t pin,unsigned level){if(level)pins[p->id]|=pin;else pins[p->id]&=~pin;}
static unsigned HAL_GPIO_ReadPin(GPIO_TypeDef *p,uint16_t pin){return (pins[p->id]&pin)?1U:0U;}
static uint32_t tick;
static uint32_t HAL_GetTick(void){return tick;}
static void HAL_Delay(uint32_t ms){tick+=ms;stream5.CR&=~1U;stream1.CR&=~1U;}
static uint32_t HAL_RCC_GetPCLK2Freq(void){return 108000000U;}
#define __HAL_RCC_DMA2_CLK_ENABLE() ((void)0)
#define __HAL_RCC_TIM1_CLK_ENABLE() ((void)0)
#define __HAL_RCC_TIM8_CLK_ENABLE() ((void)0)
#define __DMB() ((void)0)
#define __get_PRIMASK() 0U
#define __disable_irq() ((void)0)
#define __enable_irq() ((void)0)
static struct {uint32_t CYCCNT;} dwt;
#define DWT (&dwt)
static uint32_t SystemCoreClock=216000000U;
static uint32_t board_micros(void){return tick*1000U;}
#include "foxeer_dshot_under_test.inc"

static uint16_t expected_packet(uint16_t value)
{
    uint16_t payload=value<<1U;
    return (payload<<4U)|((payload^(payload>>4U)^(payload>>8U))&15U);
}

int main(void)
{
    rcc.CFGR=4U; /* APB2 divided by 2, hence TIM1/TIM8 kernel 216 MHz. */
    dshot_init();
    assert(gpio_af[0]==1U&&gpio_af[1]==3U);
    assert(tim1.DCR==(13U|(1U<<8))&&tim8.DCR==(15U|(1U<<8)));
    assert(tim1.CCER==17U&&tim8.CCER==0x1100U);
    assert((stream5.CR&(7U<<25))==DMA_CHANNEL_6);
    assert((stream1.CR&(7U<<25))==DMA_CHANNEL_7);
    const motor_protocol_t protocols[]={MOTOR_PROTOCOL_DSHOT300,MOTOR_PROTOCOL_DSHOT600,MOTOR_PROTOCOL_DSHOT1200};
    const unsigned periods[]={720,360,180};
    const uint16_t values[]={48,255,1000,2047};
    for(unsigned p=0;p<3;p++){
        stream5.CR&=~1U;stream1.CR&=~1U;
        assert(motor_protocol_set(protocols[p]));
        dshot_write(values);
        assert(tim1.ARR+1U==periods[p]&&tim8.ARR+1U==periods[p]);
        assert(stream5.NDTR==36&&stream1.NDTR==36);
        for(unsigned m=0;m<4;m++){
            unsigned group=m/2,ch=1-m%2;
            uint16_t packet_value=expected_packet(values[m]);
            for(unsigned b=0;b<16;b++)assert(dshot_dma_buffer[group][b][ch]==
                (packet_value&(1U<<(15-b))?(periods[p]*14+10)/20:(periods[p]*7+10)/20));
            assert(dshot_dma_buffer[group][16][ch]==0&&dshot_dma_buffer[group][17][ch]==0);
        }
        uint16_t saved[2][18][2];memcpy(saved,dshot_dma_buffer,sizeof(saved));
        uint16_t stopped[4]={0};dshot_write(stopped);
        assert(memcmp(saved,dshot_dma_buffer,sizeof(saved))==0); /* Busy DMA is untouched. */
    }
    esc_passthrough_begin();
    assert(!transfer_active()&&!(tim1.CR1&1U)&&!(tim8.CR1&1U));
    assert(gpio_modes[0]==GPIO_MODE_INPUT&&gpio_modes[1]==GPIO_MODE_INPUT);
    esc_passthrough_end();
    assert(gpio_modes[0]==GPIO_MODE_AF_PP&&gpio_modes[1]==GPIO_MODE_AF_PP);
    stream5.CR&=~1U;stream1.CR&=~1U;
    uint16_t clipped[]={1,47,2048,65535};dshot_write(clipped);
    for(unsigned b=0;b<16;b++){
        assert(dshot_dma_buffer[0][b][0]==(180U*7+10)/20);
        assert(dshot_dma_buffer[0][b][1]==(180U*7+10)/20);
        assert(dshot_dma_buffer[1][b][0]==dshot_dma_buffer[1][b][1]);
    }
    puts("Foxeer F7 DSHOT: all protocols, motor order, DMA busy, clipping and passthrough passed");
}
