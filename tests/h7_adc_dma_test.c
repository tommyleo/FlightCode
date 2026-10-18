#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <stddef.h>
#define BOARD_SEQUREH7V2 1
#define BOARD_HAS_CURRENT 1
#define ADC_CHANNEL_0 0
#define ADC_CHANNEL_1 1
#define GPIOC 2
#define GPIO_PIN_2 4
#define GPIO_PIN_3 8
#include "h7_adc_board_under_test.inc"
#define ADC3 3
#define ADC_CLOCK_SYNC_PCLK_DIV4 4
#define ADC_RESOLUTION_12B 12
#define ADC_SCAN_ENABLE 1
#define ADC_EOC_SINGLE_CONV 1
#define DISABLE 0
#define ENABLE 1
#define ADC_SOFTWARE_START 0
#define ADC_EXTERNALTRIGCONVEDGE_NONE 0
#define ADC_CONVERSIONDATA_DMA_CIRCULAR 3
#define ADC_OVR_DATA_OVERWRITTEN 1
#define ADC_LEFTBITSHIFT_NONE 0
#define ADC_CALIB_OFFSET 0
#define ADC_SINGLE_ENDED 0
#define ADC_REGULAR_RANK_1 1
#define ADC_REGULAR_RANK_2 2
#define ADC_SAMPLETIME_387CYCLES_5 387
#define ADC_OFFSET_NONE 0
#define GPIO_MODE_ANALOG 0
#define GPIO_NOPULL 0
#define HAL_OK 0
#define DMA1_Stream1 1
#define DMA1_Stream1_IRQn 11
#define DMA_REQUEST_ADC3 3
#define DMA_PERIPH_TO_MEMORY 4
#define DMA_PINC_DISABLE 0
#define DMA_MINC_ENABLE 1
#define DMA_PDATAALIGN_HALFWORD 2
#define DMA_MDATAALIGN_HALFWORD 2
#define DMA_CIRCULAR 8
#define DMA_PRIORITY_MEDIUM 1
#define DMA_FIFOMODE_DISABLE 0
typedef struct { unsigned Request,Direction,PeriphInc,MemInc,PeriphDataAlignment,MemDataAlignment,Mode,Priority,FIFOMode; } DMA_InitTypeDef;
typedef struct { unsigned Instance; DMA_InitTypeDef Init; } DMA_HandleTypeDef;
typedef struct {
    unsigned ClockPrescaler,Resolution,ScanConvMode,EOCSelection,LowPowerAutoWait;
    unsigned ContinuousConvMode,NbrOfConversion,DiscontinuousConvMode,ExternalTrigConv;
    unsigned ExternalTrigConvEdge,ConversionDataManagement,Overrun,LeftBitShift,OversamplingMode;
} ADC_InitTypeDef;
typedef struct { unsigned Instance; ADC_InitTypeDef Init; DMA_HandleTypeDef *DMA_Handle; } ADC_HandleTypeDef;
typedef struct { unsigned Pin,Mode,Pull; } GPIO_InitTypeDef;
typedef struct { unsigned Channel,Rank,SamplingTime,SingleDiff,OffsetNumber,Offset; } ADC_ChannelConfTypeDef;
static ADC_HandleTypeDef hadc_battery, other_adc;
static uint32_t now_us, mask, clock_on, enabled_irq, cache_invalidations, clean_invalidations;
static uint16_t *dma_target;
static uint32_t dma_length;
static unsigned adc_clock,gpio_pins,configured_channels;
#define __HAL_RCC_ADC3_CLK_ENABLE() (adc_clock=3)
static void HAL_GPIO_Init(unsigned port,GPIO_InitTypeDef *gpio) {
    assert(port==GPIOC&&gpio->Mode==GPIO_MODE_ANALOG&&gpio->Pull==GPIO_NOPULL);gpio_pins|=gpio->Pin;
}
static int HAL_ADC_Init(ADC_HandleTypeDef *adc) {
    assert(adc_clock==3&&adc->Instance==ADC3&&adc->Init.Resolution==12&&adc->Init.ClockPrescaler==4);
    assert(adc->Init.ScanConvMode==ENABLE&&adc->Init.ContinuousConvMode==ENABLE&&adc->Init.NbrOfConversion==2);
    assert(adc->Init.ConversionDataManagement==ADC_CONVERSIONDATA_DMA_CIRCULAR);return HAL_OK;
}
static int HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *adc,unsigned calibration,unsigned input) {
    assert(adc->Instance==ADC3&&calibration==ADC_CALIB_OFFSET&&input==ADC_SINGLE_ENDED);return HAL_OK;
}
static int HAL_ADC_ConfigChannel(ADC_HandleTypeDef *adc,ADC_ChannelConfTypeDef *channel) {
    assert(adc->Instance==ADC3&&channel->SamplingTime==387&&channel->SingleDiff==ADC_SINGLE_ENDED);
    assert(channel->Rank==configured_channels+1);
    assert(channel->Channel==(configured_channels==0?BATTERY_ADC_CHANNEL:CURRENT_ADC_CHANNEL));
    ++configured_channels;return HAL_OK;
}
static void board_fatal_error(void) { assert(!"fatal"); }
static uint32_t HAL_GetTick(void) { return now_us / 1000U; }
#define __HAL_RCC_DMA1_CLK_ENABLE() (clock_on=1)
#define __HAL_LINKDMA(adc,field,dma) ((adc)->field=&(dma))
static int HAL_DMA_Init(DMA_HandleTypeDef *dma) {
    assert(clock_on && dma->Instance==DMA1_Stream1);
    assert(dma->Init.Request==DMA_REQUEST_ADC3 && dma->Init.Direction==DMA_PERIPH_TO_MEMORY);
    assert(dma->Init.PeriphInc==0 && dma->Init.MemInc==1 && dma->Init.Mode==DMA_CIRCULAR);
    assert(dma->Init.PeriphDataAlignment==2 && dma->Init.MemDataAlignment==2);
    return HAL_OK;
}
static void HAL_NVIC_SetPriority(unsigned irq,unsigned priority,unsigned sub) { assert(irq==11&&priority==5&&sub==0); }
static void HAL_NVIC_EnableIRQ(unsigned irq) { enabled_irq=irq; }
static void HAL_DMA_IRQHandler(DMA_HandleTypeDef *dma) { assert(dma==hadc_battery.DMA_Handle); }
static int HAL_ADC_Start_DMA(ADC_HandleTypeDef *adc,uint32_t *buffer,uint32_t words) {
    assert(adc==&hadc_battery && adc->DMA_Handle && enabled_irq==11);
    dma_target=(uint16_t *)buffer; dma_length=words; return HAL_OK;
}
static void SCB_CleanInvalidateDCache_by_Addr(uint32_t *p,int32_t bytes) {
    assert(((uintptr_t)p%32)==0 && bytes==512); ++clean_invalidations;
}
static void SCB_InvalidateDCache_by_Addr(uint32_t *p,int32_t bytes) {
    assert(((uintptr_t)p%32)==0 && bytes==256); ++cache_invalidations;
}
static uint32_t __get_PRIMASK(void) { return mask; }
static void __disable_irq(void) { mask=1; }
static void __set_PRIMASK(uint32_t value) { mask=value; }
#include "sequre_adc_dma.h"
#include "h7_adc_init_under_test.inc"
static void fill(unsigned offset,unsigned voltage,unsigned current) {
    for(unsigned i=offset;i<offset+128;i+=2) { dma_target[i]=voltage;dma_target[i+1]=current; }
}
static void block(unsigned offset,unsigned voltage,unsigned current) {
    fill(offset,voltage,current);now_us+=2000;
    if(offset)HAL_ADC_ConvCpltCallback(&hadc_battery);else HAL_ADC_ConvHalfCpltCallback(&hadc_battery);
    board_battery_update();
}
int main(void) {
    battery_adc_init();assert(configured_channels==2&&gpio_pins==(GPIO_PIN_2|GPIO_PIN_3));
    assert(CURRENT_ADC_CHANNEL==0&&BATTERY_ADC_CHANNEL==1);
    assert(dma_length==256&&clean_invalidations==1);
    board_battery_update();assert(current_adc_count==0&&battery_current_filtered==0);
    fill(128,3000,900); /* Poison opposite half to catch incorrect offsets. */
    block(0,2100,90);
    assert(current_adc_count==64&&current_adc_raw==90&&cache_invalidations==1);
    assert(fabsf(battery_current_filtered-0.68943f)<0.001f);
    assert(fabsf(battery_voltage_filtered-18.61538f)<0.002f);
    uint32_t count=current_adc_count, stamp=current_adc_last_us;
    now_us+=10000;board_battery_update();assert(current_adc_count==count&&current_adc_last_us==stamp);
    HAL_ADC_ConvHalfCpltCallback(&other_adc);HAL_ADC_ErrorCallback(&other_adc);
    assert(dma_samples==count&&battery_adc_errors==0);
    for(unsigned i=0;i<250;i++)block(i%2?128:0,2100,900);
    assert(battery_current_filtered>6.82f&&battery_current_filtered<6.90f);
    assert(current_adc_raw==900&&current_adc_count==16064);
    assert(fabsf(battery_voltage_filtered-18.61538f)<0.002f); /* Independent rank mapping. */
    for(unsigned i=0;i<500;i++)block(i%2?128:0,2100,90);
    assert(fabsf(battery_current_filtered-0.68943f)<0.003f);
    mask=1;block(128,2100,90);assert(mask==1);mask=0;
    current_adc_last_us=UINT32_MAX-999U;now_us=UINT32_MAX-999U;
    block(0,2100,900);assert(current_adc_last_us==1000U&&isfinite(battery_current_filtered));
    HAL_ADC_ErrorCallback(&hadc_battery);assert(battery_adc_errors==1);
    DMA1_Stream1_IRQHandler();
    puts("PASS: ADC DMA setup, half/full rank mapping, cache alignment, fresh/stale samples, scaling, step response, IRQ state, timer wrap and errors");
}
