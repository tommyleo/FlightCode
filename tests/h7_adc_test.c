#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#define BOARD_HAS_CURRENT 1
#define BOARD_SEQUREH7V2 1
#define ADC_CHANNEL_0 0
#define ADC_CHANNEL_1 1
#define ADC_CHANNEL_12 12
#define ADC_CHANNEL_13 13
#define ADC1 1
#define ADC3 3
#define GPIOC 2
#define GPIO_PIN_2 4
#define GPIO_PIN_3 8
#include "h7_adc_board_under_test.inc"
#define ADC_REGULAR_RANK_1 1
#define ADC_SAMPLETIME_387CYCLES_5 387
#define ADC_SINGLE_ENDED 0
#define ADC_OFFSET_NONE 0
#define HAL_OK 0
#define HAL_ERROR 1
#define HAL_TIMEOUT 2
#define ADC_CLOCK_SYNC_PCLK_DIV4 4
#define ADC_RESOLUTION_12B 12
#define ADC_SCAN_DISABLE 0
#define ADC_EOC_SINGLE_CONV 0
#define DISABLE 0
#define ADC_SOFTWARE_START 0
#define ADC_EXTERNALTRIGCONVEDGE_NONE 0
#define ADC_CONVERSIONDATA_DR 0
#define ADC_OVR_DATA_OVERWRITTEN 0
#define ADC_LEFTBITSHIFT_NONE 0
#define ADC_CALIB_OFFSET 0
#define GPIO_MODE_ANALOG 0
#define GPIO_NOPULL 0
typedef struct {unsigned Pin,Mode,Pull;} GPIO_InitTypeDef;
typedef struct {
    unsigned ClockPrescaler,Resolution,ScanConvMode,EOCSelection,LowPowerAutoWait;
    unsigned ContinuousConvMode,NbrOfConversion,DiscontinuousConvMode;
    unsigned ExternalTrigConv,ExternalTrigConvEdge,ConversionDataManagement;
    unsigned Overrun,LeftBitShift,OversamplingMode;
} ADC_InitTypeDef;
typedef struct {unsigned Instance;ADC_InitTypeDef Init;} ADC_HandleTypeDef;
typedef struct {unsigned Channel,Rank,SamplingTime,SingleDiff,OffsetNumber,Offset;} ADC_ChannelConfTypeDef;
typedef struct {uint32_t raw,samples,age_ms,errors;} board_current_adc_diagnostics_t;
static ADC_HandleTypeDef hadc_battery;
static unsigned adc_clock,gpio_pins;
#define __HAL_RCC_ADC3_CLK_ENABLE() (adc_clock=3)
#define __HAL_RCC_ADC12_CLK_ENABLE() (adc_clock=1)
static void HAL_GPIO_Init(unsigned port,GPIO_InitTypeDef *g){assert(port==GPIOC);assert(g->Mode==GPIO_MODE_ANALOG&&g->Pull==GPIO_NOPULL);gpio_pins|=g->Pin;}
static int HAL_ADC_Init(ADC_HandleTypeDef *h){assert(adc_clock==3&&h->Instance==ADC3);assert(h->Init.ClockPrescaler==4&&h->Init.Resolution==12);return HAL_OK;}
static int HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *h,unsigned mode,unsigned input){assert(h->Instance==ADC3&&mode==ADC_CALIB_OFFSET&&input==ADC_SINGLE_ENDED);return HAL_OK;}
static void board_fatal_error(void){assert(!"ADC initialization failed");}
static uint32_t now_us,current_input=90,voltage_input=2100;
static unsigned selected,configured_voltage,configured_current,stops;
static bool started,stall,config_error,start_error;
static uint32_t board_micros(void){return now_us;}
static int HAL_ADC_ConfigChannel(void *h,ADC_ChannelConfTypeDef *c){
assert(((ADC_HandleTypeDef *)h)->Instance==ADC3);if(config_error)return HAL_ERROR;selected=c->Channel;
assert(selected==0||selected==1);assert(!started);assert(c->SamplingTime==387);
if(selected==0)configured_current++;else configured_voltage++;
return HAL_OK;
}
static int HAL_ADC_Start(void *h){(void)h;if(start_error)return HAL_ERROR;started=true;return HAL_OK;}
static int HAL_ADC_PollForConversion(void *h,unsigned timeout){(void)h;assert(timeout==0);return stall?HAL_TIMEOUT:HAL_OK;}
static uint32_t HAL_ADC_GetValue(void *h){(void)h;assert(started);return selected==0?current_input:voltage_input;}
static int HAL_ADC_Stop(void *h){(void)h;started=false;stops++;return HAL_OK;}
#include "h7_adc_under_test.inc"
static void advance(unsigned ms){for(unsigned i=0;i<ms;i++){now_us+=1000;board_battery_update();}}
int main(void)
{
    /* STM32H743 LQFP100 PC2_C/PC3_C are ADC3 INP0/INP1, not ADC1 12/13. */
    assert(CURRENT_ADC_CHANNEL==0&&BATTERY_ADC_CHANNEL==1);
    battery_adc_init();assert(gpio_pins==(GPIO_PIN_2|GPIO_PIN_3));
    configured_voltage=0;configured_current=0;
    board_current_adc_diagnostics_t d;
    board_current_adc_diagnostics(&d);assert(d.samples==0&&d.age_ms==UINT32_MAX);
    advance(1000);
    assert(fabsf(board_battery_current()-0.68943f)<0.001f);
    board_current_adc_diagnostics(&d);assert(d.raw==90&&d.samples>=19&&d.errors==0&&d.age_ms<60);
    assert(configured_current==configured_voltage||configured_current+1==configured_voltage);
    voltage_input=3000;advance(6000);
    assert(fabsf(board_battery_current()-0.68943f)<0.001f); /* independent channel */
    current_input=900;advance(10000);
    assert(board_battery_current()>6.7f&&board_battery_current()<6.9f);
    uint32_t samples=d.samples;board_current_adc_diagnostics(&d);assert(d.samples>samples);
    samples=d.samples;stall=true;unsigned old_stops=stops;advance(150);
    board_current_adc_diagnostics(&d);assert(d.errors>0&&d.samples==samples&&stops>old_stops);
    stall=false;advance(500);board_current_adc_diagnostics(&d);assert(d.samples>samples&&d.age_ms<60);
    unsigned old_attempts=configured_current+configured_voltage;uint32_t errors=d.errors;
    config_error=true;advance(100);board_current_adc_diagnostics(&d);assert(d.errors>errors&&d.errors-errors<=5);
    config_error=false;start_error=true;advance(100);start_error=false;advance(500);
    assert(configured_current+configured_voltage>old_attempts);
    /* Scheduling and diagnostics remain correct when microseconds wrap. */
    now_us=UINT32_MAX-30000U;battery_adc_pending=false;started=false;battery_adc_next_sample_us=now_us;
    samples=current_adc_count;advance(150);assert(current_adc_count>samples);
    board_current_adc_diagnostics(&d);assert(d.age_ms<60);
    puts("H7 ADC3 physical pin routing, initialization, current scaling, changing input, timeout recovery and wraparound passed");
}
