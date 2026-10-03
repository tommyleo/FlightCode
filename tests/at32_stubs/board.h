#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#define TRUE 1
#define FALSE 0
#define SET 1
#define BOARD_CORE_CLOCK_HZ 288000000U
#define MOTOR_1_PIN (1U<<10)
#define MOTOR_2_PIN (1U<<9)
#define MOTOR_3_PIN (1U<<1)
#define MOTOR_4_PIN (1U<<0)
#define GPIO_MODE_MUX 2
#define GPIO_MODE_INPUT 0
#define GPIO_MODE_OUTPUT 1
#define GPIO_PULL_DOWN 2
#define GPIO_PULL_UP 1
#define GPIO_MUX_1 1
#define GPIO_MUX_0 0
#define CRM_TMR1_PERIPH_CLOCK 1
#define CRM_TMR2_PERIPH_CLOCK 2
#define CRM_DMA1_PERIPH_CLOCK 3
#define TMR_OVERFLOW_DMA_REQUEST 1
#define TMR_OVERFLOW_SWTRIG 1
#define TMR_OUTPUT_CONTROL_PWM_MODE_A 6
#define TMR_OUTPUT_ACTIVE_HIGH 0
#define DMA_DIR_MEMORY_TO_PERIPHERAL 1
#define DMA_MEMORY_DATA_WIDTH_HALFWORD 1
#define DMA_PERIPHERAL_DATA_WIDTH_HALFWORD 1
#define DMA_PRIORITY_VERY_HIGH 3
#define DMAMUX_DMAREQ_ID_TMR1_OVERFLOW 1
#define DMAMUX_DMAREQ_ID_TMR2_OVERFLOW 2
#define DMA1_GL1_FLAG 1
#define DMA1_GL2_FLAG 2
#define DMA1_GL3_FLAG 4
#define DMA1_GL4_FLAG 8
#define TMR_SELECT_CHANNEL_1 0
#define TMR_SELECT_CHANNEL_2 1
#define TMR_SELECT_CHANNEL_3 2
typedef unsigned tmr_channel_select_type;
typedef struct { uint32_t c1dt,c2dt,c3dt,active[3],period; bool enabled,request; } tmr_type;
typedef struct { uint32_t maddr,paddr,count,position; bool enabled; } dma_channel_type;
typedef struct { unsigned request; } dmamux_channel_type;
typedef struct { unsigned oc_mode,oc_output_state,oc_polarity; } tmr_output_config_type;
typedef struct { uint32_t peripheral_base_addr,memory_base_addr,direction,buffer_size,memory_inc_enable,memory_data_width,peripheral_data_width,priority; } dma_init_type;
static tmr_type fake_timers[2];
static dma_channel_type fake_dma[4];
static dmamux_channel_type fake_mux[4];
#define TMR1 (&fake_timers[0])
#define TMR2 (&fake_timers[1])
#define DMA1_CHANNEL1 (&fake_dma[0])
#define DMA1_CHANNEL2 (&fake_dma[1])
#define DMA1_CHANNEL3 (&fake_dma[2])
#define DMA1_CHANNEL4 (&fake_dma[3])
#define DMA1MUX_CHANNEL1 (&fake_mux[0])
#define DMA1MUX_CHANNEL2 (&fake_mux[1])
#define DMA1MUX_CHANNEL3 (&fake_mux[2])
#define DMA1MUX_CHANNEL4 (&fake_mux[3])
#define DMA1 0
#define GPIOA 0
static uint32_t now_us,mask,gpio_value,gpio_calls;
static struct {uint32_t CYCCNT;} fake_dwt;
#define DWT (&fake_dwt)
static uint32_t board_micros(void){return now_us;}
static uint32_t HAL_GetTick(void){return now_us/1000;}
static void HAL_Delay(uint32_t ms){now_us+=ms*1000;}
static uint32_t __get_PRIMASK(void){return mask;}
static void __disable_irq(void){mask=1;}
static void __set_PRIMASK(uint32_t v){mask=v;}
static void __NOP(void){fake_dwt.CYCCNT++;}
static void crm_periph_clock_enable(unsigned a,bool b){(void)a;(void)b;}
static void dmamux_enable(unsigned a,bool b){(void)a;(void)b;}
static void dmamux_init(dmamux_channel_type *m,unsigned r){m->request=r;}
static void tmr_counter_enable(tmr_type *t,bool e){t->enabled=e;}
static void tmr_dma_request_enable(tmr_type *t,unsigned r,bool e){(void)r;t->request=e;}
static void tmr_channel_value_set(tmr_type *t,unsigned c,uint32_t v){(&t->c1dt)[c]=v;}
static void tmr_event_sw_trigger(tmr_type *t,unsigned e){(void)e;for(unsigned c=0;c<3;c++)t->active[c]=(&t->c1dt)[c];}
static void tmr_base_init(tmr_type *t,uint32_t p,uint32_t div){(void)div;t->period=p+1;}
static void tmr_output_default_para_init(tmr_output_config_type *o){memset(o,0,sizeof(*o));}
static void tmr_output_channel_config(tmr_type *t,unsigned c,tmr_output_config_type *o){(void)t;(void)c;(void)o;}
static void tmr_output_channel_buffer_enable(tmr_type *t,unsigned c,bool e){(void)t;(void)c;(void)e;}
static void tmr_output_enable(tmr_type *t,bool e){(void)t;(void)e;}
static void tmr_counter_value_set(tmr_type *t,uint32_t v){(void)t;(void)v;}
static void dma_default_para_init(dma_init_type *d){memset(d,0,sizeof(*d));}
static void dma_init(dma_channel_type *c,dma_init_type *d){c->maddr=d->memory_base_addr;c->paddr=d->peripheral_base_addr;c->count=d->buffer_size;}
static void dma_channel_enable(dma_channel_type *c,bool e){c->enabled=e;}
static void dma_data_number_set(dma_channel_type *c,uint32_t n){c->count=n;c->position=0;}
static void dma_flag_clear(uint32_t f){(void)f;}
static void gpio_bits_reset(unsigned p,uint16_t pins){(void)p;gpio_value&=~pins;gpio_calls++;}
static void gpio_bits_set(unsigned p,uint16_t pins){(void)p;gpio_value|=pins;gpio_calls++;}
static void gpio_bits_write(unsigned p,uint16_t pins,bool v){if(v)gpio_bits_set(p,pins);else gpio_bits_reset(p,pins);}
static int gpio_input_data_bit_read(unsigned p,uint16_t pins){(void)p;return (gpio_value&pins)!=0;}
static void at32_gpio_config(unsigned p,uint16_t pins,unsigned mode,unsigned pull,unsigned mux){(void)p;(void)pins;(void)mode;(void)pull;(void)mux;gpio_calls++;}
