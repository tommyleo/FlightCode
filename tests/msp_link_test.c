#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "../src/drivers/osd/hdzero_msp.h"
#include "../src/drivers/osd/msp_displayport.h"

#define PLATFORM_AT32 1
#define BOARD_HAS_DIGITAL_OSD 1
#define BOARD_HAS_CURRENT 0
#define VTX_PROTOCOL_HDZERO_MSP 3U
#define OSD_ELEMENT_COUNT 5U
#define UART_FLAG_TXE 1U
#define UART_FLAG_RXNE 2U
#define UART_FLAG_ORE 3U
#define UART_FLAG_FE 4U
#define UART_FLAG_NE 5U
#define RESET 0U
typedef struct { unsigned id; } peripheral_t;
typedef struct { peripheral_t *Instance; } UART_HandleTypeDef;
typedef struct {
    uint32_t vtx_protocol,vtx_uart,vtx_band,vtx_channel,vtx_power_mw;
    uint32_t osd_enabled,osd_element_enabled_mask,osd_element_positions[5];
    uint32_t vtx_osd_enabled,vtx_osd_position;
    char osd_pilot_name[13];
} flight_settings_t;
static flight_settings_t settings={.vtx_protocol=3,.vtx_uart=5,
    .vtx_band=4,.vtx_channel=2,.vtx_power_mw=200,.osd_enabled=1,
    .osd_element_enabled_mask=1};
static peripheral_t peripheral;
static uint32_t now;
static bool armed, overrun, uart_ok=true;
static uint8_t rx[256], wire[4096];
static unsigned rx_read,rx_size,wire_size;
static const flight_settings_t *flight_settings_get(void){return &settings;}
static bool flight_control_is_armed(void){return armed;}
static uint32_t board_micros(void){return now;}
static bool board_uart_tx_rx_init(uint8_t port,uint32_t baud,UART_HandleTypeDef *h)
{assert(port==5 && baud==115200);h->Instance=&peripheral;return uart_ok;}
static unsigned flag(UART_HandleTypeDef *h,unsigned f)
{(void)h;return f==UART_FLAG_TXE || (f==UART_FLAG_RXNE && rx_read<rx_size) || (f==UART_FLAG_ORE && overrun);}
#define __HAL_UART_GET_FLAG(h,f) flag(h,f)
static void at32_uart_clear_errors(UART_HandleTypeDef *h)
{(void)h;assert(overrun);overrun=false;if(rx_read<rx_size)++rx_read;}
static uint16_t usart_data_receive(peripheral_t *p)
{(void)p;assert(rx_read<rx_size);return rx[rx_read++];}
static void usart_data_transmit(peripheral_t *p,uint16_t byte)
{(void)p;assert(wire_size<sizeof(wire));wire[wire_size++]=(uint8_t)byte;}
#include "msp_displayport_under_test.inc"

static void drain(void)
{for(unsigned i=0;i<700;++i)msp_displayport_process();}
static void poll(uint8_t cmd)
{
    const uint8_t request[]={'$','M','<',0,cmd,cmd};
    memcpy(rx,request,sizeof(request));rx_read=0;rx_size=sizeof(request);
    wire_size=0;drain();assert(rx_read==rx_size);
}
static void check_wire(void)
{
    unsigned offset=0;
    while(offset<wire_size){
        assert(wire[offset]=='$' && wire[offset+1]=='M' && wire[offset+2]=='>');
        const unsigned n=wire[offset+3];
        assert(offset+n+6U<=wire_size);
        uint8_t checksum=0;
        for(unsigned i=offset+3;i<offset+n+5U;++i)checksum^=wire[i];
        assert(checksum==wire[offset+n+5U]);offset+=n+6U;
    }
}
int main(void)
{
    now=1000;assert(msp_displayport_init());drain();check_wire();
    assert(wire[4]==182 && wire[5]==5 && wire[6]==2); /* OSD options survive. */
    now=1000000;poll(88);check_wire();
    const uint8_t expected[]={'$','M','>',15,88,5,5,3,2,0,0x64,0x16,1,0,0,0,0,6,8,2};
    assert(wire_size==21 && memcmp(wire,expected,sizeof(expected))==0);
    assert(strcmp(msp_displayport_vtx_status_name(),"MSP_SETTINGS_SENT")==0);
    /* RF control continues with the OSD disabled. */
    settings.osd_enabled=0;poll(88);assert(wire[4]==88);
    /* Actual transmitted config is frozen even if settings change armed. */
    armed=true;settings.vtx_channel=7;settings.vtx_power_mw=25;
    poll(88);assert(wire[7]==3 && wire[8]==2);
    armed=false;poll(88);assert(wire[7]==8 && wire[8]==1);
    /* Native v2 response serialization is distinct from v1 DisplayPort. */
    const uint8_t api[]={'$','X','<',0,1,0,0,0,0x45};
    memcpy(rx,api,sizeof(api));rx_read=0;rx_size=sizeof(api);wire_size=0;drain();
    assert(wire_size==12 && memcmp(wire,"$X>",3)==0);
    assert(wire[3]==0 && wire[4]==1 && wire[5]==0 && wire[6]==3 && wire[7]==0);
    assert(wire[8]==0 && wire[9]==1 && wire[10]==46);
    uint8_t crc=0;for(unsigned i=3;i<11;++i)crc=hdzero_crc(crc,wire[i]);assert(wire[11]==crc);
    /* Full TX queue rejects whole frames without partial headers. */
    tx_head=tx_tail=0;uint8_t block[64]={0};
    for(unsigned i=0;i<7;++i)assert(enqueue_packet(false,'>',88,block,64));
    const uint16_t before=tx_head;
    assert(!enqueue_packet(true,'>',88,block,64));assert(tx_head==before);
    tx_head=tx_tail=0;
    /* Changing the active UART/protocol stops reads/writes until reboot. */
    settings.vtx_uart=7;wire_size=0;drain();assert(wire_size==0 && !msp_displayport_is_available());
    settings.vtx_uart=5;settings.vtx_protocol=0;drain();assert(wire_size==0);
    settings.vtx_protocol=3;overrun=true;drain();assert(!overrun);
    uart_ok=false;assert(!msp_displayport_init());
    assert(strcmp(msp_displayport_vtx_status_name(),"UART_ERROR")==0);
    puts("Shared DisplayPort/MSP UART framing, AT32 RX, queue, armed guard and OSD-off control: PASS");
}
