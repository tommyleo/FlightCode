#include <assert.h>
#include <stdio.h>
#include "../src/drivers/vtx/vtx_smartaudio.c"
static uint32_t tick;
static USART_TypeDef peripheral;
static flight_settings_t settings = {1,4,3,1,25,2};
static bool init_ok = true;
uint32_t HAL_GetTick(void) { return tick; }
const flight_settings_t *flight_settings_get(void) { return &settings; }
bool board_uart_half_duplex_init(uint8_t port, uint32_t baud, uint32_t stop,
                                UART_HandleTypeDef *handle)
{ assert(port == 2 && baud == 4800 && stop == 2); handle->Instance = &peripheral; return init_ok; }
int HAL_HalfDuplex_EnableTransmitter(UART_HandleTypeDef *handle) { (void)handle; return 0; }
int HAL_HalfDuplex_EnableReceiver(UART_HandleTypeDef *handle) { (void)handle; return 0; }
unsigned test_uart_flag(unsigned flag) { return flag == UART_FLAG_TXE || flag == UART_FLAG_TC; }
static void finish_tx(void)
{ for (unsigned i=0; i<12 && state==TRANSMITTING; ++i) { ++tick; vtx_smartaudio_update(false); } assert(state==WAIT_RESPONSE); }
static void start(void)
{ tick=0; assert(vtx_smartaudio_init()); tick=1000; vtx_smartaudio_update(false); assert(state==TRANSMITTING); finish_tx(); }
static void settings_response(uint8_t version, uint8_t channel, uint8_t power,
                              uint8_t mode, uint16_t freq, bool inclusive)
{
    uint8_t frame[32]={0xAA,0x55,version,5,channel,power,mode,(uint8_t)(freq>>8),(uint8_t)freq};
    uint8_t size=5;
    if (version==17) { frame[9]=power; frame[10]=3; frame[11]=0; frame[12]=14; frame[13]=23; frame[14]=29; size=11; }
    frame[3]=(uint8_t)(size+(inclusive?1:0));
    frame[4+size]=crc8(&frame[2],(uint8_t)(size+2));
    for (unsigned i=0;i<size+5U;++i) receive(frame[i]);
}
static void ack(uint8_t code)
{ uint8_t frame[]={0xAA,0x55,code,2,0,1,0}; frame[6]=crc8(&frame[2],4); for(unsigned i=0;i<7;++i)receive(frame[i]); assert(state==QUERY_DELAY); tick+=200; vtx_smartaudio_update(false); finish_tx(); }
int main(void)
{
    const uint8_t get[]={0xAA,0x55,3,0}, channel0[]={0xAA,0x55,7,1,0};
    assert(crc8(get,4)==0x9F && crc8(channel0,5)==0xB8);
    start(); assert(tx_size==6 && tx[0]==0 && tx[5]==0x9F);
    settings_response(9,0,0,0,5865,true);
    assert(state==TRANSMITTING && tx[3]==7 && tx[5]==35);
    finish_tx(); ack(3); settings_response(9,35,0,0,5769,false);
    assert(state==DONE && !strcmp(status,"APPLIED"));
    /* Frequency mode must return to channel mode, even at the desired MHz. */
    start(); settings_response(1,35,7,1,5769,true); assert(tx[3]==7); finish_tx(); ack(3);
    settings_response(1,35,7,0,5769,true); assert(state==DONE);
    settings.vtx_power_mw=800; start(); settings_response(1,35,7,0,5769,true);
    assert(tx[3]==5 && tx[5]==40); finish_tx(); ack(2);
    settings_response(1,35,40,0,5769,true); assert(state==DONE);
    settings.vtx_power_mw=200; start(); settings_response(9,35,0,0,5769,false);
    assert(tx[5]==1); finish_tx(); ack(2); settings_response(9,35,1,0,5769,false); assert(state==DONE);
    start(); settings_response(17,35,14,0,5769,false); assert(tx[5]==(0x80|23));
    finish_tx(); ack(2); settings_response(17,35,23,0,5769,false); assert(state==DONE);
    settings.vtx_power_mw=100; start(); settings_response(9,35,0,0,5769,true);
    assert(state==FAILED && !strcmp(status,"UNSUPPORTED_POWER"));
    settings.vtx_power_mw=25; start();
    /* Corruption, command echo and invalid length must not confirm settings. */
    uint8_t bad[]={0xAA,0x55,9,5,35,0,0,22,137,0};
    for(unsigned i=0;i<sizeof(bad);++i)receive(bad[i]);
    assert(state==WAIT_RESPONSE);
    for(unsigned i=1;i<tx_size;++i)receive(tx[i]);
    assert(state==WAIT_RESPONSE);
    receive(0xAA);receive(0x55);receive(9);receive(255);assert(rx_position==0);
    settings_response(9,35,0,0,5769,false); assert(state==DONE);
    start(); for(unsigned i=0;i<MAX_ATTEMPTS;++i) { tick+=300; vtx_smartaudio_update(false); if(state==FAILED)break; tick+=200;vtx_smartaudio_update(false);finish_tx(); }
    assert(state==FAILED && !strcmp(status,"NO_RESPONSE"));
    start(); vtx_smartaudio_update(true); assert(state==START_DELAY);
    /* Every native TBS band/channel maps to exactly one of the 40 entries. */
    for(unsigned band=0;band<5;++band) for(unsigned ch=0;ch<8;++ch) {
        settings.vtx_band=band;settings.vtx_channel=ch;settings.vtx_region=1;
        bool restricted=band==2 && (ch==3 || ch>=6);
        if(restricted) { assert(!vtx_smartaudio_init()); continue; }
        start(); settings_response(9,255,0,0,0,false);
        assert(tx[3]==7 && tx[5]==band*8+ch);
        finish_tx();ack(3);settings_response(9,(uint8_t)(band*8+ch),0,0,frequencies[band][ch],false);
        assert(state==DONE);
    }
    /* A responsive device which refuses changes must terminate, too. */
    settings.vtx_band=4;settings.vtx_channel=3;start();
    for(unsigned i=0;i<=MAX_ATTEMPTS;++i) {
        settings_response(9,0,0,0,5865,false);
        if(state==FAILED)break;
        finish_tx();ack(3);
    }
    assert(state==FAILED && !strcmp(status,"NOT_CONFIRMED"));
    settings.vtx_band=5; assert(!vtx_smartaudio_init());
    settings.vtx_band=4;settings.vtx_channel=0;settings.vtx_region=0;assert(!vtx_smartaudio_init());
    settings.vtx_region=1; init_ok=false;assert(!vtx_smartaudio_init());assert(!strcmp(status,"UART_ERROR"));
    puts("SmartAudio CRC, R4 mapping, async UART, V1/V2/V2.1 power, readback, bad frames, timeouts and armed guard passed");
}
