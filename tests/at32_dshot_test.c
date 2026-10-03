#include <assert.h>
#include <stdio.h>
#include "../src/drivers/motors/dshot_at32.c"

/* Independent known DSHOT packet vectors: throttle + telemetry=0 + checksum. */
static const uint16_t values[4]={0,48,1000,2047};
static const uint16_t packets[4]={0x0000,0x0606,0x7D0A,0xFFEE};
static void overflow(void)
{
    /* Timer shadow registers latch before an overflow-triggered DMA write. */
    for(unsigned t=0;t<2;t++)tmr_event_sw_trigger(&fake_timers[t],0);
    for(unsigned i=0;i<4;i++){
        dma_channel_type *d=&fake_dma[i];
        if(d->enabled&&d->count&&timers[i]->request){
            assert(d->maddr==(uint32_t)(uintptr_t)&frame[i][2]);
            assert(d->paddr==(uint32_t)(uintptr_t)compare[i]);
            tmr_channel_value_set(timers[i],outputs[i],frame[i][2+d->position]);
            d->position++;d->count--;
        }
    }
}
static void check_protocol(motor_protocol_t p,uint32_t period)
{
    now_us+=100;assert(motor_protocol_set(p));dshot_write(values);
    assert(fake_timers[0].period==period&&fake_timers[1].period==period);
    for(unsigned bit=0;bit<18;bit++){
        for(unsigned i=0;i<4;i++){
            uint32_t expected=bit>=16?0:(packets[i]&(0x8000U>>bit)?period*3/4:period*3/8);
            assert(timers[i]->active[outputs[i]]==expected);
        }
        overflow();
    }
    for(unsigned i=0;i<4;i++)assert(fake_dma[i].count==0);
}
int main(void)
{
    dshot_write(values);assert(!frame_active);
    dshot_init();
    assert(timers[0]==TMR1&&outputs[0]==TMR_SELECT_CHANNEL_3);
    assert(timers[1]==TMR1&&outputs[1]==TMR_SELECT_CHANNEL_2);
    assert(timers[2]==TMR2&&outputs[2]==TMR_SELECT_CHANNEL_2);
    assert(timers[3]==TMR2&&outputs[3]==TMR_SELECT_CHANNEL_1);
    for(unsigned i=0;i<4;i++)assert(fake_mux[i].request==(i<2?1U:2U));
    check_protocol(MOTOR_PROTOCOL_DSHOT300,960);
    check_protocol(MOTOR_PROTOCOL_DSHOT600,480);
    check_protocol(MOTOR_PROTOCOL_DSHOT1200,240);
    uint16_t snapshot[4][FRAME_WORDS];memcpy(snapshot,frame,sizeof(frame));
    uint16_t other[4]={4000,0,0,0};dshot_write(other);
    assert(memcmp(snapshot,frame,sizeof(frame))==0); /* early frame cannot truncate DMA */
    now_us+=100;dshot_write(other);assert(memcmp(frame[0],snapshot[3],sizeof(frame[0]))==0);
    now_us+=100;esc_passthrough_begin();dshot_write(values);assert(!frame_active);
    unsigned calls=gpio_calls;esc_passthrough_input(4);esc_passthrough_output(4);esc_passthrough_write(4,true);
    assert(gpio_calls==calls&&!esc_passthrough_read(4));
    mask=0;assert(esc_passthrough_critical_enter()==0);assert(mask==1);esc_passthrough_critical_exit(0);assert(mask==0);
    mask=1;assert(esc_passthrough_critical_enter()==1);esc_passthrough_critical_exit(1);assert(mask==1);
    esc_passthrough_end();assert(!passthrough&&!frame_active);
    assert(!motor_protocol_set((motor_protocol_t)99));
    assert(dshot_from_percent(NAN)==0&&dshot_from_percent(-1)==0&&dshot_from_percent(0)==0);
    assert(dshot_from_percent(100)==2047&&dshot_from_percent(101)==2047);
    puts("AT32 motor DMA waveform and passthrough tests passed");
}
