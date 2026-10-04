#include <assert.h>
#include <stdio.h>
#include "../src/drivers/osd/hdzero_msp.h"

static bool sent_v2, queue_available=true;
static char direction;
static uint16_t command;
static uint8_t payload[64], length;
static unsigned sent;
static bool transmit(bool v2, char dir, uint16_t cmd, const uint8_t *data, uint8_t size)
{
    if (!queue_available) return false;
    ++sent; sent_v2=v2; direction=dir; command=cmd; length=size;
    if (size) memcpy(payload,data,size);
    return true;
}
static void feed(hdzero_msp_t *p,const uint8_t *bytes,unsigned count,bool armed,uint32_t now)
{ for (unsigned i=0;i<count;++i) hdzero_receive(p,bytes[i],armed,now,transmit); }
static void request(hdzero_msp_t *p,uint8_t cmd,bool armed,uint32_t now)
{ const uint8_t packet[]={'$','M','<',0,cmd,cmd}; feed(p,packet,sizeof(packet),armed,now); }

int main(void)
{
    hdzero_msp_t p={0};
    hdzero_configure(&p,4,2,200,false); /* R3, 5732 MHz, power INDEX 2. */
    assert(p.configured);
    assert(strcmp(hdzero_status(&p,0),"INITIALIZING")==0);
    assert(strcmp(hdzero_status(&p,5000000),"NO_RESPONSE")==0);
    request(&p,2,false,1000000);
    assert(command==2 && direction=='>' && length==4 && !sent_v2);
    assert(memcmp(payload,"BTFL",4)==0);
    assert(strcmp(hdzero_status(&p,1000000),"MSP_CONNECTED")==0);
    request(&p,88,false,1000001);
    const uint8_t expected[]={5,5,3,2,0,0x64,0x16,1,0,0,0,0,6,8,2};
    assert(length==15 && memcmp(payload,expected,15)==0);
    assert(strcmp(hdzero_status(&p,1000001),"MSP_SETTINGS_SENT")==0);
    /* Communication does not advertise APPLIED / RF confirmation. */
    assert(strcmp(hdzero_status(&p,3000001),"NO_RESPONSE")==0);
    request(&p,101,true,3100000); assert(length==11 && payload[6]==1);
    request(&p,189,false,3100001); assert(length==2 && payload[0]==30 && payload[1]==16);
    request(&p,119,false,3100002); assert(length==1 && payload[0]==0);
    request(&p,105,false,3100003); assert(length==16 && payload[6]==0xE8 && payload[7]==3);
    const unsigned before=sent;
    const uint8_t invalid[]={'$','M','<',0,88,0};
    feed(&p,invalid,sizeof(invalid),false,3100004); assert(sent==before);
    const uint8_t response[]={'$','M','>',0,88,88};
    feed(&p,response,sizeof(response),false,3100005); assert(sent==before);
    request(&p,250,false,3100006); assert(direction=='!' && length==0);
    /* Remote startup config/table uploads cannot overwrite saved selection. */
    const uint8_t set_config[]={'$','M','<',4,89,0,0,1,0,92};
    feed(&p,set_config,sizeof(set_config),false,3100007); assert(direction=='!');
    assert(memcmp(p.config,expected,15)==0);
    /* Armed polling and unsolicited pushes keep the selected RF settings. */
    hdzero_configure(&p,4,7,25,true);
    request(&p,88,true,3200000); assert(memcmp(payload,expected,15)==0);
    hdzero_configure(&p,4,7,25,false); assert(!p.settings_sent);
    queue_available=false; hdzero_service(&p,3800000,transmit); assert(!p.settings_sent);
    queue_available=true; hdzero_service(&p,3800001,transmit);
    assert(command==88 && payload[2]==8 && payload[3]==1 && p.settings_sent);
    unsigned count=sent; hdzero_service(&p,3800002,transmit); assert(sent==count);
    hdzero_service(&p,5200000,transmit); assert(sent==count); /* Disconnected. */
    /* Every native channel agrees with the documented frequency table. */
    assert(hdzero_frequency(2,0)==5705 && hdzero_frequency(2,1)==0);
    assert(hdzero_frequency(3,3)==5800 && hdzero_frequency(3,2)==0);
    assert(hdzero_frequency(5,0)==5362 && hdzero_frequency(5,7)==5621);
    assert(hdzero_frequency(6,0)==0 && hdzero_frequency(4,8)==0);
    hdzero_configure(&p,0,0,25,false); assert(!p.configured);
    request(&p,88,false,6000000); assert(direction=='!');
    assert(strcmp(hdzero_status(&p,6000000),"INVALID_SETTINGS")==0);
    hdzero_configure(&p,4,0,100,false); assert(!p.configured);
    hdzero_configure(&p,4,0,25,false);
    /* A native MSPv2 API_VERSION request with a known DVB-S2 CRC. */
    const uint8_t v2[]={'$','X','<',0,1,0,0,0,0x45};
    feed(&p,v2,sizeof(v2),false,6100000);
    assert(sent_v2 && command==1 && direction=='>' && length==3);
    /* Partial frames expire; oversized bodies cannot inject nested requests. */
    const uint8_t partial[]={'$','M','<',0};
    feed(&p,partial,sizeof(partial),false,6200000);
    request(&p,88,false,6400000); assert(command==88 && !sent_v2);
    uint8_t oversized[71]={'$','M','<',65,88};
    memcpy(oversized+6,"$M<\0\x58\x58",6);
    count=sent; feed(&p,oversized,sizeof(oversized),false,6500000); assert(sent==count);
    request(&p,88,false,6500001); assert(sent==count+1);
    memset(&p,0,sizeof(p)); hdzero_configure(&p,4,0,25,false);
    request(&p,88,false,0); assert(strcmp(hdzero_status(&p,1),"MSP_SETTINGS_SENT")==0);
    request(&p,88,false,UINT32_MAX-100U);
    assert(strcmp(hdzero_status(&p,100),"MSP_SETTINGS_SENT")==0);
    puts("HDZero MSP handshake, RF config, framing, armed guard and timeout: PASS");
    return 0;
}
