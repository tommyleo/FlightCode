#include "vtx_tramp.h"
#include <stdint.h>
#include <string.h>
#include "board.h"
#include "flight_settings.h"

#define FRAME_SIZE 16U
#define BAUD_RATE 9600U
#define START_DELAY_MS 1000U
#define REQUEST_PERIOD_MS 200U
#define RESPONSE_TIMEOUT_MS 300U
#define MAX_RETRIES 20U

typedef enum { IDLE, START_DELAY, WAIT_LIMITS, WAIT_STATUS,
               DELAY_STATUS_QUERY, DONE, FAILED } tramp_state_t;
typedef enum { STATUS_NOT_CONFIGURED, STATUS_INITIALIZING, STATUS_APPLIED,
               STATUS_INVALID_SETTINGS, STATUS_UART_ERROR, STATUS_NO_RESPONSE,
               STATUS_RACE_LOCKED } tramp_status_t;
#if defined(BOARD_MAMBAF411)
/* Mamba exposes only UART1, so VTX and receiver are mutually exclusive. */
#define uart hsbus_uart
#else
static UART_HandleTypeDef uart;
#endif
static uint8_t state;
static uint8_t status_code;
static uint16_t desired_frequency, desired_power;
static uint8_t response[FRAME_SIZE], response_position, retries;
static uint32_t deadline_ms;

static const uint16_t frequencies_eu[5][8] = {
 {5865,5845,5825,5805,5785,5765,5745,0},{5733,5752,5771,5790,5809,5828,5847,5866},
 {0,0,0,0,0,0,0,0},{5740,5760,5780,5800,5820,5840,5860,0},{0,0,5732,5769,5806,5843,0,0}};
static const uint16_t frequencies_us[5][8] = {
 {5865,5845,5825,5805,5785,5765,5745,5725},{5733,5752,5771,5790,5809,5828,5847,5866},
 {5705,5685,5665,0,5885,5905,0,0},{5740,5760,5780,5800,5820,5840,5860,5880},
 {5658,5695,5732,5769,5806,5843,5880,5917}};

const char *vtx_tramp_status_name(void)
{
 switch((tramp_status_t)status_code) {
 case STATUS_INITIALIZING: return "INITIALIZING";
 case STATUS_APPLIED: return "APPLIED";
 case STATUS_INVALID_SETTINGS: return "INVALID_SETTINGS";
 case STATUS_UART_ERROR: return "UART_ERROR";
 case STATUS_NO_RESPONSE: return "NO_RESPONSE";
 case STATUS_RACE_LOCKED: return "RACE_LOCKED";
 default: return "NOT_CONFIGURED";
 }
}
static uint8_t checksum(const uint8_t frame[FRAME_SIZE])
{ uint8_t result=0; for(uint32_t i=1;i<14;i++) result+=frame[i]; return result; }
static bool valid(uint8_t command)
{ return response[0]==0x0F && response[1]==command && response[14]==checksum(response) && response[15]==0; }
static void fail(tramp_status_t code) { status_code=(uint8_t)code; state=FAILED; }

static bool send_frame(uint8_t command, uint16_t value)
{
 uint8_t frame[FRAME_SIZE]={0}; frame[0]=0x0F; frame[1]=command;
 frame[2]=(uint8_t)value; frame[3]=(uint8_t)(value>>8); frame[14]=checksum(frame);
 if(HAL_HalfDuplex_EnableTransmitter(&uart)!=HAL_OK) return false;
 return HAL_UART_Transmit(&uart,frame,sizeof(frame),100U)==HAL_OK;
}
static bool begin_query(uint8_t command, tramp_state_t waiting)
{
 if(!send_frame(command,0)||HAL_HalfDuplex_EnableReceiver(&uart)!=HAL_OK) return false;
 memset(response,0,sizeof(response)); response_position=0; state=waiting;
 deadline_ms=HAL_GetTick()+RESPONSE_TIMEOUT_MS; return true;
}
static bool receive_byte(uint8_t *value)
{
#if defined(PLATFORM_STM32H7)
 if(__HAL_UART_GET_FLAG(&uart,UART_FLAG_RXNE)==RESET) return false;
 *value=(uint8_t)uart.Instance->RDR;
#else
 if(__HAL_UART_GET_FLAG(&uart,UART_FLAG_RXNE)==RESET) return false;
 *value=(uint8_t)uart.Instance->DR;
#endif
 return true;
}
static bool collect_response(void)
{
 uint8_t value; while(receive_byte(&value)) {
  if(response_position==0 && value!=0x0F) continue;
  response[response_position++]=value; if(response_position==FRAME_SIZE) return true;
 } return false;
}
static void schedule_status_query(void)
{ state=DELAY_STATUS_QUERY; deadline_ms=HAL_GetTick()+REQUEST_PERIOD_MS; }

bool vtx_tramp_init(void)
{
 const flight_settings_t *s=flight_settings_get();
 if(s->vtx_protocol!=VTX_PROTOCOL_TRAMP){status_code=STATUS_NOT_CONFIGURED;state=IDLE;return false;}
 if(s->vtx_band>=5||s->vtx_channel>=8){fail(STATUS_INVALID_SETTINGS);return false;}
 desired_frequency=s->vtx_region==VTX_REGION_US?frequencies_us[s->vtx_band][s->vtx_channel]:frequencies_eu[s->vtx_band][s->vtx_channel];
 if(!desired_frequency||!s->vtx_power_mw||s->vtx_power_mw>UINT16_MAX){fail(STATUS_INVALID_SETTINGS);return false;}
 desired_power=(uint16_t)s->vtx_power_mw;
 if(!board_uart_half_duplex_init((uint8_t)s->vtx_uart,BAUD_RATE,&uart)){fail(STATUS_UART_ERROR);return false;}
 retries=0;status_code=STATUS_INITIALIZING;state=START_DELAY;deadline_ms=HAL_GetTick()+START_DELAY_MS;return true;
}
static void retry_query(uint8_t command,tramp_state_t waiting)
{
 if(++retries>=MAX_RETRIES) fail(STATUS_NO_RESPONSE);
 else if(!begin_query(command,waiting)) fail(STATUS_UART_ERROR);
}
void vtx_tramp_update(bool armed)
{
 if(armed||state==IDLE||state==DONE||state==FAILED)return;
 uint32_t now=HAL_GetTick();
 if(state==START_DELAY){if((int32_t)(now-deadline_ms)>=0&&!begin_query('r',WAIT_LIMITS))fail(STATUS_UART_ERROR);return;}
 if(state==DELAY_STATUS_QUERY){if((int32_t)(now-deadline_ms)>=0&&!begin_query('v',WAIT_STATUS))fail(STATUS_UART_ERROR);return;}
 uint8_t expected=state==WAIT_LIMITS?'r':'v';
 if(!collect_response()){if((int32_t)(now-deadline_ms)>=0)retry_query(expected,state);return;}
 if(!valid(expected)){retry_query(expected,state);return;}
 retries=0;
 if(state==WAIT_LIMITS){if((response[2]==0&&response[3]==0)||!begin_query('v',WAIT_STATUS))fail(STATUS_NO_RESPONSE);return;}
 uint16_t frequency=(uint16_t)response[2]|((uint16_t)response[3]<<8);
 uint16_t power=(uint16_t)response[4]|((uint16_t)response[5]<<8);
 if(response[6]&1)fail(STATUS_RACE_LOCKED);
 else if(frequency!=desired_frequency){if(!send_frame('F',desired_frequency))fail(STATUS_UART_ERROR);else schedule_status_query();}
 else if(power!=desired_power){if(!send_frame('P',desired_power))fail(STATUS_UART_ERROR);else schedule_status_query();}
 else{status_code=STATUS_APPLIED;state=DONE;}
}
