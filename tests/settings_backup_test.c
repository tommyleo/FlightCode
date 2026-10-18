#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
typedef struct { uint32_t pid, alignment, hidden; char osd_pilot_name[13]; } flight_settings_t;
static flight_settings_t settings={123,456,789,"TEST"};
static char response[512];
static unsigned saves,sets;
static bool save_ok=true,values_ok=true;
static uint32_t now;
static const flight_settings_t *flight_settings_get(void){return &settings;}
static bool flight_settings_are_saved(void){return true;}
static uint32_t flight_settings_schema_version(void){return 28;}
static bool flight_settings_set(const flight_settings_t *value){if(!values_ok)return false;settings=*value;++sets;return true;}
static bool flight_settings_save(void){++saves;return save_ok;}
static uint32_t clock_us(void){return now;}
static void reply(const char *fmt,...){va_list args;va_start(args,fmt);vsnprintf(response,sizeof response,fmt,args);va_end(args);}
#define BACKUP_REPLY reply
#define BACKUP_FAMILY "FlightCode"
#define BACKUP_BOARD "SEQUREH7V2"
#define BACKUP_NOW_US clock_us
#include "settings_backup_protocol.h"
static void command(const char *value,bool unsafe){response[0]=0;assert(settings_backup_process(value,unsafe));}
static void begin(const flight_settings_t *value){char cmd[160];snprintf(cmd,sizeof cmd,"RESTORE_SETTINGS_BEGIN FlightCode SEQUREH7V2 28 %lu %08lX",(unsigned long)sizeof *value,(unsigned long)settings_backup_crc(value,sizeof *value));command(cmd,false);assert(strstr(response,"OK RESTORE_SETTINGS_BEGIN"));}
static void chunk(const flight_settings_t *value){char cmd[192],hex[sizeof *value*2+1];const uint8_t *p=(const uint8_t *)value;for(size_t i=0;i<sizeof *value;i++)sprintf(hex+i*2,"%02X",p[i]);snprintf(cmd,sizeof cmd,"RESTORE_SETTINGS_CHUNK 0 %s",hex);command(cmd,false);assert(strstr(response,"OK RESTORE_SETTINGS_CHUNK"));}
int main(void){
 assert(settings_backup_crc("123456789",9)==0xCBF43926U);
 assert(!settings_backup_process("GET_PIDS",false));
 command("GET_SETTINGS_CHUNK 0 1",false);assert(strstr(response,"ERROR"));
 command("GET_SETTINGS_INFO",false);assert(strstr(response,"SETTINGS_INFO FlightCode SEQUREH7V2 28"));
 flight_settings_t original=settings;settings.hidden=999;
 command("GET_SETTINGS_CHUNK 0 28",false);assert(strstr(response,"SETTINGS_CHUNK 0"));assert(backup_snapshot.hidden==789); /* Stable snapshot across changes. */
 command("GET_SETTINGS_CHUNK 99999 1",false);assert(strstr(response,"RANGE"));
 command("GET_SETTINGS_CHUNK 0 49",false);assert(strstr(response,"RANGE"));
 command("RESTORE_SETTINGS_BEGIN FlightCode OTHER 28 28 0",false);assert(strstr(response,"INCOMPATIBLE"));
 command("RESTORE_SETTINGS_BEGIN FlightCodePI SEQUREH7V2 28 28 0",false);assert(strstr(response,"INCOMPATIBLE"));
 begin(&original);command("RESTORE_SETTINGS_COMMIT",false);assert(strstr(response,"INTEGRITY")&&saves==0&&settings.hidden==999);
 begin(&original);command("RESTORE_SETTINGS_CHUNK 1 AA",false);assert(strstr(response,"SEQUENCE"));
 begin(&original);command("RESTORE_SETTINGS_CHUNK 0 GG",false);assert(strstr(response,"HEX"));
 begin(&original);command("RESTORE_SETTINGS_CHUNK 0 A",false);assert(strstr(response,"SEQUENCE"));
 begin(&original);chunk(&original);command("RESTORE_SETTINGS_COMMIT",true);assert(strstr(response,"BUSY")&&saves==0);
 begin(&original);chunk(&original);now+=SETTINGS_BACKUP_TIMEOUT_US+1;command("RESTORE_SETTINGS_COMMIT",false);assert(strstr(response,"INTEGRITY")&&saves==0);
 begin(&original);chunk(&original);restore_staging.hidden++;command("RESTORE_SETTINGS_COMMIT",false);assert(strstr(response,"INTEGRITY")&&saves==0);
 begin(&original);chunk(&original);values_ok=false;command("RESTORE_SETTINGS_COMMIT",false);assert(strstr(response,"VALUES")&&saves==0&&settings.hidden==999);values_ok=true;
 flight_settings_t invalid=original;memset(invalid.osd_pilot_name,'X',sizeof invalid.osd_pilot_name);
 begin(&invalid);chunk(&invalid);command("RESTORE_SETTINGS_COMMIT",false);assert(strstr(response,"INTEGRITY")&&saves==0);
 begin(&original);chunk(&original);save_ok=false;command("RESTORE_SETTINGS_COMMIT",false);assert(strstr(response,"SAVE_FAILED")&&settings.hidden==999);save_ok=true;
 begin(&original);chunk(&original);command("RESTORE_SETTINGS_COMMIT",false);assert(strstr(response,"OK RESTORE_SETTINGS_COMMIT REBOOT_REQUIRED")&&memcmp(&settings,&original,sizeof settings)==0);
 unsigned before=saves;command("RESTORE_SETTINGS_COMMIT",false);assert(strstr(response,"INTEGRITY")&&saves==before);
 begin(&original);command("RESTORE_SETTINGS_CANCEL",false);assert(!restore_active);
 command("RESTORE_SETTINGS_BEGIN bad data",false);assert(strstr(response,"COMMAND"));
 command("GET_SETTINGS_INFO",true);assert(strstr(response,"BUSY"));
 puts("PASS complete firmware snapshot, CRC, compatibility, ordered chunks, atomic validation, arming, timeout, errors, save failure and full restore");
}
