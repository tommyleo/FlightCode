#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
static char wire[2048];
static size_t wire_length;
static size_t usb_cdc_write(const uint8_t *data,size_t length){assert(wire_length+length<sizeof wire);memcpy(wire+wire_length,data,length);wire_length+=length;wire[wire_length]=0;return length;}
#include "protocol_reply_under_test.inc"
int main(void){
 char capabilities[520];memset(capabilities,'A',sizeof capabilities);memcpy(capabilities,"@CFG CAPABILITIES ",18);capabilities[sizeof capabilities-2]='\n';capabilities[sizeof capabilities-1]=0;
 reply("%s",capabilities);reply("@CFG PIDS 1 2 3\n");
 assert(wire_length==strlen(capabilities)+strlen("@CFG PIDS 1 2 3\n"));
 assert(memcmp(wire,capabilities,strlen(capabilities))==0);
 assert(strcmp(wire+strlen(capabilities),"@CFG PIDS 1 2 3\n")==0);
 wire_length=0;char oversized[900];memset(oversized,'X',sizeof oversized-1);oversized[sizeof oversized-1]=0;
 reply("%s",oversized);assert(strcmp(wire,"@CFG ERROR RESPONSE_TOO_LONG\n")==0);
 puts("PASS actual protocol reply: long capabilities retain newline and next packet; oversize response fails as a complete error line");
}
