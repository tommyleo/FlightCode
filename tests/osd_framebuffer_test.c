#include <assert.h>
#include <stdio.h>
#include "osd_framebuffer.h"
int main(void) {
    osd_framebuffer_t frame;
    char screen[OSD_FRAME_CELLS];
    osd_framebuffer_reset(&frame);
    memset(screen, 'A', sizeof(screen));
    osd_framebuffer_publish(&frame, screen);
    unsigned writes=0, calls=0;
    uint16_t position; char character;
    while(frame.remaining) {
        uint16_t before=frame.remaining;
        if(osd_framebuffer_next(&frame,&position,&character)) {
            assert(position<OSD_FRAME_CELLS && character=='A'); ++writes;
        }
        assert(before-frame.remaining<=OSD_SCAN_BUDGET); ++calls;
    }
    assert(writes==480 && calls==480);
    memset(screen,' ',sizeof(screen)); screen[479]='Z';
    osd_framebuffer_publish(&frame,screen);
    for(unsigned i=0;i<40;++i) osd_framebuffer_next(&frame,&position,&character);
    screen[0]='B'; screen[250]='C';
    osd_framebuffer_publish(&frame,screen);
    while(frame.remaining) osd_framebuffer_next(&frame,&position,&character);
    assert(memcmp(frame.displayed,screen,sizeof(screen))==0);
    osd_framebuffer_publish(&frame,screen); calls=0;
    while(frame.remaining) {assert(!osd_framebuffer_next(&frame,&position,&character));++calls;}
    assert(calls==30);
    osd_framebuffer_reset(&frame);
    assert(!osd_framebuffer_next(&frame,&position,&character));
    puts("OSD bounded writes, replacement, erasure and reset passed");
}
