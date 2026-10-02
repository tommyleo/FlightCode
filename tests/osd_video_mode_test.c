#include <assert.h>
#include "../src/drivers/osd/max7456.c"
int main(void){
assert(max7456_set_video_mode(OSD_VIDEO_PAL)); assert(max7456_init());
assert(max7456_video_is_pal()); assert(vm0 & VM0_PAL);
assert(max7456_set_video_mode(OSD_VIDEO_NTSC)); assert(!(vm0 & VM0_PAL));
status=STAT_PAL; assert(max7456_init()); assert(!max7456_video_is_pal());
assert(max7456_set_video_mode(3)==false); assert(!max7456_video_is_pal());
unsigned before=writes; write_character(389,'A'); assert(writes==before+1); assert(dmah==1 && dmal==133);
write_character(390,'A'); assert(writes==before+1);
assert(max7456_set_video_mode(OSD_VIDEO_PAL)); write_character(479,'A'); assert(writes==before+2);assert(dmah==1 && dmal==223);
status=STAT_NTSC; assert(max7456_set_video_mode(OSD_VIDEO_AUTO));assert(!max7456_video_is_pal());
status=STAT_PAL;assert(max7456_init());assert(max7456_video_is_pal());
puts("OSD forced PAL/NTSC startup, live changes, invalid mode, auto and row bounds passed");
}
