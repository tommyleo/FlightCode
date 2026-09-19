#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define OSD_FRAME_CELLS 480U
#define OSD_SCAN_BUDGET 16U

typedef struct {
    char displayed[OSD_FRAME_CELLS];
    char desired[OSD_FRAME_CELLS];
    uint16_t cursor;
    uint16_t remaining;
} osd_framebuffer_t;

static inline void osd_framebuffer_reset(osd_framebuffer_t *frame)
{
    memset(frame->displayed, ' ', sizeof(frame->displayed));
    memset(frame->desired, ' ', sizeof(frame->desired));
    frame->cursor = 0U;
    frame->remaining = 0U;
}

static inline void osd_framebuffer_publish(osd_framebuffer_t *frame,
                                           const char screen[OSD_FRAME_CELLS])
{
    memcpy(frame->desired, screen, sizeof(frame->desired));
    frame->cursor = 0U;
    frame->remaining = OSD_FRAME_CELLS;
}

/* One character write at most, and a bounded scan even for an empty screen. */
static inline bool osd_framebuffer_next(osd_framebuffer_t *frame,
                                       uint16_t *position, char *character)
{
    for (unsigned scanned = 0U;
         scanned < OSD_SCAN_BUDGET && frame->remaining != 0U; ++scanned) {
        const uint16_t cell = frame->cursor++;
        --frame->remaining;
        if (frame->displayed[cell] != frame->desired[cell]) {
            *position = cell;
            *character = frame->desired[cell];
            frame->displayed[cell] = *character;
            return true;
        }
    }
    return false;
}
