#pragma once
#include <stdint.h>
#define OSD_VIDEO_AUTO 0U
#define OSD_VIDEO_PAL 1U
#define OSD_VIDEO_NTSC 2U
typedef struct {uint32_t current_osd_enabled,current_osd_position,vtx_band,vtx_channel,vtx_power_mw,vtx_osd_enabled,vtx_osd_position;} flight_settings_t;
static flight_settings_t fake_settings;
static inline const flight_settings_t *flight_settings_get(void){return &fake_settings;}
