#pragma once
#include <stdint.h>
#define VTX_PROTOCOL_SMARTAUDIO 1U
#define VTX_REGION_EU 0U
#define VTX_REGION_US 1U
typedef struct { uint32_t vtx_protocol, vtx_band, vtx_channel, vtx_region,
                         vtx_power_mw, vtx_uart; } flight_settings_t;
const flight_settings_t *flight_settings_get(void);
