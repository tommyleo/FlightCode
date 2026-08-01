#pragma once

#include <stdbool.h>
#include <stdint.h>

bool max7456_init(void);
bool max7456_is_available(void);
bool max7456_font_is_ready(void);
uint8_t max7456_probe_value(void);
uint8_t max7456_probe_spi_mode(void);
bool max7456_video_is_pal(void);
bool max7456_is_enabled(void);
uint8_t max7456_position(void);
bool max7456_set_config(bool enabled, uint8_t position);
void max7456_update_battery(float voltage);
