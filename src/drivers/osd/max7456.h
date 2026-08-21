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
bool max7456_set_layout(uint32_t enabled_mask,
                        const uint32_t positions[5],
                        const char *pilot_name);
void max7456_update(float voltage, bool armed, uint32_t now_us);
bool max7456_menu_begin(void);
void max7456_menu_end(void);
void max7456_clear_screen(void);
void max7456_write_text(uint8_t row, uint8_t column, const char *text);
