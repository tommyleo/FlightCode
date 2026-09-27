#pragma once

#include <stdbool.h>
#include <stdint.h>

uint8_t esc_passthrough_count(void);
void esc_passthrough_begin(void);
void esc_passthrough_end(void);
void esc_passthrough_input(uint8_t index);
void esc_passthrough_output(uint8_t index);
bool esc_passthrough_read(uint8_t index);
void esc_passthrough_write(uint8_t index, bool high);
uint32_t esc_passthrough_micros(void);
uint32_t esc_passthrough_timing_now(void);
void esc_passthrough_wait_until(uint32_t started, uint32_t offset_us);
uint32_t esc_passthrough_critical_enter(void);
void esc_passthrough_critical_exit(uint32_t state);
