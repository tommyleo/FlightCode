#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef size_t (*esc_passthrough_host_write_fn)(const uint8_t *data,
                                                size_t length);

void esc_passthrough_init(esc_passthrough_host_write_fn writer);
bool esc_passthrough_consume(uint8_t byte, bool armed);
bool esc_passthrough_active(void);
