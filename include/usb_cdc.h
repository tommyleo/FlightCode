#pragma once

#include <stddef.h>
#include <stdint.h>

void usb_cdc_init(void);
void usb_cdc_deinit(void);
size_t usb_cdc_read(uint8_t *data, size_t capacity);
size_t usb_cdc_write(const uint8_t *data, size_t length);
void usb_cdc_poll(void);
