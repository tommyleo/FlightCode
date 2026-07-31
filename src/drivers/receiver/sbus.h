#pragma once

#include <stdbool.h>
#include <stdint.h>

#define SBUS_CHANNEL_COUNT 16U

typedef enum {
    SBUS_LOSS_NONE = 0,
    SBUS_LOSS_FAILSAFE,
    SBUS_LOSS_TIMEOUT
} sbus_loss_reason_t;

typedef struct {
    uint16_t channel_us[SBUS_CHANNEL_COUNT];
    bool valid;
    bool failsafe;
    uint32_t last_frame_us;
    uint32_t frame_age_us;
    uint32_t valid_frame_count;
    uint32_t uart_error_count;
    uint32_t recovery_count;
    sbus_loss_reason_t loss_reason;
} sbus_data_t;

void sbus_init(void);
void sbus_update(void);
const sbus_data_t *sbus_get(void);
