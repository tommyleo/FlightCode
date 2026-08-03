#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "flight_log.h"

typedef enum {
    BLACKBOX_SD_UNSUPPORTED = 0,
    BLACKBOX_SD_ABSENT,
    BLACKBOX_SD_INITIALIZING,
    BLACKBOX_SD_READY,
    BLACKBOX_SD_RECORDING,
    BLACKBOX_SD_ERROR
} blackbox_sd_state_t;

void blackbox_sd_init(void);
void blackbox_sd_probe(void);
void blackbox_sd_update(void);
void blackbox_sd_set_enabled(bool enabled);
bool blackbox_sd_is_enabled(void);
blackbox_sd_state_t blackbox_sd_state(void);
const char *blackbox_sd_state_name(void);
uint32_t blackbox_sd_capacity_mb(void);
uint32_t blackbox_sd_written_bytes(void);
uint32_t blackbox_sd_dropped_records(void);
void blackbox_sd_start(void);
void blackbox_sd_append(const flight_log_record_t *record);
void blackbox_sd_stop(uint8_t stop_flag, bool retain);
