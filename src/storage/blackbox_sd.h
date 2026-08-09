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

typedef struct {
    uint32_t flight_id;
    uint32_t record_count;
    uint32_t block_count;
    uint8_t stop_flag;
} blackbox_sd_flight_info_t;

void blackbox_sd_init(void);
void blackbox_sd_probe(void);
void blackbox_sd_update(void);
void blackbox_sd_set_enabled(bool enabled);
bool blackbox_sd_is_enabled(void);
bool blackbox_sd_is_busy(void);
blackbox_sd_state_t blackbox_sd_state(void);
const char *blackbox_sd_state_name(void);
uint32_t blackbox_sd_capacity_mb(void);
uint32_t blackbox_sd_written_bytes(void);
uint32_t blackbox_sd_dropped_records(void);
uint32_t blackbox_sd_total_bytes(void);
uint32_t blackbox_sd_flight_count(void);
bool blackbox_sd_get_flight(uint32_t index,
                            blackbox_sd_flight_info_t *info);
bool blackbox_sd_get_record(uint32_t flight_id, uint32_t record_index,
                            flight_log_record_t *record);
bool blackbox_sd_clear(void);
void blackbox_sd_start(void);
void blackbox_sd_append(const flight_log_record_t *record);
void blackbox_sd_stop(uint8_t stop_flag, bool retain);
