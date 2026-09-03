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
    uint16_t sample_rate_hz;
} blackbox_sd_flight_info_t;

typedef struct {
    uint32_t jedec_id;
    uint32_t start_calls;
    uint32_t start_reject_mask;
    uint32_t append_calls;
    uint32_t stop_calls;
    uint32_t completed_records;
    uint8_t last_retain;
    uint8_t state;
    uint8_t operation;
    uint8_t queue_count;
    uint8_t write_bank;
    uint8_t retained_bank;
    uint8_t erase_active;
    uint8_t finalise_pending;
    uint8_t error_code;
    uint8_t error_operation;
    uint8_t last_status;
    uint32_t error_address;
} blackbox_sd_diagnostics_t;

typedef struct {
    uint32_t address;
    uint8_t before_erased;
    uint8_t program_ok;
    uint8_t read_ok;
    uint8_t verify_ok;
    uint8_t mismatch_index;
    uint8_t expected;
    uint8_t actual;
} blackbox_sd_write_test_t;

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
                            blackbox_record_t *record);
uint32_t blackbox_sd_get_records(uint32_t flight_id, uint32_t record_index,
                                 blackbox_record_t *records,
                                 uint32_t capacity,
                                 uint32_t *failed_sector);
bool blackbox_sd_get_metadata(uint32_t flight_id,
                              flight_log_metadata_t *metadata);
bool blackbox_sd_clear(void);
void blackbox_sd_start(const flight_log_metadata_t *metadata);
void blackbox_sd_append(const blackbox_record_t *record);
void blackbox_sd_stop(uint8_t stop_flag, bool retain);
void blackbox_sd_get_diagnostics(blackbox_sd_diagnostics_t *diagnostics);
void blackbox_sd_restore_diagnostics(
    const blackbox_sd_diagnostics_t *diagnostics);
bool blackbox_sd_write_test(blackbox_sd_write_test_t *result);
bool blackbox_sd_session_test(void);
