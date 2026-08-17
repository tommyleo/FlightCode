#include "flight_log.h"

#include <math.h>
#include <string.h>

#include "board.h"
#include "blackbox_sd.h"
#include "flight_settings.h"
#include "imu.h"

#define FLIGHT_LOG_CAPACITY BOARD_FLIGHT_LOG_CAPACITY
#define CONTROL_LOOP_HZ 16000U
#define LOG_DECIMATION (CONTROL_LOOP_HZ / FLIGHT_LOG_RATE_HZ)
#define DSHOT_MIN 48U
#define DSHOT_MAX 2047U
#define LOG_FLASH_MAGIC 0x46344C47U
#define LOG_FLASH_VERSION 5U
#define LOG_PERSIST_DELAY_US 200000U
#define LOG_MIN_FLIGHT_THROTTLE_PERCENT 1.0f

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t count;
    uint32_t rate_hz;
    uint32_t record_size;
    uint32_t checksum;
    blackbox_sd_diagnostics_t blackbox_diagnostics;
    flight_log_metadata_t metadata;
} flight_log_flash_header_t;

static flight_log_record_t records[FLIGHT_LOG_CAPACITY];
static uint32_t write_index;
static uint32_t record_count;
static uint16_t decimation_count;
static bool recording;
static bool inhibited;
static bool using_flash;
static bool persist_pending;
static uint32_t persist_requested_us;
static bool flight_qualified;
static uint32_t preserved_flash_count;
static uint16_t battery_centivolts;
static uint16_t cell_centivolts;
static uint8_t battery_cells;
static flight_log_metadata_t flight_metadata;

static uint32_t hash_bytes(uint32_t hash, const void *data, size_t length)
{
    const uint8_t *bytes = (const uint8_t *)data;
    for (size_t i = 0U; i < length; ++i) {
        hash = (hash ^ bytes[i]) * 16777619U;
    }
    return hash;
}

static const flight_log_record_t *flash_records(void)
{
    return (const flight_log_record_t *)
        (FLIGHT_LOG_ADDRESS + sizeof(flight_log_flash_header_t));
}

static const flight_log_record_t *ram_record(uint32_t index)
{
    const uint32_t oldest =
        record_count == FLIGHT_LOG_CAPACITY ? write_index : 0U;
    return &records[(oldest + index) % FLIGHT_LOG_CAPACITY];
}

static int16_t scaled_i16(float value, float scale)
{
    const float scaled = value * scale;
    if (scaled > 32767.0f) return 32767;
    if (scaled < -32768.0f) return -32768;
    return (int16_t)lroundf(scaled);
}

static int8_t scaled_pid(float value)
{
    const float scaled = value * 2.0f;
    if (scaled > 127.0f) return 127;
    if (scaled < -128.0f) return -128;
    return (int8_t)lroundf(scaled);
}

void flight_log_init(void)
{
    write_index = 0U;
    record_count = 0U;
    decimation_count = 0U;
    recording = false;
    inhibited = false;
    using_flash = false;
    persist_pending = false;
    persist_requested_us = 0U;
    flight_qualified = false;
    preserved_flash_count = 0U;
    battery_centivolts = 0U;
    cell_centivolts = 0U;
    battery_cells = 0U;

    const flight_log_flash_header_t *const header =
        (const flight_log_flash_header_t *)FLIGHT_LOG_ADDRESS;
    if (header->magic == LOG_FLASH_MAGIC &&
        header->version == LOG_FLASH_VERSION &&
        header->count > 0U &&
        header->count <= FLIGHT_LOG_CAPACITY &&
        header->rate_hz == FLIGHT_LOG_RATE_HZ &&
        header->record_size == sizeof(flight_log_record_t)) {
        const uint32_t calculated =
            hash_bytes(2166136261U, flash_records(),
                       header->count * sizeof(flight_log_record_t));
        if (calculated == header->checksum) {
            record_count = header->count;
            using_flash = true;
            blackbox_sd_restore_diagnostics(
                &header->blackbox_diagnostics);
        }
    }
}

void flight_log_set_inhibited(bool value)
{
    inhibited = value;
    if (value) {
        recording = false;
    }
}

void flight_log_set_battery_voltage(float voltage)
{
    if (voltage < 1.0f || voltage >= 100.0f) {
        battery_centivolts = 0U;
        cell_centivolts = 0U;
        battery_cells = 0U;
        return;
    }
    uint8_t candidate_cells = (uint8_t)ceilf(voltage / 4.25f);
    if (candidate_cells > 8U) candidate_cells = 8U;
    if (candidate_cells > battery_cells) battery_cells = candidate_cells;
    battery_centivolts = (uint16_t)lroundf(voltage * 100.0f);
    cell_centivolts = battery_cells > 0U
        ? (uint16_t)lroundf(voltage * 100.0f / (float)battery_cells)
        : 0U;
}

void flight_log_start(void)
{
    if (inhibited) return;
    preserved_flash_count = using_flash ? record_count : 0U;
    using_flash = false;
    persist_pending = false;
    write_index = 0U;
    record_count = 0U;
    decimation_count = 0U;
    flight_qualified = false;
    const flight_settings_t *const settings = flight_settings_get();
    memset(&flight_metadata, 0, sizeof(flight_metadata));
    flight_metadata.version = FLIGHT_LOG_METADATA_VERSION;
    flight_metadata.main_loop_hz = settings->main_loop_hz;
    flight_metadata.gyro_rate_hz = imu_get_gyro_rate_hz();
    flight_metadata.log_rate_hz = FLIGHT_LOG_RATE_HZ;
    const pid_gains_t gains[3] = {settings->roll, settings->pitch, settings->yaw};
    for (uint8_t i = 0U; i < 3U; ++i) {
        flight_metadata.pids[i * 3U] = gains[i].kp;
        flight_metadata.pids[i * 3U + 1U] = gains[i].ki;
        flight_metadata.pids[i * 3U + 2U] = gains[i].kd;
    }
    flight_metadata.rates[0] = settings->roll_rate_dps;
    flight_metadata.rates[1] = settings->pitch_rate_dps;
    flight_metadata.rates[2] = settings->yaw_rate_dps;
    flight_metadata.rates[3] = settings->rate_expo;
    flight_metadata.feedforward[0] = settings->roll_feedforward;
    flight_metadata.feedforward[1] = settings->pitch_feedforward;
    flight_metadata.feedforward[2] = settings->yaw_feedforward;
    flight_metadata.tpa[0] = settings->tpa_attenuation;
    flight_metadata.tpa[1] = settings->tpa_breakpoint_percent;
    flight_metadata.filters[0] = settings->gyro_lpf_hz;
    flight_metadata.filters[1] = settings->dterm_lpf_hz;
    flight_metadata.alignment[0] = settings->board_roll_deg;
    flight_metadata.alignment[1] = settings->board_pitch_deg;
    flight_metadata.alignment[2] = settings->board_yaw_deg;
    flight_metadata.motor_idle_percent = settings->motor_idle_percent;
    flight_metadata.motor_protocol = settings->motor_protocol;
    flight_metadata.motor_direction_reversed = settings->motor_direction_reversed;
    flight_metadata.receiver_protocol = settings->receiver_protocol;
    flight_metadata.initial_battery_centivolts = battery_centivolts;
    flight_metadata.initial_battery_cells = battery_cells;
    recording = true;
}

void flight_log_stop(uint8_t stop_flag)
{
    const bool retain_blackbox = recording && flight_qualified;
    blackbox_sd_stop(stop_flag, retain_blackbox);
    if (recording && !flight_qualified) {
        recording = false;
        persist_pending = false;
        write_index = 0U;
        record_count = preserved_flash_count;
        using_flash = preserved_flash_count > 0U;
        return;
    }
    if (recording && record_count > 0U) {
        /*
         * Always retain the event that ended the flight.  The buffer remains
         * circular, so on long flights this replaces the oldest sample.
         */
        flight_log_record_t *const marker = &records[write_index];
        *marker = *ram_record(record_count - 1U);
        memset(marker->motor, 0, sizeof(marker->motor));
        marker->flags =
            (marker->flags & FLIGHT_LOG_FLAG_MIXER_SATURATED) | stop_flag;
        write_index = (write_index + 1U) % FLIGHT_LOG_CAPACITY;
        if (record_count < FLIGHT_LOG_CAPACITY) ++record_count;
        persist_pending = true;
        persist_requested_us = board_micros();
    }
    recording = false;
}

void flight_log_stop_rx(bool failsafe)
{
    flight_log_stop(FLIGHT_LOG_FLAG_STOP_RX_LOSS |
                    (failsafe ? FLIGHT_LOG_FLAG_STOP_RX_FAILSAFE
                              : FLIGHT_LOG_FLAG_STOP_RX_TIMEOUT));
}

bool flight_log_is_recording(void) { return recording; }
bool flight_log_is_available(void) { return !recording && record_count > 0U; }
uint32_t flight_log_count(void) { return record_count; }

bool flight_log_get(uint32_t index, flight_log_record_t *record)
{
    if (recording || index >= record_count || record == NULL) return false;
    *record = using_flash ? flash_records()[index] : *ram_record(index);
    return true;
}

bool flight_log_get_metadata(flight_log_metadata_t *metadata)
{
    if (metadata == NULL) return false;
    if (using_flash) {
        const flight_log_flash_header_t *const header =
            (const flight_log_flash_header_t *)FLIGHT_LOG_ADDRESS;
        if (header->version != LOG_FLASH_VERSION) return false;
        *metadata = header->metadata;
    } else {
        *metadata = flight_metadata;
    }
    return metadata->version == FLIGHT_LOG_METADATA_VERSION;
}

bool flight_log_persist_pending(void)
{
    return persist_pending;
}

void flight_log_persist_if_ready(void)
{
    if (!persist_pending || recording ||
        (uint32_t)(board_micros() - persist_requested_us) <
            LOG_PERSIST_DELAY_US) {
        return;
    }

    flight_log_flash_header_t header = {
        .magic = LOG_FLASH_MAGIC,
        .version = LOG_FLASH_VERSION,
        .count = record_count,
        .rate_hz = FLIGHT_LOG_RATE_HZ,
        .record_size = sizeof(flight_log_record_t),
        .checksum = 2166136261U,
    };
    blackbox_sd_get_diagnostics(&header.blackbox_diagnostics);
    header.metadata = flight_metadata;
    for (uint32_t i = 0U; i < record_count; ++i) {
        header.checksum =
            hash_bytes(header.checksum, ram_record(i), sizeof(*ram_record(i)));
    }

    board_status_led_set(true);
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef erase = {
        .TypeErase = FLASH_TYPEERASE_SECTORS,
        .Sector = FLIGHT_LOG_FLASH_SECTOR,
        .NbSectors = 1U,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3,
    };
    uint32_t sector_error = 0U;
    bool ok = HAL_FLASHEx_Erase(&erase, &sector_error) == HAL_OK;

    const uint32_t *header_words = (const uint32_t *)&header;
    for (uint32_t i = 0U;
         ok && i < sizeof(header) / sizeof(uint32_t); ++i) {
        ok = HAL_FLASH_Program(
                 FLASH_TYPEPROGRAM_WORD,
                 FLIGHT_LOG_ADDRESS + i * sizeof(uint32_t),
                 header_words[i]) == HAL_OK;
    }
    uint32_t address = FLIGHT_LOG_ADDRESS + sizeof(header);
    for (uint32_t i = 0U; ok && i < record_count; ++i) {
        for (uint32_t word = 0U;
             ok && word < sizeof(flight_log_record_t) / sizeof(uint32_t);
             ++word) {
            uint32_t value;
            memcpy(&value,
                   (const uint8_t *)ram_record(i) +
                       word * sizeof(uint32_t),
                   sizeof(value));
            ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address,
                                   value) == HAL_OK;
            address += sizeof(uint32_t);
        }
    }
    HAL_FLASH_Lock();
    persist_pending = false;
    using_flash = ok;
    board_status_led_set(false);
    if (ok) {
        board_buzzer_set(true);
        HAL_Delay(120U);
        board_buzzer_set(false);
    }
}

void flight_log_record(const float gyro[3], const float setpoint[3],
                       const float pid[3], const float p_term[3],
                       const float i_term[3], const float d_term[3],
                       const float ff_term[3],
                       const uint16_t motors[4],
                       float throttle_percent, bool mixer_saturated,
                       uint16_t loop_us)
{
    if (!recording || inhibited) return;
    if (!flight_qualified &&
        throttle_percent > LOG_MIN_FLIGHT_THROTTLE_PERCENT) {
        flight_qualified = true;
        /* Start persistent logging only once this is a real flight. */
        blackbox_sd_start(&flight_metadata);
    }
    if (++decimation_count < LOG_DECIMATION) return;
    decimation_count = 0U;

    flight_log_record_t *const item = &records[write_index];
    for (uint8_t i = 0U; i < 3U; ++i) {
        item->gyro[i] = scaled_i16(gyro[i], 10.0f);
        item->setpoint[i] = scaled_i16(setpoint[i], 10.0f);
        item->pid[i] = scaled_pid(pid[i]);
        item->p_term[i] = scaled_pid(p_term[i]);
        item->i_term[i] = scaled_pid(i_term[i]);
        item->d_term[i] = scaled_pid(d_term[i]);
        item->ff_term[i] = scaled_pid(ff_term[i]);
    }
    for (uint8_t i = 0U; i < 4U; ++i) {
        float percent = motors[i] == 0U ? 0.0f :
            (float)(motors[i] - DSHOT_MIN) * 100.0f /
            (float)(DSHOT_MAX - DSHOT_MIN);
        if (percent < 0.0f) percent = 0.0f;
        if (percent > 100.0f) percent = 100.0f;
        item->motor[i] = (uint8_t)lroundf(percent * 2.55f);
    }
    if (throttle_percent < 0.0f) throttle_percent = 0.0f;
    if (throttle_percent > 100.0f) throttle_percent = 100.0f;
    item->throttle = (uint8_t)lroundf(throttle_percent * 2.0f);
    item->flags =
        mixer_saturated ? FLIGHT_LOG_FLAG_MIXER_SATURATED : 0U;
    item->loop_us = loop_us;
    item->battery_centivolts = battery_centivolts;
    item->cell_centivolts = cell_centivolts;
    item->battery_cells = battery_cells;
    if (flight_qualified) blackbox_sd_append(item);

    write_index = (write_index + 1U) % FLIGHT_LOG_CAPACITY;
    if (record_count < FLIGHT_LOG_CAPACITY) ++record_count;
}
