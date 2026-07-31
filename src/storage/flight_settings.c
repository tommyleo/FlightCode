#include "flight_settings.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "board.h"
#include "mpu6000.h"
#include "stm32f4xx_hal.h"

#define SETTINGS_MAGIC 0x46344643U
#define SETTINGS_VERSION 10U
#define SETTINGS_LEGACY_VERSION 7U
#define SETTINGS_LEGACY_VERSION_8 8U
#define SETTINGS_LEGACY_VERSION_9 9U

typedef struct {
    pid_gains_t roll;
    pid_gains_t pitch;
    pid_gains_t yaw;
    motor_protocol_t motor_protocol;
    float board_roll_deg;
    float board_pitch_deg;
    float board_yaw_deg;
    uint32_t motor_direction_reversed;
    float motor_idle_percent;
} legacy_settings_v7_t;

typedef struct {
    uint32_t magic;
    uint32_t version;
    legacy_settings_v7_t settings;
    uint32_t checksum;
} legacy_record_v7_t;

typedef struct {
    legacy_settings_v7_t base;
    float roll_rate_dps;
    float pitch_rate_dps;
    float yaw_rate_dps;
    float rate_expo;
} legacy_settings_v8_t;

typedef struct {
    uint32_t magic;
    uint32_t version;
    legacy_settings_v8_t settings;
    uint32_t checksum;
} legacy_record_v8_t;

typedef struct {
    legacy_settings_v8_t base;
    float roll_feedforward;
    float pitch_feedforward;
    float yaw_feedforward;
} legacy_settings_v9_t;

typedef struct {
    uint32_t magic;
    uint32_t version;
    legacy_settings_v9_t settings;
    uint32_t checksum;
} legacy_record_v9_t;

typedef struct {
    uint32_t magic;
    uint32_t version;
    flight_settings_t settings;
    uint32_t checksum;
} settings_record_t;

static flight_settings_t current_settings;
static bool settings_saved;

static uint32_t checksum(const settings_record_t *record)
{
    const uint8_t *bytes = (const uint8_t *)record;
    uint32_t hash = 2166136261U;
    for (size_t i = 0; i < offsetof(settings_record_t, checksum); ++i) {
        hash = (hash ^ bytes[i]) * 16777619U;
    }
    return hash;
}

static uint32_t checksum_bytes(const void *data, size_t length)
{
    const uint8_t *bytes = (const uint8_t *)data;
    uint32_t hash = 2166136261U;
    for (size_t i = 0; i < length; ++i) {
        hash = (hash ^ bytes[i]) * 16777619U;
    }
    return hash;
}

static bool gains_valid(const pid_gains_t *gains)
{
    return isfinite(gains->kp) && isfinite(gains->ki) && isfinite(gains->kd) &&
           gains->kp >= 0.0f && gains->kp <= 1000.0f &&
           gains->ki >= 0.0f && gains->ki <= 1000.0f &&
           gains->kd >= 0.0f && gains->kd <= 1000.0f;
}

static bool angle_valid(float angle)
{
    return isfinite(angle) && angle >= -180.0f && angle <= 180.0f;
}

static bool rates_valid(const flight_settings_t *settings)
{
    return isfinite(settings->roll_rate_dps) &&
           isfinite(settings->pitch_rate_dps) &&
           isfinite(settings->yaw_rate_dps) &&
           isfinite(settings->rate_expo) &&
           settings->roll_rate_dps >= 100.0f &&
           settings->roll_rate_dps <= 1200.0f &&
           settings->pitch_rate_dps >= 100.0f &&
           settings->pitch_rate_dps <= 1200.0f &&
           settings->yaw_rate_dps >= 100.0f &&
           settings->yaw_rate_dps <= 1200.0f &&
           settings->rate_expo >= 0.0f && settings->rate_expo <= 0.9f;
}

static bool feedforward_valid(const flight_settings_t *settings)
{
    return isfinite(settings->roll_feedforward) &&
           isfinite(settings->pitch_feedforward) &&
           isfinite(settings->yaw_feedforward) &&
           settings->roll_feedforward >= 0.0f &&
           settings->roll_feedforward <= 1.0f &&
           settings->pitch_feedforward >= 0.0f &&
           settings->pitch_feedforward <= 1.0f &&
           settings->yaw_feedforward >= 0.0f &&
           settings->yaw_feedforward <= 1.0f;
}

static void apply(void)
{
    flight_control_set_gains(&current_settings.roll,
                             &current_settings.pitch,
                             &current_settings.yaw);
    flight_control_set_rates(current_settings.roll_rate_dps,
                             current_settings.pitch_rate_dps,
                             current_settings.yaw_rate_dps,
                             current_settings.rate_expo);
    flight_control_set_feedforward(current_settings.roll_feedforward,
                                   current_settings.pitch_feedforward,
                                   current_settings.yaw_feedforward);
    flight_control_set_tpa(current_settings.tpa_attenuation,
                           current_settings.tpa_breakpoint_percent);
    motor_protocol_set(current_settings.motor_protocol);
    mpu6000_set_board_alignment(current_settings.board_roll_deg,
                                current_settings.board_pitch_deg,
                                current_settings.board_yaw_deg);
    flight_control_set_motor_direction_reversed(
        current_settings.motor_direction_reversed != 0U);
    flight_control_set_motor_idle_percent(current_settings.motor_idle_percent);
}

void flight_settings_reset_defaults(void)
{
    current_settings = (flight_settings_t){
        .roll = {0.090f, 0.200f, 0.0012f},
        .pitch = {0.090f, 0.200f, 0.0012f},
        .yaw = {0.120f, 0.200f, 0.0f},
        .motor_protocol = MOTOR_PROTOCOL_DSHOT300,
        .board_roll_deg = 0.0f,
        .board_pitch_deg = 0.0f,
        .board_yaw_deg = 0.0f,
        .motor_direction_reversed = 0U,
        .motor_idle_percent = 3.0f,
        .roll_rate_dps = 500.0f,
        .pitch_rate_dps = 500.0f,
        .yaw_rate_dps = 400.0f,
        .rate_expo = 0.35f,
        .roll_feedforward = 0.025f,
        .pitch_feedforward = 0.025f,
        .yaw_feedforward = 0.015f,
        .tpa_attenuation = 0.0f,
        .tpa_breakpoint_percent = 65.0f,
    };
    settings_saved = false;
    apply();
}

void flight_settings_init(void)
{
    const settings_record_t *stored = (const settings_record_t *)SETTINGS_ADDRESS;
    const legacy_record_v7_t *legacy =
        (const legacy_record_v7_t *)SETTINGS_ADDRESS;
    const legacy_record_v8_t *legacy_v8 =
        (const legacy_record_v8_t *)SETTINGS_ADDRESS;
    const legacy_record_v9_t *legacy_v9 =
        (const legacy_record_v9_t *)SETTINGS_ADDRESS;
    if (legacy_v9->magic == SETTINGS_MAGIC &&
        legacy_v9->version == SETTINGS_LEGACY_VERSION_9 &&
        legacy_v9->checksum ==
            checksum_bytes(legacy_v9,
                           offsetof(legacy_record_v9_t, checksum))) {
        flight_settings_reset_defaults();
        memcpy(&current_settings, &legacy_v9->settings,
               sizeof(legacy_v9->settings));
        settings_saved = false;
        apply();
        return;
    }
    if (legacy_v8->magic == SETTINGS_MAGIC &&
        legacy_v8->version == SETTINGS_LEGACY_VERSION_8 &&
        legacy_v8->checksum ==
            checksum_bytes(legacy_v8,
                           offsetof(legacy_record_v8_t, checksum))) {
        flight_settings_reset_defaults();
        memcpy(&current_settings, &legacy_v8->settings,
               sizeof(legacy_v8->settings));
        settings_saved = false;
        apply();
        return;
    }
    if (legacy->magic == SETTINGS_MAGIC &&
        legacy->version == SETTINGS_LEGACY_VERSION &&
        legacy->checksum ==
            checksum_bytes(legacy, offsetof(legacy_record_v7_t, checksum))) {
        flight_settings_reset_defaults();
        current_settings.roll = legacy->settings.roll;
        current_settings.pitch = legacy->settings.pitch;
        current_settings.yaw = legacy->settings.yaw;
        current_settings.motor_protocol = legacy->settings.motor_protocol;
        current_settings.board_roll_deg = legacy->settings.board_roll_deg;
        current_settings.board_pitch_deg = legacy->settings.board_pitch_deg;
        current_settings.board_yaw_deg = legacy->settings.board_yaw_deg;
        current_settings.motor_direction_reversed =
            legacy->settings.motor_direction_reversed;
        current_settings.motor_idle_percent =
            legacy->settings.motor_idle_percent;
        settings_saved = false;
        apply();
        return;
    }
    if (stored->magic != SETTINGS_MAGIC ||
        stored->version != SETTINGS_VERSION ||
        stored->checksum != checksum(stored) ||
        !gains_valid(&stored->settings.roll) ||
        !gains_valid(&stored->settings.pitch) ||
        !gains_valid(&stored->settings.yaw) ||
        !rates_valid(&stored->settings) ||
        !feedforward_valid(&stored->settings) ||
        !isfinite(stored->settings.tpa_attenuation) ||
        stored->settings.tpa_attenuation < 0.0f ||
        stored->settings.tpa_attenuation > 1.0f ||
        !isfinite(stored->settings.tpa_breakpoint_percent) ||
        stored->settings.tpa_breakpoint_percent < 0.0f ||
        stored->settings.tpa_breakpoint_percent > 100.0f ||
        !angle_valid(stored->settings.board_roll_deg) ||
        !angle_valid(stored->settings.board_pitch_deg) ||
        !angle_valid(stored->settings.board_yaw_deg) ||
        stored->settings.motor_direction_reversed > 1U ||
        !isfinite(stored->settings.motor_idle_percent) ||
        stored->settings.motor_idle_percent < 1.0f ||
        stored->settings.motor_idle_percent > 10.0f ||
        (stored->settings.motor_protocol != MOTOR_PROTOCOL_DSHOT300 &&
         stored->settings.motor_protocol != MOTOR_PROTOCOL_ONESHOT125 &&
         stored->settings.motor_protocol != MOTOR_PROTOCOL_MULTISHOT)) {
        flight_settings_reset_defaults();
        return;
    }
    current_settings = stored->settings;
    settings_saved = true;
    apply();
}

const flight_settings_t *flight_settings_get(void)
{
    return &current_settings;
}

bool flight_settings_set(const flight_settings_t *settings)
{
    if (!gains_valid(&settings->roll) ||
        !gains_valid(&settings->pitch) ||
        !gains_valid(&settings->yaw) ||
        !rates_valid(settings) ||
        !feedforward_valid(settings) ||
        !isfinite(settings->tpa_attenuation) ||
        settings->tpa_attenuation < 0.0f ||
        settings->tpa_attenuation > 1.0f ||
        !isfinite(settings->tpa_breakpoint_percent) ||
        settings->tpa_breakpoint_percent < 0.0f ||
        settings->tpa_breakpoint_percent > 100.0f ||
        !angle_valid(settings->board_roll_deg) ||
        !angle_valid(settings->board_pitch_deg) ||
        !angle_valid(settings->board_yaw_deg) ||
        settings->motor_direction_reversed > 1U ||
        !isfinite(settings->motor_idle_percent) ||
        settings->motor_idle_percent < 1.0f ||
        settings->motor_idle_percent > 10.0f ||
        (settings->motor_protocol != MOTOR_PROTOCOL_DSHOT300 &&
         settings->motor_protocol != MOTOR_PROTOCOL_ONESHOT125 &&
         settings->motor_protocol != MOTOR_PROTOCOL_MULTISHOT)) {
        return false;
    }
    const bool alignment_changed =
        settings->board_roll_deg != current_settings.board_roll_deg ||
        settings->board_pitch_deg != current_settings.board_pitch_deg ||
        settings->board_yaw_deg != current_settings.board_yaw_deg;
    current_settings = *settings;
    settings_saved = false;
    apply();
    if (alignment_changed) {
        flight_control_start_calibration();
    }
    return true;
}

bool flight_settings_save(void)
{
    settings_record_t record = {
        .magic = SETTINGS_MAGIC,
        .version = SETTINGS_VERSION,
        .settings = current_settings,
    };
    record.checksum = checksum(&record);

    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef erase = {
        .TypeErase = FLASH_TYPEERASE_SECTORS,
        .Sector = SETTINGS_FLASH_SECTOR,
        .NbSectors = 1U,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3,
    };
    uint32_t sector_error = 0U;
    bool ok = HAL_FLASHEx_Erase(&erase, &sector_error) == HAL_OK;

    const uint32_t *words = (const uint32_t *)&record;
    for (size_t i = 0; ok && i < sizeof(record) / sizeof(uint32_t); ++i) {
        ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                               SETTINGS_ADDRESS + (uint32_t)i * 4U,
                               words[i]) == HAL_OK;
    }
    HAL_FLASH_Lock();
    settings_saved = ok;
    return ok;
}

bool flight_settings_are_saved(void)
{
    return settings_saved;
}
