#include "flight_settings.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "board.h"
#include "imu.h"
#include "max7456.h"
#include "blackbox_sd.h"
#include "sbus.h"

#define SETTINGS_MAGIC 0x46344643U
#define SETTINGS_VERSION 22U
#define SETTINGS_LEGACY_VERSION_21 21U
#define SETTINGS_LEGACY_VERSION_20 20U
#define SETTINGS_LEGACY_VERSION_19 19U
#define SETTINGS_LEGACY_VERSION_18 18U
#define SETTINGS_LEGACY_VERSION_17 17U
#define SETTINGS_LEGACY_VERSION_16 16U
#define SETTINGS_LEGACY_VERSION_15 15U
#define SETTINGS_LEGACY_VERSION_14 14U
#define SETTINGS_LEGACY_VERSION_13 13U
#define SETTINGS_LEGACY_VERSION_12 12U
#define SETTINGS_LEGACY_VERSION_11 11U
#define SETTINGS_LEGACY_VERSION 7U
#define SETTINGS_LEGACY_VERSION_8 8U
#define SETTINGS_LEGACY_VERSION_9 9U
#define SETTINGS_LEGACY_VERSION_10 10U

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
    legacy_settings_v9_t base;
    float tpa_attenuation;
    float tpa_breakpoint_percent;
} legacy_settings_v10_t;

typedef struct {
    uint32_t magic;
    uint32_t version;
    legacy_settings_v10_t settings;
    uint32_t checksum;
} legacy_record_v10_t;

typedef struct {
    legacy_settings_v10_t base;
    uint32_t receiver_channel_order;
    uint32_t arm_channel;
    uint32_t arm_min_us;
    uint32_t arm_max_us;
    uint32_t beep_channel;
    uint32_t beep_min_us;
    uint32_t beep_max_us;
} legacy_settings_v11_t;

typedef struct {
    uint32_t magic;
    uint32_t version;
    legacy_settings_v11_t settings;
    uint32_t checksum;
} legacy_record_v11_t;

typedef struct {
    uint32_t magic;
    uint32_t version;
    legacy_settings_v11_t base;
    uint32_t osd_enabled;
    uint32_t osd_position;
    uint32_t checksum;
} legacy_record_v12_t;

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
    float roll_rate_dps;
    float pitch_rate_dps;
    float yaw_rate_dps;
    float rate_expo;
    float roll_feedforward;
    float pitch_feedforward;
    float yaw_feedforward;
    float tpa_attenuation;
    float tpa_breakpoint_percent;
    uint32_t receiver_channel_order;
    uint32_t arm_channel;
    uint32_t arm_min_us;
    uint32_t arm_max_us;
    uint32_t beep_channel;
    uint32_t beep_min_us;
    uint32_t beep_max_us;
    uint32_t osd_enabled;
    uint32_t osd_position;
    uint32_t blackbox_enabled;
} legacy_settings_v13_t;

typedef struct {
    uint32_t magic;
    uint32_t version;
    legacy_settings_v13_t settings;
    uint32_t checksum;
} legacy_record_v13_t;

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint8_t settings[offsetof(flight_settings_t, receiver_protocol)];
    uint32_t checksum;
} legacy_record_v14_t;

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint8_t settings[offsetof(flight_settings_t, main_loop_hz)];
    uint32_t checksum;
} legacy_record_v15_t;

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint8_t settings[offsetof(flight_settings_t, vbat_multiplier)];
    uint32_t checksum;
} legacy_record_v16_t;

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint8_t settings[offsetof(flight_settings_t, osd_element_enabled_mask)];
    uint32_t checksum;
} legacy_record_v17_t;

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint8_t settings[offsetof(flight_settings_t, vtx_protocol)];
    uint32_t checksum;
} legacy_record_v18_t;

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint8_t settings[offsetof(flight_settings_t, vtx_band)];
    uint32_t checksum;
} legacy_record_v19_t;

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint8_t settings[offsetof(flight_settings_t,
                              dynamic_d_boost_percent)];
    uint32_t checksum;
} legacy_record_v20_t;

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint8_t settings[offsetof(flight_settings_t, throttle_rise_ms)];
    uint32_t checksum;
} legacy_record_v21_t;

_Static_assert(offsetof(flight_settings_t, gyro_lpf_hz) ==
                   sizeof(legacy_settings_v13_t),
               "Flight settings v13 migration layout changed");

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

static bool main_loop_valid(uint32_t hz)
{
    return hz == 8000U || hz == 16000U;
}

static bool vbat_multiplier_valid(float multiplier)
{
    return isfinite(multiplier) && multiplier >= 0.5f && multiplier <= 1.5f;
}

static bool osd_layout_valid(const flight_settings_t *settings)
{
    if (settings->osd_element_enabled_mask >= (1U << OSD_ELEMENT_COUNT) ||
        memchr(settings->osd_pilot_name, '\0',
               sizeof(settings->osd_pilot_name)) == NULL) return false;
    for (uint8_t i = 0U; i < OSD_ELEMENT_COUNT; ++i) {
        if (settings->osd_element_positions[i] >= 30U * 16U) return false;
    }
    for (size_t i = 0U; settings->osd_pilot_name[i] != '\0'; ++i) {
        const char c = settings->osd_pilot_name[i];
        if (c != ' ' && c != '-' &&
            !(c >= '0' && c <= '9') && !(c >= 'A' && c <= 'Z')) return false;
    }
    return true;
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

static bool filters_valid(const flight_settings_t *settings)
{
    return isfinite(settings->gyro_lpf_hz) &&
           isfinite(settings->dterm_lpf_hz) &&
           settings->gyro_lpf_hz >= 50.0f &&
           settings->gyro_lpf_hz <= 250.0f &&
           settings->dterm_lpf_hz >= 20.0f &&
           settings->dterm_lpf_hz <= 200.0f &&
           settings->dterm_lpf_hz <= settings->gyro_lpf_hz &&
           isfinite(settings->dynamic_d_boost_percent) &&
           settings->dynamic_d_boost_percent >= 0.0f &&
           settings->dynamic_d_boost_percent <= 50.0f;
}

static bool receiver_valid(const flight_settings_t *settings)
{
    return settings->receiver_channel_order <= RECEIVER_ORDER_AETR1234 &&
           settings->receiver_protocol <= RECEIVER_PROTOCOL_CRSF &&
#if !BOARD_HAS_CRSF
           settings->receiver_protocol == RECEIVER_PROTOCOL_SBUS &&
#endif
           settings->arm_channel >= 4U && settings->arm_channel < 16U &&
           settings->beep_channel >= 4U && settings->beep_channel < 16U &&
           settings->arm_min_us >= 900U && settings->arm_max_us <= 2100U &&
           settings->arm_min_us < settings->arm_max_us &&
           settings->beep_min_us >= 900U && settings->beep_max_us <= 2100U &&
           settings->beep_min_us < settings->beep_max_us;
}

static bool vtx_valid(const flight_settings_t *settings)
{
    return settings->vtx_protocol <= VTX_PROTOCOL_HDZERO_MSP &&
           settings->vtx_uart >= 1U && settings->vtx_uart <= 6U &&
           settings->vtx_region <= VTX_REGION_US &&
           settings->vtx_band < 6U && settings->vtx_channel < 8U &&
           settings->vtx_power_mw >= 1U && settings->vtx_power_mw <= 2000U &&
           settings->vtx_osd_enabled_mask < 4U &&
           settings->vtx_osd_positions[0] < 480U &&
           settings->vtx_osd_positions[1] < 480U;
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
    imu_set_board_alignment(current_settings.board_roll_deg,
                            current_settings.board_pitch_deg,
                            current_settings.board_yaw_deg);
    flight_control_set_motor_direction_reversed(
        current_settings.motor_direction_reversed != 0U);
    flight_control_set_motor_idle_percent(current_settings.motor_idle_percent);
    (void)max7456_set_config(current_settings.osd_enabled != 0U,
                             (uint8_t)current_settings.osd_position);
    (void)max7456_set_layout(current_settings.osd_element_enabled_mask,
                             current_settings.osd_element_positions,
                             current_settings.osd_pilot_name);
    blackbox_sd_set_enabled(current_settings.blackbox_enabled != 0U);
    (void)sbus_set_protocol(current_settings.receiver_protocol);
    board_battery_set_multiplier(current_settings.vbat_multiplier);
}

void flight_settings_reset_tuning_defaults(flight_settings_t *settings)
{
    settings->roll = (pid_gains_t){0.10100f, 0.19000f, 0.00120f};
    settings->pitch = (pid_gains_t){0.09950f, 0.20000f, 0.00100f};
    settings->yaw = (pid_gains_t){0.15000f, 0.25000f, 0.00000f};
    settings->roll_rate_dps = 420.0f;
    settings->pitch_rate_dps = 420.0f;
    settings->yaw_rate_dps = 350.0f;
    settings->rate_expo = 0.30f;
    settings->roll_feedforward = 0.025f;
    settings->pitch_feedforward = 0.025f;
    settings->yaw_feedforward = 0.015f;
    settings->tpa_attenuation = 0.20f;
    settings->tpa_breakpoint_percent = 70.0f;
    settings->gyro_lpf_hz = 90.0f;
    settings->dterm_lpf_hz = 50.0f;
    settings->dynamic_d_boost_percent = 12.5f;
    settings->throttle_rise_ms = 0.0f;
}

void flight_settings_reset_defaults(void)
{
    current_settings = (flight_settings_t){
        .motor_protocol = MOTOR_PROTOCOL_DSHOT600,
        .board_roll_deg = 0.0f,
        .board_pitch_deg = 0.0f,
        .board_yaw_deg = 0.0f,
        .motor_direction_reversed = 0U,
        .motor_idle_percent = 5.0f,
        .receiver_channel_order = RECEIVER_ORDER_TAER1234,
        .arm_channel = 5U,
        .arm_min_us = 1950U,
        .arm_max_us = 2100U,
        .beep_channel = 4U,
        .beep_min_us = 1950U,
        .beep_max_us = 2100U,
        .osd_enabled = BOARD_DEFAULT_OSD_ENABLED,
        .osd_position = 4U,
        .blackbox_enabled = 0U,
        .receiver_protocol = BOARD_DEFAULT_RECEIVER_PROTOCOL,
        .main_loop_hz = 16000U,
        .vbat_multiplier = 1.0f,
        .osd_element_enabled_mask = 1U,
        .osd_element_positions = {31U, 61U, 51U, 340U, 369U},
        .osd_pilot_name = "PILOT",
        .vtx_protocol = BOARD_DEFAULT_VTX_PROTOCOL,
        .vtx_uart = BOARD_DEFAULT_VTX_UART,
        .vtx_region = VTX_REGION_EU,
        .vtx_band = 4U,
        .vtx_channel = 0U,
        .vtx_power_mw = 25U,
        .vtx_osd_enabled_mask = 0U,
        .vtx_osd_positions = {55U, 85U},
    };
    flight_settings_reset_tuning_defaults(&current_settings);
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
    const legacy_record_v10_t *legacy_v10 =
        (const legacy_record_v10_t *)SETTINGS_ADDRESS;
    const legacy_record_v11_t *legacy_v11 =
        (const legacy_record_v11_t *)SETTINGS_ADDRESS;
    const legacy_record_v12_t *legacy_v12 =
        (const legacy_record_v12_t *)SETTINGS_ADDRESS;
    const legacy_record_v13_t *legacy_v13 =
        (const legacy_record_v13_t *)SETTINGS_ADDRESS;
    const legacy_record_v14_t *legacy_v14 =
        (const legacy_record_v14_t *)SETTINGS_ADDRESS;
    const legacy_record_v15_t *legacy_v15 =
        (const legacy_record_v15_t *)SETTINGS_ADDRESS;
    const legacy_record_v16_t *legacy_v16 =
        (const legacy_record_v16_t *)SETTINGS_ADDRESS;
    const legacy_record_v17_t *legacy_v17 =
        (const legacy_record_v17_t *)SETTINGS_ADDRESS;
    const legacy_record_v18_t *legacy_v18 =
        (const legacy_record_v18_t *)SETTINGS_ADDRESS;
    const legacy_record_v19_t *legacy_v19 =
        (const legacy_record_v19_t *)SETTINGS_ADDRESS;
    const legacy_record_v20_t *legacy_v20 =
        (const legacy_record_v20_t *)SETTINGS_ADDRESS;
    const legacy_record_v21_t *legacy_v21 =
        (const legacy_record_v21_t *)SETTINGS_ADDRESS;
    if (legacy_v21->magic == SETTINGS_MAGIC &&
        legacy_v21->version == SETTINGS_LEGACY_VERSION_21 &&
        legacy_v21->checksum == checksum_bytes(
            legacy_v21, offsetof(legacy_record_v21_t, checksum))) {
        flight_settings_reset_defaults();
        memcpy(&current_settings, legacy_v21->settings,
               sizeof(legacy_v21->settings));
        settings_saved = false;
        apply();
        return;
    }
    if (legacy_v20->magic == SETTINGS_MAGIC &&
        legacy_v20->version == SETTINGS_LEGACY_VERSION_20 &&
        legacy_v20->checksum == checksum_bytes(
            legacy_v20, offsetof(legacy_record_v20_t, checksum))) {
        flight_settings_reset_defaults();
        memcpy(&current_settings, legacy_v20->settings,
               sizeof(legacy_v20->settings));
        settings_saved = false;
        apply();
        return;
    }
    if (legacy_v19->magic == SETTINGS_MAGIC &&
        legacy_v19->version == SETTINGS_LEGACY_VERSION_19 &&
        legacy_v19->checksum == checksum_bytes(
            legacy_v19, offsetof(legacy_record_v19_t, checksum))) {
        flight_settings_reset_defaults();
        memcpy(&current_settings, legacy_v19->settings,
               sizeof(legacy_v19->settings));
        settings_saved = false;
        apply();
        return;
    }
    if (legacy_v18->magic == SETTINGS_MAGIC &&
        legacy_v18->version == SETTINGS_LEGACY_VERSION_18 &&
        legacy_v18->checksum == checksum_bytes(
            legacy_v18, offsetof(legacy_record_v18_t, checksum))) {
        flight_settings_reset_defaults();
        memcpy(&current_settings, legacy_v18->settings,
               sizeof(legacy_v18->settings));
        settings_saved = false;
        apply();
        return;
    }
    if (legacy_v17->magic == SETTINGS_MAGIC &&
        legacy_v17->version == SETTINGS_LEGACY_VERSION_17 &&
        legacy_v17->checksum == checksum_bytes(
            legacy_v17, offsetof(legacy_record_v17_t, checksum))) {
        flight_settings_reset_defaults();
        memcpy(&current_settings, legacy_v17->settings,
               sizeof(legacy_v17->settings));
        current_settings.osd_element_enabled_mask =
            current_settings.osd_enabled != 0U ? 1U : 0U;
        settings_saved = false;
        apply();
        return;
    }
    if (legacy_v16->magic == SETTINGS_MAGIC &&
        legacy_v16->version == SETTINGS_LEGACY_VERSION_16 &&
        legacy_v16->checksum == checksum_bytes(
            legacy_v16, offsetof(legacy_record_v16_t, checksum))) {
        flight_settings_reset_defaults();
        memcpy(&current_settings, legacy_v16->settings,
               sizeof(legacy_v16->settings));
        settings_saved = false;
        apply();
        return;
    }
    if (legacy_v15->magic == SETTINGS_MAGIC &&
        legacy_v15->version == SETTINGS_LEGACY_VERSION_15 &&
        legacy_v15->checksum == checksum_bytes(
            legacy_v15, offsetof(legacy_record_v15_t, checksum))) {
        flight_settings_reset_defaults();
        memcpy(&current_settings, legacy_v15->settings,
               sizeof(legacy_v15->settings));
        if (current_settings.motor_protocol != MOTOR_PROTOCOL_DSHOT300 &&
            current_settings.motor_protocol != MOTOR_PROTOCOL_DSHOT600 &&
            current_settings.motor_protocol != MOTOR_PROTOCOL_DSHOT1200) {
            current_settings.motor_protocol = MOTOR_PROTOCOL_DSHOT600;
        }
        settings_saved = false;
        apply();
        return;
    }
    if (legacy_v14->magic == SETTINGS_MAGIC &&
        legacy_v14->version == SETTINGS_LEGACY_VERSION_14 &&
        legacy_v14->checksum == checksum_bytes(
            legacy_v14, offsetof(legacy_record_v14_t, checksum))) {
        flight_settings_reset_defaults();
        memcpy(&current_settings, legacy_v14->settings,
               sizeof(legacy_v14->settings));
        settings_saved = false;
        apply();
        return;
    }
    if (legacy_v13->magic == SETTINGS_MAGIC &&
        legacy_v13->version == SETTINGS_LEGACY_VERSION_13 &&
        legacy_v13->checksum == checksum_bytes(
            legacy_v13, offsetof(legacy_record_v13_t, checksum))) {
        flight_settings_reset_defaults();
        memcpy(&current_settings, &legacy_v13->settings,
               sizeof(legacy_v13->settings));
        settings_saved = false;
        apply();
        return;
    }
    if (legacy_v12->magic == SETTINGS_MAGIC &&
        legacy_v12->version == SETTINGS_LEGACY_VERSION_12 &&
        legacy_v12->checksum == checksum_bytes(
            legacy_v12, offsetof(legacy_record_v12_t, checksum))) {
        flight_settings_reset_defaults();
        memcpy(&current_settings, &legacy_v12->base,
               sizeof(legacy_v12->base));
        current_settings.osd_enabled = legacy_v12->osd_enabled;
        current_settings.osd_position = legacy_v12->osd_position;
        settings_saved = false;
        apply();
        return;
    }
    if (legacy_v11->magic == SETTINGS_MAGIC &&
        legacy_v11->version == SETTINGS_LEGACY_VERSION_11 &&
        legacy_v11->checksum == checksum_bytes(
            legacy_v11, offsetof(legacy_record_v11_t, checksum))) {
        flight_settings_reset_defaults();
        memcpy(&current_settings, &legacy_v11->settings,
               sizeof(legacy_v11->settings));
        settings_saved = false;
        apply();
        return;
    }
    if (legacy_v10->magic == SETTINGS_MAGIC &&
        legacy_v10->version == SETTINGS_LEGACY_VERSION_10 &&
        legacy_v10->checksum == checksum_bytes(
            legacy_v10, offsetof(legacy_record_v10_t, checksum))) {
        flight_settings_reset_defaults();
        memcpy(&current_settings, &legacy_v10->settings,
               sizeof(legacy_v10->settings));
        settings_saved = false;
        apply();
        return;
    }
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
        !filters_valid(&stored->settings) ||
        !receiver_valid(&stored->settings) ||
        !vtx_valid(&stored->settings) ||
        !main_loop_valid(stored->settings.main_loop_hz) ||
        !vbat_multiplier_valid(stored->settings.vbat_multiplier) ||
        !osd_layout_valid(&stored->settings) ||
        stored->settings.osd_enabled > 1U ||
        stored->settings.osd_position > 8U ||
        stored->settings.blackbox_enabled > 1U ||
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
        !isfinite(stored->settings.throttle_rise_ms) ||
        stored->settings.throttle_rise_ms < 0.0f ||
        stored->settings.throttle_rise_ms > 1000.0f ||
        !isfinite(stored->settings.motor_idle_percent) ||
        stored->settings.motor_idle_percent < 1.0f ||
        stored->settings.motor_idle_percent > 10.0f ||
        (stored->settings.motor_protocol != MOTOR_PROTOCOL_DSHOT300 &&
         stored->settings.motor_protocol != MOTOR_PROTOCOL_DSHOT600 &&
         stored->settings.motor_protocol != MOTOR_PROTOCOL_DSHOT1200)) {
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
        !filters_valid(settings) ||
        !receiver_valid(settings) ||
        !vtx_valid(settings) ||
        !main_loop_valid(settings->main_loop_hz) ||
        !vbat_multiplier_valid(settings->vbat_multiplier) ||
        !osd_layout_valid(settings) ||
        settings->osd_enabled > 1U || settings->osd_position > 8U ||
        settings->blackbox_enabled > 1U ||
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
        !isfinite(settings->throttle_rise_ms) ||
        settings->throttle_rise_ms < 0.0f ||
        settings->throttle_rise_ms > 1000.0f ||
        !isfinite(settings->motor_idle_percent) ||
        settings->motor_idle_percent < 1.0f ||
        settings->motor_idle_percent > 10.0f ||
        (settings->motor_protocol != MOTOR_PROTOCOL_DSHOT300 &&
         settings->motor_protocol != MOTOR_PROTOCOL_DSHOT600 &&
         settings->motor_protocol != MOTOR_PROTOCOL_DSHOT1200)) {
        return false;
    }
    const bool alignment_changed =
        settings->board_roll_deg != current_settings.board_roll_deg ||
        settings->board_pitch_deg != current_settings.board_pitch_deg ||
        settings->board_yaw_deg != current_settings.board_yaw_deg;
    const bool filters_changed =
        settings->gyro_lpf_hz != current_settings.gyro_lpf_hz ||
        settings->dterm_lpf_hz != current_settings.dterm_lpf_hz;
    current_settings = *settings;
    settings_saved = false;
    apply();
    if (filters_changed) {
        flight_control_reset_pid_state();
    }
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
#if defined(PLATFORM_STM32H7)
        .Banks = SETTINGS_FLASH_BANK,
#else
        .VoltageRange = FLASH_VOLTAGE_RANGE_3,
#endif
    };
    uint32_t sector_error = 0U;
    bool ok = HAL_FLASHEx_Erase(&erase, &sector_error) == HAL_OK;

    const uint32_t *words = (const uint32_t *)&record;
#if defined(PLATFORM_STM32H7)
    uint32_t flash_word[8] __attribute__((aligned(32)));
    const size_t word_count = (sizeof(record) + 3U) / 4U;
    for (size_t base = 0U; ok && base < word_count; base += 8U) {
        for (size_t i = 0U; i < 8U; ++i)
            flash_word[i] = base + i < word_count ? words[base + i] : 0xFFFFFFFFU;
        ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                               SETTINGS_ADDRESS + (uint32_t)base * 4U,
                               (uint32_t)flash_word) == HAL_OK;
    }
#else
    for (size_t i = 0; ok && i < sizeof(record) / sizeof(uint32_t); ++i) {
        ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                               SETTINGS_ADDRESS + (uint32_t)i * 4U,
                               words[i]) == HAL_OK;
    }
#endif
    HAL_FLASH_Lock();
    settings_saved = ok;
    return ok;
}

bool flight_settings_are_saved(void)
{
    return settings_saved;
}
