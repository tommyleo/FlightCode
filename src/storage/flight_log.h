#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define FLIGHT_LOG_RATE_HZ 200U
#define BLACKBOX_LOG_RATE_HZ 1000U
#define FLIGHT_LOG_FORMAT_VERSION_MAJOR 1U
#define FLIGHT_LOG_FORMAT_VERSION_MINOR 0U
#define FLIGHT_LOG_FORMAT_VERSION_PACKED \
    ((FLIGHT_LOG_FORMAT_VERSION_MAJOR << 4U) | \
     FLIGHT_LOG_FORMAT_VERSION_MINOR)
#define FLIGHT_LOG_FLAG_MIXER_SATURATED 0x01U
#define FLIGHT_LOG_FLAG_STOP_DISARM 0x02U
#define FLIGHT_LOG_FLAG_STOP_RX_LOSS 0x04U
#define FLIGHT_LOG_FLAG_STOP_IMU 0x08U
#define FLIGHT_LOG_FLAG_STOP_RX_FAILSAFE 0x10U
#define FLIGHT_LOG_FLAG_STOP_RX_TIMEOUT 0x20U


typedef struct __attribute__((packed)) {
    int16_t gyro[3];       /* 0.1 deg/s */
    int16_t setpoint[3];   /* 0.1 deg/s */
    uint8_t motor[4];      /* 0..255 = 0..100 percent */
    uint8_t throttle;      /* 0..200 = 0..100 percent */
    uint8_t flags;         /* bit 0 mixer saturated */
    uint16_t main_loop_us;
    uint16_t gyro_loop_us;
    uint16_t battery_centivolts;
    uint16_t cell_centivolts;
    uint8_t battery_cells;
    int8_t p_term[3];      /* 0.5 percent */
    int8_t i_term[3];      /* 0.5 percent */
    int8_t d_term[3];      /* 0.5 percent */
    int8_t ff_term[3];     /* 0.5 percent */
    uint8_t reserved; /* retain a 40-byte stride for aligned record storage */
} flight_log_record_t;

_Static_assert(sizeof(flight_log_record_t) == 40U,
               "flight log record format must remain 40 bytes");

/* Persistent SD/dataflash sample format 1.0. Eight 60-byte records plus the
 * 32-byte block header fill one 512-byte SD sector exactly. Loop periods use
 * two packed 12-bit values (0..4095 us). Static tuning values belong to
 * flight_log_metadata_t and are written once per flight. */
typedef struct __attribute__((packed)) {
    uint32_t timestamp_us;
    int16_t gyro_raw[3];          /* 0.1 deg/s, bias removed, before LPF */
    int16_t gyro_filtered[3];     /* 0.1 deg/s */
    int16_t setpoint[3];          /* 0.1 deg/s */
    int16_t d_unfiltered[3];      /* 0.01 percent, gain and TPA applied */
    int16_t d_filtered[3];        /* 0.01 percent */
    uint8_t motor[4];             /* 0..255 = 0..100 percent */
    uint8_t throttle;             /* 0..200 = 0..100 percent */
    uint8_t flags;
    int8_t pid[3];                /* final axis PID, 0.5 percent */
    int8_t p_term[3];             /* 0.5 percent */
    int8_t i_term[3];             /* 0.5 percent */
    int8_t ff_term[3];            /* 0.5 percent */
    uint16_t battery_centivolts;
    uint16_t dropped_records;     /* cumulative persistent-backend drops */
    uint8_t loop_timing[3];       /* main then gyro, packed 12-bit us */
    uint8_t format_version;       /* high nibble major, low nibble minor */
} blackbox_record_t;

_Static_assert(sizeof(blackbox_record_t) == 60U,
               "blackbox record must remain 60 bytes");

static inline void blackbox_record_set_loop_timing(
    blackbox_record_t *record, uint16_t main_loop_us, uint16_t gyro_loop_us)
{
    const uint32_t main_value = main_loop_us > 4095U ? 4095U : main_loop_us;
    const uint32_t gyro_value = gyro_loop_us > 4095U ? 4095U : gyro_loop_us;
    const uint32_t packed = main_value | (gyro_value << 12U);
    record->loop_timing[0] = (uint8_t)packed;
    record->loop_timing[1] = (uint8_t)(packed >> 8U);
    record->loop_timing[2] = (uint8_t)(packed >> 16U);
}

static inline uint16_t blackbox_record_main_loop_us(
    const blackbox_record_t *record)
{
    return (uint16_t)(record->loop_timing[0] |
        ((uint16_t)(record->loop_timing[1] & 0x0FU) << 8U));
}

static inline uint16_t blackbox_record_gyro_loop_us(
    const blackbox_record_t *record)
{
    return (uint16_t)((record->loop_timing[1] >> 4U) |
        ((uint16_t)record->loop_timing[2] << 4U));
}

#define FLIGHT_LOG_METADATA_VERSION 4U
typedef struct __attribute__((packed)) {
    uint32_t version;
    uint32_t main_loop_hz;
    uint32_t gyro_rate_hz;
    uint32_t log_rate_hz;
    float pids[9];
    float rates[4];
    float feedforward[3];
    float tpa[2];
    float filters[2];
    float alignment[3];
    float motor_idle_percent;
    uint32_t motor_protocol;
    uint32_t motor_direction_reversed;
    uint32_t receiver_protocol;
    uint16_t initial_battery_centivolts;
    uint8_t initial_battery_cells;
    uint8_t reserved;
} flight_log_metadata_t;

_Static_assert(sizeof(flight_log_metadata_t) == 128U,
               "metadata v4 must remain 128 bytes");
static inline bool flight_log_metadata_decode(flight_log_metadata_t *out,
                                               const void *stored)
{
    uint32_t version;
    memcpy(&version, stored, sizeof(version));
    if (version != FLIGHT_LOG_METADATA_VERSION) return false;
    memcpy(out, stored, sizeof(*out));
    return true;
}


void flight_log_init(void);
void flight_log_set_inhibited(bool inhibited);
void flight_log_set_battery_voltage(float voltage);
void flight_log_start(void);
void flight_log_stop(uint8_t stop_flag);
void flight_log_stop_rx(bool failsafe);
bool flight_log_is_recording(void);
bool flight_log_is_available(void);
uint32_t flight_log_count(void);
bool flight_log_get(uint32_t index, flight_log_record_t *record);
bool flight_log_get_metadata(flight_log_metadata_t *metadata);
bool flight_log_persist_pending(void);
void flight_log_persist_if_ready(void);
void flight_log_record(const float gyro_raw[3], const float gyro_filtered[3],
                       const float setpoint[3],
                       const float p_term[3],
                       const float i_term[3], const float d_unfiltered[3],
                       const float d_filtered[3],
                       const float ff_term[3],
                       const uint16_t motors[4],
                       float throttle_percent, bool mixer_saturated,
                       uint16_t main_loop_us, uint16_t gyro_loop_us,
                       const float pid_output[3]);
