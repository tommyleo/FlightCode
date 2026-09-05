#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define FLIGHT_LOG_RATE_HZ 200U
#define BLACKBOX_LOG_RATE_HZ 1000U
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
    uint8_t reserved;
} flight_log_record_t;

_Static_assert(sizeof(flight_log_record_t) == 40U,
               "flight log record format must remain 40 bytes");

/* Persistent SD/dataflash format. 48 bytes allows ten records plus the
 * 32-byte SD header to fit exactly in one 512-byte sector. */
#define BLACKBOX_RECORD_VERSION 2U
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
    uint16_t battery_centivolts;
    uint16_t dropped_records;     /* cumulative persistent-backend drops */
    uint8_t format_version;
} blackbox_record_t;

_Static_assert(sizeof(blackbox_record_t) == 48U,
               "blackbox record must remain 48 bytes");

#define FLIGHT_LOG_METADATA_VERSION 3U
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
    float throttle_rise_ms; /* full-scale rise time in ms; v3+ */
} flight_log_metadata_t;

/* Version 2 ended before throttle_rise_ms. Never interpret trailing padding
 * or the first legacy sample as a saved ramp. Keep the original version. */
_Static_assert(offsetof(flight_log_metadata_t, throttle_rise_ms) == 128U,
               "legacy metadata prefix must remain 128 bytes");
_Static_assert(sizeof(flight_log_metadata_t) == 132U,
               "metadata v3 must remain 132 bytes");
static inline bool flight_log_metadata_decode(flight_log_metadata_t *out,
                                               const void *stored)
{
    uint32_t version;
    memcpy(&version, stored, sizeof(version));
    if (version != 2U && version != FLIGHT_LOG_METADATA_VERSION) return false;
    memset(out, 0, sizeof(*out));
    memcpy(out, stored, version == 2U
        ? offsetof(flight_log_metadata_t, throttle_rise_ms) : sizeof(*out));
    if (version == 2U) out->throttle_rise_ms = -1.0f;
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
