#pragma once

#include <stdbool.h>
#include <stdint.h>

#define FLIGHT_LOG_RATE_HZ 200U
#define FLIGHT_LOG_FLAG_MIXER_SATURATED 0x01U
#define FLIGHT_LOG_FLAG_STOP_DISARM 0x02U
#define FLIGHT_LOG_FLAG_STOP_RX_LOSS 0x04U
#define FLIGHT_LOG_FLAG_STOP_IMU 0x08U
#define FLIGHT_LOG_FLAG_STOP_RX_FAILSAFE 0x10U
#define FLIGHT_LOG_FLAG_STOP_RX_TIMEOUT 0x20U

typedef struct __attribute__((packed)) {
    int16_t gyro[3];       /* 0.1 deg/s */
    int16_t setpoint[3];   /* 0.1 deg/s */
    int8_t pid[3];         /* 0.5 percent */
    uint8_t motor[4];      /* 0..255 = 0..100 percent */
    uint8_t throttle;      /* 0..200 = 0..100 percent */
    uint8_t flags;         /* bit 0 mixer saturated */
    uint16_t loop_us;
    uint16_t battery_centivolts;
    uint16_t cell_centivolts;
    uint8_t battery_cells;
    int8_t p_term[3];      /* 0.5 percent */
    int8_t i_term[3];      /* 0.5 percent */
    int8_t d_term[3];      /* 0.5 percent */
    int8_t ff_term[3];     /* 0.5 percent */
} flight_log_record_t;

#define FLIGHT_LOG_METADATA_VERSION 1U
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
void flight_log_record(const float gyro[3], const float setpoint[3],
                       const float pid[3], const float p_term[3],
                       const float i_term[3], const float d_term[3],
                       const float ff_term[3],
                       const uint16_t motors[4],
                       float throttle_percent, bool mixer_saturated,
                       uint16_t loop_us);
