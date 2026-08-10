#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "imu.h"
#include "sbus.h"

typedef struct {
    float kp;
    float ki;
    float kd;
} pid_gains_t;

void flight_control_init(void);
void flight_control_set_gains(const pid_gains_t *roll,
                              const pid_gains_t *pitch,
                              const pid_gains_t *yaw);
void flight_control_set_rates(float roll_max_dps, float pitch_max_dps,
                              float yaw_max_dps, float expo);
void flight_control_set_feedforward(float roll_ff, float pitch_ff,
                                    float yaw_ff);
void flight_control_set_tpa(float attenuation, float breakpoint_percent);
void flight_control_set_motor_direction_reversed(bool reversed);
void flight_control_set_motor_idle_percent(float percent);
void flight_control_update(const imu_sample_t *imu,
                           const sbus_data_t *receiver,
                           float dt,
                           uint16_t motor_dshot[4]);
bool flight_control_is_armed(void);
bool flight_control_is_calibrated(void);
uint16_t flight_control_get_calibration_samples(void);
void flight_control_start_calibration(void);
void flight_control_reset_pid_state(void);
void flight_control_emergency_stop(uint8_t log_stop_flag);
void flight_control_get_corrected_imu(const imu_sample_t *raw,
                                      imu_sample_t *corrected);
