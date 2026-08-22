#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "imu.h"

#define MAIN_MOTOR_COUNT 4U

typedef struct {
    uint32_t phase;
    uint32_t rate_hz;
} loop_task_t;

typedef struct {
    uint32_t loop_hz;
    uint32_t loop_cycles;
    uint32_t next_loop;
    uint32_t previous_loop_us;
    uint32_t previous_gyro_update_us;
    uint32_t loop_window_start_us;
    uint32_t loop_window_count;
    uint32_t max_loop_period_us;
    loop_task_t service_task;
    loop_task_t telemetry_task;
    loop_task_t osd_task;
    loop_task_t imu_retry_task;
    loop_task_t esc_task;
    loop_task_t imu_task;
    imu_sample_t imu;
    float measured_loop_hz;
    uint16_t motors[MAIN_MOTOR_COUNT];
    uint8_t consecutive_imu_failures;
    uint8_t osd_retry_ticks;
    bool imu_ready;
} main_loop_state_t;
