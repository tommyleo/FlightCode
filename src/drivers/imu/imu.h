#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
} imu_sample_t;

bool imu_init(uint32_t sample_rate_hz);
bool imu_read(imu_sample_t *sample);
const char *imu_get_name(void);
void imu_set_board_alignment(float roll_deg, float pitch_deg, float yaw_deg);
