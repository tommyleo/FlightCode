#pragma once

#include <stdbool.h>

typedef struct {
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
} imu_sample_t;

bool mpu6000_init(void);
bool mpu6000_read(imu_sample_t *sample);
void mpu6000_set_board_alignment(float roll_deg, float pitch_deg,
                                 float yaw_deg);
