#pragma once

#include <stdbool.h>

#include "imu.h"

bool mpu6000_init(void);
bool mpu6000_read(imu_sample_t *sample);
uint32_t mpu6000_get_gyro_rate_hz(void);
