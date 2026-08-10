#pragma once

#include <stdbool.h>

#include "imu.h"

bool mpu6000_init(void);
bool mpu6000_read(imu_sample_t *sample);
