#pragma once

#include <stdbool.h>

#include "imu.h"

bool icm42688p_init(uint32_t sample_rate_hz);
bool icm42688p_read(imu_sample_t *sample);
