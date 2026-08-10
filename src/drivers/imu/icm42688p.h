#pragma once

#include <stdbool.h>

#include "imu.h"

bool icm42688p_init(void);
bool icm42688p_read(imu_sample_t *sample);
