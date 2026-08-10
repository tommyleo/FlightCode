#pragma once

#include "imu.h"
#include "sbus.h"

#include <stdbool.h>

void config_protocol_init(void);
void config_protocol_update(void);
bool config_protocol_apply_motor_test(const sbus_data_t *receiver,
                                      uint16_t motors[4]);
bool config_protocol_motor_output_suppressed(void);
void config_protocol_send_telemetry(const sbus_data_t *receiver,
                                    const imu_sample_t *imu,
                                    const uint16_t motors[4],
                                    float loop_hz,
                                    uint32_t max_loop_period_us);
