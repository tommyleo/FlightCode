#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "flight_control.h"
#include "dshot.h"

typedef struct {
    pid_gains_t roll;
    pid_gains_t pitch;
    pid_gains_t yaw;
    motor_protocol_t motor_protocol;
    float board_roll_deg;
    float board_pitch_deg;
    float board_yaw_deg;
    uint32_t motor_direction_reversed;
    float motor_idle_percent;
    float roll_rate_dps;
    float pitch_rate_dps;
    float yaw_rate_dps;
    float rate_expo;
    float roll_feedforward;
    float pitch_feedforward;
    float yaw_feedforward;
    float tpa_attenuation;
    float tpa_breakpoint_percent;
} flight_settings_t;

void flight_settings_init(void);
const flight_settings_t *flight_settings_get(void);
bool flight_settings_set(const flight_settings_t *settings);
void flight_settings_reset_defaults(void);
bool flight_settings_save(void);
bool flight_settings_are_saved(void);
