#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    MOTOR_PROTOCOL_DSHOT300 = 0,
    MOTOR_PROTOCOL_ONESHOT125 = 1,
    MOTOR_PROTOCOL_MULTISHOT = 2
} motor_protocol_t;

void dshot_init(void);
void dshot_write(const uint16_t values[4]);
uint16_t dshot_from_percent(float percent);
bool motor_protocol_set(motor_protocol_t protocol);
motor_protocol_t motor_protocol_get(void);
const char *motor_protocol_name(motor_protocol_t protocol);
