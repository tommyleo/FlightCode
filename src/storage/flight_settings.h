#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "flight_control.h"
#include "dshot.h"

#define OSD_ELEMENT_COUNT 5U
#define OSD_PILOT_NAME_LENGTH 12U

typedef enum {
    OSD_ELEMENT_TOTAL_VOLTAGE = 0,
    OSD_ELEMENT_CELL_VOLTAGE,
    OSD_ELEMENT_FLIGHT_TIMER,
    OSD_ELEMENT_FLIGHTCODE,
    OSD_ELEMENT_PILOT_NAME,
} osd_element_t;

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
    uint32_t receiver_channel_order;
    uint32_t arm_channel;
    uint32_t arm_min_us;
    uint32_t arm_max_us;
    uint32_t beep_channel;
    uint32_t beep_min_us;
    uint32_t beep_max_us;
    uint32_t osd_enabled;
    uint32_t osd_position;
    uint32_t blackbox_enabled;
    float gyro_lpf_hz;
    float dterm_lpf_hz;
    uint32_t receiver_protocol;
    uint32_t main_loop_hz;
    float vbat_multiplier;
    uint32_t osd_element_enabled_mask;
    uint32_t osd_element_positions[OSD_ELEMENT_COUNT];
    char osd_pilot_name[OSD_PILOT_NAME_LENGTH + 1U];
    uint32_t vtx_protocol;
    uint32_t vtx_uart;
    uint32_t vtx_region;
    uint32_t vtx_band;
    uint32_t vtx_channel;
    uint32_t vtx_power_mw;
    uint32_t vtx_osd_enabled_mask;
    uint32_t vtx_osd_positions[2];
    float dynamic_d_boost_percent;
} flight_settings_t;

#define RECEIVER_ORDER_TAER1234 0U
#define RECEIVER_ORDER_AETR1234 1U
#define RECEIVER_PROTOCOL_SBUS 0U
#define RECEIVER_PROTOCOL_CRSF 1U
#define VTX_PROTOCOL_OFF 0U
#define VTX_PROTOCOL_SMARTAUDIO 1U
#define VTX_PROTOCOL_TRAMP 2U
#define VTX_PROTOCOL_HDZERO_MSP 3U
#define VTX_REGION_EU 0U
#define VTX_REGION_US 1U

void flight_settings_init(void);
const flight_settings_t *flight_settings_get(void);
bool flight_settings_set(const flight_settings_t *settings);
void flight_settings_reset_defaults(void);
void flight_settings_reset_tuning_defaults(flight_settings_t *settings);
bool flight_settings_save(void);
bool flight_settings_are_saved(void);
