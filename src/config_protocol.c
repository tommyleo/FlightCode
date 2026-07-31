#include "config_protocol.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "board.h"
#include "flight_control.h"
#include "flight_log.h"
#include "flight_settings.h"
#include "usb_cdc.h"

#define LINE_LENGTH 192U
#define CLIENT_TIMEOUT_US 3000000U
#define MOTOR_TEST_TIMEOUT_US 1000000U
#define ARM_CHANNEL 5U

static char input_line[LINE_LENGTH];
static size_t input_length;
static bool client_active;
static uint32_t last_activity_us;
static bool motor_test_enabled;
static bool pid_simulation_enabled;
static float motor_test_percent[4];
static uint32_t last_motor_test_us;
static bool dfu_pending;
static uint32_t dfu_deadline_us;

static void reply(const char *format, ...)
{
    char output[384];
    va_list args;
    va_start(args, format);
    const int length = vsnprintf(output, sizeof(output), format, args);
    va_end(args);
    if (length > 0) {
        usb_cdc_write((const uint8_t *)output,
                      (size_t)length < sizeof(output) ? (size_t)length
                                                     : sizeof(output) - 1U);
    }
}

static void send_pids(void)
{
    const flight_settings_t *s = flight_settings_get();
    reply("@CFG PIDS %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %u\n",
          s->roll.kp, s->roll.ki, s->roll.kd,
          s->pitch.kp, s->pitch.ki, s->pitch.kd,
          s->yaw.kp, s->yaw.ki, s->yaw.kd,
          flight_settings_are_saved() ? 1U : 0U);
}

static void send_motor_protocol(void)
{
    reply("@CFG MOTOR_PROTOCOL %s %u\n",
          motor_protocol_name(flight_settings_get()->motor_protocol),
          flight_settings_are_saved() ? 1U : 0U);
}

static void send_board_alignment(void)
{
    const flight_settings_t *s = flight_settings_get();
    reply("@CFG BOARD_ALIGNMENT %.2f %.2f %.2f %u\n",
          s->board_roll_deg, s->board_pitch_deg, s->board_yaw_deg,
          flight_settings_are_saved() ? 1U : 0U);
}

static void send_motor_direction(void)
{
    reply("@CFG MOTOR_DIRECTION %s %u\n",
          flight_settings_get()->motor_direction_reversed != 0U
              ? "REVERSED" : "NORMAL",
          flight_settings_are_saved() ? 1U : 0U);
}

static void send_motor_idle(void)
{
    reply("@CFG MOTOR_IDLE %.2f %u\n",
          flight_settings_get()->motor_idle_percent,
          flight_settings_are_saved() ? 1U : 0U);
}

static void send_rates(void)
{
    const flight_settings_t *s = flight_settings_get();
    reply("@CFG RATES %.1f %.1f %.1f %.3f %u\n",
          s->roll_rate_dps, s->pitch_rate_dps, s->yaw_rate_dps,
          s->rate_expo, flight_settings_are_saved() ? 1U : 0U);
}

static void send_feedforward(void)
{
    const flight_settings_t *s = flight_settings_get();
    reply("@CFG FEEDFORWARD %.6f %.6f %.6f %u\n",
          s->roll_feedforward, s->pitch_feedforward,
          s->yaw_feedforward, flight_settings_are_saved() ? 1U : 0U);
}

static void send_tpa(void)
{
    const flight_settings_t *s = flight_settings_get();
    reply("@CFG TPA %.3f %.1f %u\n",
          s->tpa_attenuation, s->tpa_breakpoint_percent,
          flight_settings_are_saved() ? 1U : 0U);
}

static void process(const char *command)
{
    if (strcmp(command, "HELLO") == 0) {
        client_active = true;
        last_activity_us = board_micros();
        reply("@CFG HELLO FlightCode 3 %s\n", BOARD_NAME);
        reply("@CFG CAPABILITIES PIDS MOTOR_TEST TELEMETRY MOTOR_PROTOCOL "
              "BOARD_ALIGNMENT MOTOR_DIRECTION MOTOR_IDLE RATES "
              "FEEDFORWARD TPA GYRO_CALIBRATION FLIGHT_LOG PID_SIM DFU "
              "TELEMETRY_EXT\n");
        send_pids();
        send_motor_protocol();
        send_board_alignment();
        send_motor_direction();
        send_motor_idle();
        send_rates();
        send_feedforward();
        send_tpa();
        return;
    }
    if (strcmp(command, "PING") == 0) {
        client_active = true;
        last_activity_us = board_micros();
        return;
    }
    if (strcmp(command, "BYE") == 0) {
        client_active = false;
        motor_test_enabled = false;
        return;
    }
    if (strcmp(command, "GET_PIDS") == 0) {
        last_activity_us = board_micros();
        send_pids();
        return;
    }
    if (strcmp(command, "GET_MOTOR_PROTOCOL") == 0) {
        last_activity_us = board_micros();
        send_motor_protocol();
        return;
    }
    if (strcmp(command, "GET_BOARD_ALIGNMENT") == 0) {
        last_activity_us = board_micros();
        send_board_alignment();
        return;
    }
    if (strcmp(command, "GET_MOTOR_DIRECTION") == 0) {
        last_activity_us = board_micros();
        send_motor_direction();
        return;
    }
    if (strcmp(command, "GET_MOTOR_IDLE") == 0) {
        last_activity_us = board_micros();
        send_motor_idle();
        return;
    }
    if (strcmp(command, "GET_RATES") == 0) {
        last_activity_us = board_micros();
        send_rates();
        return;
    }
    if (strcmp(command, "GET_FEEDFORWARD") == 0) {
        last_activity_us = board_micros();
        send_feedforward();
        return;
    }
    if (strcmp(command, "GET_TPA") == 0) {
        last_activity_us = board_micros();
        send_tpa();
        return;
    }
    if (strcmp(command, "GET_FLIGHT_LOG_INFO") == 0) {
        const sbus_data_t *const rx = sbus_get();
        reply("@CFG FLIGHT_LOG_INFO %lu %u %u %u %lu %lu %lu %lu\n",
              (unsigned long)flight_log_count(), FLIGHT_LOG_RATE_HZ,
              flight_log_is_recording() ? 1U : 0U,
              (unsigned int)rx->loss_reason,
              (unsigned long)(rx->frame_age_us == UINT32_MAX
                                  ? UINT32_MAX
                                  : rx->frame_age_us / 1000U),
              (unsigned long)rx->valid_frame_count,
              (unsigned long)rx->uart_error_count,
              (unsigned long)rx->recovery_count);
        return;
    }
    if (strcmp(command, "CALIBRATE_GYRO") == 0) {
        if (flight_control_is_armed()) {
            reply("@CFG ERROR ARMED\n");
        } else {
            flight_control_start_calibration();
            reply("@CFG OK CALIBRATE_GYRO\n");
        }
        return;
    }
    unsigned int log_offset, log_count;
    if (sscanf(command, "GET_FLIGHT_LOG_CHUNK %u %u",
               &log_offset, &log_count) == 2) {
        if (flight_log_is_recording()) {
            reply("@CFG ERROR FLIGHT_LOG_RECORDING\n");
            return;
        }
        if (log_count > 4U) log_count = 4U;
        uint32_t sent = 0U;
        for (; sent < log_count; ++sent) {
            flight_log_record_t item;
            if (!flight_log_get((uint32_t)log_offset + sent, &item)) break;
            reply("@CFG FLIGHT_LOG %lu %d %d %d %d %d %d %d %d %d %u %u %u %u %u %u %u\n",
                  (unsigned long)((uint32_t)log_offset + sent),
                  item.gyro[0], item.gyro[1], item.gyro[2],
                  item.setpoint[0], item.setpoint[1], item.setpoint[2],
                  item.pid[0], item.pid[1], item.pid[2],
                  item.motor[0], item.motor[1],
                  item.motor[2], item.motor[3],
                  item.throttle, item.flags, item.loop_us);
        }
        reply("@CFG FLIGHT_LOG_CHUNK_END %lu\n",
              (unsigned long)((uint32_t)log_offset + sent));
        return;
    }
    if (strcmp(command, "PID_SIM_RESET") == 0 &&
        pid_simulation_enabled) {
        flight_control_reset_pid_state();
        reply("@CFG OK PID_SIM_RESET\n");
        return;
    }
    if (flight_control_is_armed()) {
        reply("@CFG ERROR ARMED\n");
        return;
    }
    if (strcmp(command, "ENTER_DFU") == 0) {
        motor_test_enabled = false;
        memset(motor_test_percent, 0, sizeof(motor_test_percent));
        dfu_pending = true;
        dfu_deadline_us = board_micros() + 200000U;
        reply("@CFG OK ENTER_DFU\n");
        return;
    }

    unsigned int enable;
    if (sscanf(command, "PID_SIM_ENABLE %u", &enable) == 1) {
        if (enable == 0U) {
            pid_simulation_enabled = false;
            flight_log_set_inhibited(false);
            reply("@CFG OK PID_SIM_DISABLED\n");
        } else {
            motor_test_enabled = false;
            memset(motor_test_percent, 0, sizeof(motor_test_percent));
            pid_simulation_enabled = true;
            flight_log_set_inhibited(true);
            reply("@CFG OK PID_SIM_ENABLED\n");
        }
        return;
    }
    if (sscanf(command, "MOTOR_TEST_ENABLE %u", &enable) == 1) {
        if (enable == 0U) {
            motor_test_enabled = false;
            memset(motor_test_percent, 0, sizeof(motor_test_percent));
            reply("@CFG OK MOTOR_TEST_DISABLED\n");
        } else if (sbus_get()->channel_us[ARM_CHANNEL] > 2000U) {
            reply("@CFG ERROR ARM_SWITCH\n");
        } else {
            motor_test_enabled = true;
            last_motor_test_us = board_micros();
            reply("@CFG OK MOTOR_TEST_ENABLED\n");
        }
        return;
    }

    float test[4];
    if (sscanf(command, "MOTOR_TEST %f %f %f %f",
               &test[0], &test[1], &test[2], &test[3]) == 4) {
        if (!motor_test_enabled) {
            reply("@CFG ERROR MOTOR_TEST_DISABLED\n");
            return;
        }
        for (uint8_t i = 0U; i < 4U; ++i) {
            if (test[i] < 0.0f || test[i] > 100.0f) {
                reply("@CFG ERROR MOTOR_TEST_RANGE\n");
                return;
            }
        }
        memcpy(motor_test_percent, test, sizeof(test));
        last_motor_test_us = board_micros();
        return;
    }

    flight_settings_t settings = *flight_settings_get();
    if (sscanf(command, "SET_TPA %f %f",
               &settings.tpa_attenuation,
               &settings.tpa_breakpoint_percent) == 2) {
        if (flight_settings_set(&settings)) {
            reply("@CFG OK SET_TPA\n");
            send_tpa();
        } else {
            reply("@CFG ERROR INVALID_TPA\n");
        }
        return;
    }
    if (sscanf(command, "SET_FEEDFORWARD %f %f %f",
               &settings.roll_feedforward, &settings.pitch_feedforward,
               &settings.yaw_feedforward) == 3) {
        if (flight_settings_set(&settings)) {
            reply("@CFG OK SET_FEEDFORWARD\n");
            send_feedforward();
        } else {
            reply("@CFG ERROR INVALID_FEEDFORWARD\n");
        }
        return;
    }
    if (sscanf(command, "SET_RATES %f %f %f %f",
               &settings.roll_rate_dps, &settings.pitch_rate_dps,
               &settings.yaw_rate_dps, &settings.rate_expo) == 4) {
        if (flight_settings_set(&settings)) {
            reply("@CFG OK SET_RATES\n");
            send_rates();
        } else {
            reply("@CFG ERROR INVALID_RATES\n");
        }
        return;
    }
    float idle_percent;
    if (sscanf(command, "SET_MOTOR_IDLE %f", &idle_percent) == 1) {
        settings.motor_idle_percent = idle_percent;
        if (flight_settings_set(&settings)) {
            reply("@CFG OK SET_MOTOR_IDLE\n");
            send_motor_idle();
        } else {
            reply("@CFG ERROR INVALID_MOTOR_IDLE\n");
        }
        return;
    }
    char direction_name[16];
    if (sscanf(command, "SET_MOTOR_DIRECTION %15s", direction_name) == 1) {
        if (strcmp(direction_name, "REVERSED") == 0) {
            settings.motor_direction_reversed = 1U;
        } else if (strcmp(direction_name, "NORMAL") == 0) {
            settings.motor_direction_reversed = 0U;
        } else {
            reply("@CFG ERROR INVALID_MOTOR_DIRECTION\n");
            return;
        }
        if (flight_settings_set(&settings)) {
            reply("@CFG OK SET_MOTOR_DIRECTION\n");
            send_motor_direction();
        } else {
            reply("@CFG ERROR SET_MOTOR_DIRECTION\n");
        }
        return;
    }
    if (sscanf(command, "SET_BOARD_ALIGNMENT %f %f %f",
               &settings.board_roll_deg, &settings.board_pitch_deg,
               &settings.board_yaw_deg) == 3) {
        if (flight_settings_set(&settings)) {
            reply("@CFG OK SET_BOARD_ALIGNMENT\n");
            send_board_alignment();
        } else {
            reply("@CFG ERROR INVALID_BOARD_ALIGNMENT\n");
        }
        return;
    }
    if (sscanf(command, "SET_PIDS %f %f %f %f %f %f %f %f %f",
               &settings.roll.kp, &settings.roll.ki, &settings.roll.kd,
               &settings.pitch.kp, &settings.pitch.ki, &settings.pitch.kd,
               &settings.yaw.kp, &settings.yaw.ki, &settings.yaw.kd) == 9) {
        if (flight_settings_set(&settings)) {
            reply("@CFG OK SET_PIDS\n");
            send_pids();
        } else {
            reply("@CFG ERROR INVALID_PIDS\n");
        }
        return;
    }
    if (strcmp(command, "SAVE_PIDS") == 0 ||
        strcmp(command, "SAVE_SETTINGS") == 0) {
        reply(flight_settings_save() ? "@CFG OK SAVE_SETTINGS\n"
                                     : "@CFG ERROR SAVE_SETTINGS\n");
        send_pids();
        send_motor_protocol();
        send_board_alignment();
        send_motor_direction();
        send_motor_idle();
        send_rates();
        send_feedforward();
        send_tpa();
        return;
    }
    if (strcmp(command, "RESET_PIDS") == 0) {
        settings = *flight_settings_get();
        settings.roll = (pid_gains_t){0.090f, 0.200f, 0.0012f};
        settings.pitch = (pid_gains_t){0.090f, 0.200f, 0.0012f};
        settings.yaw = (pid_gains_t){0.120f, 0.200f, 0.0f};
        settings.roll_rate_dps = 500.0f;
        settings.pitch_rate_dps = 500.0f;
        settings.yaw_rate_dps = 400.0f;
        settings.rate_expo = 0.35f;
        settings.roll_feedforward = 0.025f;
        settings.pitch_feedforward = 0.025f;
        settings.yaw_feedforward = 0.015f;
        settings.tpa_attenuation = 0.0f;
        settings.tpa_breakpoint_percent = 65.0f;
        reply(flight_settings_set(&settings) ? "@CFG OK RESET_PIDS\n"
                                             : "@CFG ERROR RESET_PIDS\n");
        send_pids();
        send_rates();
        send_feedforward();
        send_tpa();
        return;
    }
    char protocol_name[24];
    if (sscanf(command, "SET_MOTOR_PROTOCOL %23s", protocol_name) == 1) {
        settings = *flight_settings_get();
        if (strcmp(protocol_name, "ONESHOT125") == 0) {
            settings.motor_protocol = MOTOR_PROTOCOL_ONESHOT125;
        } else if (strcmp(protocol_name, "MULTISHOT") == 0) {
            settings.motor_protocol = MOTOR_PROTOCOL_MULTISHOT;
        } else if (strcmp(protocol_name, "DSHOT300") == 0) {
            settings.motor_protocol = MOTOR_PROTOCOL_DSHOT300;
        } else {
            reply("@CFG ERROR INVALID_MOTOR_PROTOCOL\n");
            return;
        }
        if (flight_settings_set(&settings)) {
            reply("@CFG OK SET_MOTOR_PROTOCOL\n");
            send_motor_protocol();
        } else {
            reply("@CFG ERROR SET_MOTOR_PROTOCOL\n");
        }
        return;
    }
    reply("@CFG ERROR UNKNOWN_COMMAND\n");
}

void config_protocol_init(void)
{
    motor_test_enabled = false;
    pid_simulation_enabled = false;
    dfu_pending = false;
    memset(motor_test_percent, 0, sizeof(motor_test_percent));
    usb_cdc_init();
}

bool config_protocol_motor_output_suppressed(void)
{
    return pid_simulation_enabled;
}

void config_protocol_update(void)
{
    uint8_t character;
    while (usb_cdc_read(&character, 1U) == 1U) {
        if (character == '\r') {
            continue;
        }
        if (character == '\n') {
            input_line[input_length] = '\0';
            if (input_length != 0U) {
                process(input_line);
            }
            input_length = 0U;
        } else if (input_length < LINE_LENGTH - 1U) {
            input_line[input_length++] = (char)character;
        } else {
            input_length = 0U;
            reply("@CFG ERROR LINE_TOO_LONG\n");
        }
    }
    usb_cdc_poll();
    if (dfu_pending &&
        (int32_t)(board_micros() - dfu_deadline_us) >= 0) {
        usb_cdc_deinit();
        HAL_Delay(100U);
        board_enter_dfu();
    }
    if (client_active &&
        (uint32_t)(board_micros() - last_activity_us) > CLIENT_TIMEOUT_US) {
        client_active = false;
    }
}

bool config_protocol_apply_motor_test(const sbus_data_t *receiver,
                                      uint16_t motors[4])
{
    const uint32_t now = board_micros();
    if (!motor_test_enabled ||
        receiver->channel_us[ARM_CHANNEL] > 2000U ||
        (uint32_t)(now - last_motor_test_us) > MOTOR_TEST_TIMEOUT_US) {
        motor_test_enabled = false;
        memset(motor_test_percent, 0, sizeof(motor_test_percent));
        return false;
    }
    for (uint8_t i = 0U; i < 4U; ++i) {
        motors[i] = dshot_from_percent(motor_test_percent[i]);
    }
    return true;
}

void config_protocol_send_telemetry(const sbus_data_t *rx,
                                    const imu_sample_t *imu,
                                    const uint16_t motors[4],
                                    float loop_hz,
                                    uint32_t max_loop_period_us)
{
    if (!client_active) {
        return;
    }
    imu_sample_t corrected;
    flight_control_get_corrected_imu(imu, &corrected);

    char output[384];
    int used = snprintf(output, sizeof(output),
                        "@CFG TELEMETRY %lu %u %u %.1f %.3f %.3f %.3f %.3f %.3f %.3f",
                        (unsigned long)board_micros(), rx->valid ? 1U : 0U,
                        flight_control_is_armed() ? 1U : 0U,
                        loop_hz,
                        corrected.gyro_x_dps,
                        corrected.gyro_y_dps,
                        corrected.gyro_z_dps,
                        corrected.accel_x_g,
                        corrected.accel_y_g,
                        corrected.accel_z_g);
    for (uint8_t i = 0U; i < SBUS_CHANNEL_COUNT && used > 0; ++i) {
        used += snprintf(output + used, sizeof(output) - (size_t)used,
                         " %u", rx->channel_us[i]);
    }
    for (uint8_t i = 0U; i < 4U && used > 0; ++i) {
        const float percent = motors[i] == 0U ? 0.0f :
            ((float)(motors[i] - 48U) * 100.0f / 1999.0f);
        used += snprintf(output + used, sizeof(output) - (size_t)used,
                         " %.2f", percent);
    }
    if (used > 0) {
        used += snprintf(output + used, sizeof(output) - (size_t)used,
                         " %u", flight_control_is_calibrated() ? 1U : 0U);
    }
    if (used > 0) {
        used += snprintf(output + used, sizeof(output) - (size_t)used,
                         " %lu", (unsigned long)max_loop_period_us);
    }
    if (used > 0) {
        /* Expose raw Y to distinguish sensor data from gyro bias correction. */
        used += snprintf(output + used, sizeof(output) - (size_t)used,
                         " %.3f", imu->gyro_y_dps);
    }
    if (used > 0 && (size_t)used < sizeof(output) - 1U) {
        output[used++] = '\n';
        usb_cdc_write((const uint8_t *)output, (size_t)used);
    }
}
