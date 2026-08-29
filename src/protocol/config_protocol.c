#include "config_protocol.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "board.h"
#include "flight_control.h"
#include "flight_log.h"
#include "flight_settings.h"
#include "max7456.h"
#include "usb_cdc.h"
#include "blackbox_sd.h"
#include "vtx_tramp.h"

#define LINE_LENGTH 192U
#define CLIENT_TIMEOUT_US 3000000U
#define MOTOR_TEST_TIMEOUT_US 1000000U
#define BLACKBOX_CHUNK_MAX 16U

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
static bool reboot_pending;
static uint32_t reboot_deadline_us;

static bool arm_mode_active(const sbus_data_t *receiver)
{
    const flight_settings_t *s = flight_settings_get();
    return receiver->valid && s->arm_channel < 16U &&
           receiver->channel_us[s->arm_channel] >= s->arm_min_us &&
           receiver->channel_us[s->arm_channel] <= s->arm_max_us;
}

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
    reply("@CFG PIDS %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %u\n",
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

static void send_main_loop(void)
{
    reply("@CFG MAIN_LOOP %lu %u\n",
          (unsigned long)flight_settings_get()->main_loop_hz,
          flight_settings_are_saved() ? 1U : 0U);
}

#if BOARD_HAS_VBAT_CALIBRATION
static void send_vbat_multiplier(void)
{
    reply("@CFG VBAT_MULTIPLIER %.3f %u\n",
          flight_settings_get()->vbat_multiplier,
          flight_settings_are_saved() ? 1U : 0U);
}
#endif

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

static void send_filters(void)
{
    const flight_settings_t *s = flight_settings_get();
    reply("@CFG FILTERS %.1f %.1f %u\n",
          s->gyro_lpf_hz, s->dterm_lpf_hz,
          flight_settings_are_saved() ? 1U : 0U);
}

static void send_receiver_config(void)
{
    const flight_settings_t *s = flight_settings_get();
    const char *port =
#if defined(BOARD_FLYWOOF405NANO) || defined(BOARD_FLYWOOF405NANO_ANALOG)
        s->receiver_protocol == RECEIVER_PROTOCOL_CRSF ? "UART4" : "UART5";
#elif defined(BOARD_HDZERO_HALO)
        s->receiver_protocol == RECEIVER_PROTOCOL_CRSF ? "UART1" : "UART2";
#else
        "UART1";
#endif
    reply("@CFG RECEIVER_CONFIG %s %s %s %lu %lu %lu %lu %lu %lu %u\n",
          s->receiver_protocol == RECEIVER_PROTOCOL_CRSF ? "ELRS" : "SBUS",
          port,
          s->receiver_channel_order == RECEIVER_ORDER_AETR1234
              ? "AETR1234" : "TAER1234",
          (unsigned long)(s->arm_channel + 1U),
          (unsigned long)s->arm_min_us, (unsigned long)s->arm_max_us,
          (unsigned long)(s->beep_channel + 1U),
          (unsigned long)s->beep_min_us, (unsigned long)s->beep_max_us,
          flight_settings_are_saved() ? 1U : 0U);
}

static const char *osd_position_name(uint8_t position)
{
    static const char *const names[9] = {
        "TOP_LEFT", "TOP_CENTER", "TOP_RIGHT",
        "CENTER_LEFT", "CENTER", "CENTER_RIGHT",
        "BOTTOM_LEFT", "BOTTOM_CENTER", "BOTTOM_RIGHT",
    };
    return position < 9U ? names[position] : "CENTER";
}

static void send_osd_status(void)
{
#if BOARD_HAS_OSD
    reply("@CFG OSD_STATUS %u %u %s %s %s %02X %u %u\n",
          max7456_is_available() ? 1U : 0U,
          max7456_is_enabled() ? 1U : 0U,
          osd_position_name(max7456_position()),
          max7456_video_is_pal() ? "PAL" : "NTSC",
          max7456_font_is_ready() ? "FONT_OK" : "FONT_FAILED",
          max7456_probe_value(), max7456_probe_spi_mode(),
          flight_settings_are_saved() ? 1U : 0U);
#endif
}

static void send_vtx_config(void)
{
    const flight_settings_t *s = flight_settings_get();
    static const char bands[] = "ABEFRL";
    const char *protocol = s->vtx_protocol == VTX_PROTOCOL_SMARTAUDIO
        ? "SMARTAUDIO" : s->vtx_protocol == VTX_PROTOCOL_TRAMP ? "TRAMP" :
          s->vtx_protocol == VTX_PROTOCOL_HDZERO_MSP ? "HDZERO_MSP" : "OFF";
    reply("@CFG VTX_CONFIG %s UART%lu %s %c %lu %lu %u\n", protocol,
          (unsigned long)s->vtx_uart,
          s->vtx_region == VTX_REGION_US ? "US" : "EU",
          bands[s->vtx_band], (unsigned long)(s->vtx_channel + 1U),
          (unsigned long)s->vtx_power_mw,
          flight_settings_are_saved() ? 1U : 0U);
    reply("@CFG VTX_STATUS %s\n", vtx_tramp_status_name());
}

static void send_osd_layout(void)
{
#if BOARD_HAS_OSD
    const flight_settings_t *s = flight_settings_get();
    char pilot[OSD_PILOT_NAME_LENGTH + 1U];
    (void)snprintf(pilot, sizeof(pilot), "%s", s->osd_pilot_name);
    for (size_t i = 0U; pilot[i] != '\0'; ++i) {
        if (pilot[i] == ' ') pilot[i] = '_';
    }
    reply("@CFG OSD_LAYOUT %lu %lu %lu %lu %lu %lu %lu %lu %s %u\n",
          (unsigned long)(s->osd_element_enabled_mask |
                          (s->vtx_osd_enabled_mask << OSD_ELEMENT_COUNT)),
          (unsigned long)s->osd_element_positions[0],
          (unsigned long)s->osd_element_positions[1],
          (unsigned long)s->osd_element_positions[2],
          (unsigned long)s->osd_element_positions[3],
          (unsigned long)s->osd_element_positions[4],
          (unsigned long)s->vtx_osd_positions[0],
          (unsigned long)s->vtx_osd_positions[1],
          pilot[0] != '\0' ? pilot : "-",
          flight_settings_are_saved() ? 1U : 0U);
#endif
}

static void send_blackbox_status(void)
{
#if BOARD_HAS_BLACKBOX_STORAGE
    blackbox_sd_diagnostics_t bb;
    blackbox_sd_get_diagnostics(&bb);
    reply("@CFG BLACKBOX_STATUS %s %u %lu %lu %lu %u %lu %lu %u %u %u %02X %06lX\n",
          blackbox_sd_state_name(), blackbox_sd_is_enabled() ? 1U : 0U,
          (unsigned long)blackbox_sd_capacity_mb(),
          (unsigned long)blackbox_sd_written_bytes(),
          (unsigned long)blackbox_sd_dropped_records(),
          flight_settings_are_saved() ? 1U : 0U,
          (unsigned long)blackbox_sd_flight_count(),
          (unsigned long)blackbox_sd_total_bytes(),
          blackbox_sd_is_busy() ? 1U : 0U,
          bb.error_code, bb.error_operation, bb.last_status,
          (unsigned long)bb.error_address);
#endif
}

static void process(const char *command)
{
    if (strcmp(command, "HELLO") == 0) {
        client_active = true;
        last_activity_us = board_micros();
        reply("@CFG HELLO FlightCode 3 %s\n", BOARD_NAME);
        reply("@CFG IMU %s 1\n", imu_get_name());
        reply("@CFG GYRO_RATE %lu\n",
              (unsigned long)imu_get_gyro_rate_hz());
#if BOARD_HAS_CRSF
        reply("@CFG RECEIVER_PROTOCOLS SBUS ELRS\n");
#else
        reply("@CFG RECEIVER_PROTOCOLS SBUS\n");
#endif
#if defined(BOARD_CLRACINGF4)
        reply("@CFG SERIAL_PORTS UART1 UART3 UART4 UART6\n");
#elif defined(BOARD_FLYWOOF405NANO) || defined(BOARD_FLYWOOF405NANO_ANALOG)
        reply("@CFG SERIAL_PORTS UART4 UART5 UART6\n");
#elif defined(BOARD_HDZERO_HALO)
        reply("@CFG SERIAL_PORTS UART1 UART2 UART4\n");
#else
        reply("@CFG SERIAL_PORTS UART1\n");
#endif
#if BOARD_HAS_BATTERY_VOLTAGE
        reply("@CFG CAPABILITIES PIDS MOTOR_TEST TELEMETRY MOTOR_PROTOCOL MAIN_LOOP "
              "BOARD_ALIGNMENT MOTOR_DIRECTION MOTOR_IDLE RATES "
              "FEEDFORWARD TPA FILTERS GYRO_CALIBRATION FLIGHT_LOG PID_SIM DFU REBOOT "
              "TELEMETRY_EXT RECEIVER_CONFIG BATTERY_VOLTAGE OSD "
              "VTX_CONFIG "
#if BOARD_HAS_OSD
              "OSD_LAYOUT "
#endif
#if BOARD_HAS_VBAT_CALIBRATION
              "VBAT_CALIBRATION "
#endif
#if BOARD_HAS_DATAFLASH
              "BLACKBOX_SD BLACKBOX_FLASH BLACKBOX_CATALOG "
#elif BOARD_HAS_SDCARD
              "BLACKBOX_SD BLACKBOX_CATALOG "
#endif
              "\n");
#else
        reply("@CFG CAPABILITIES PIDS MOTOR_TEST TELEMETRY MOTOR_PROTOCOL MAIN_LOOP "
              "BOARD_ALIGNMENT MOTOR_DIRECTION MOTOR_IDLE RATES "
              "FEEDFORWARD TPA FILTERS GYRO_CALIBRATION FLIGHT_LOG PID_SIM DFU REBOOT "
              "TELEMETRY_EXT RECEIVER_CONFIG VTX_CONFIG\n");
#endif
        send_pids();
        send_motor_protocol();
        send_board_alignment();
        send_motor_direction();
        send_motor_idle();
        send_rates();
        send_feedforward();
        send_tpa();
        send_filters();
        send_receiver_config();
        send_vtx_config();
#if BOARD_HAS_VBAT_CALIBRATION
        send_vbat_multiplier();
#endif
#if BOARD_HAS_OSD
        send_osd_status();
        send_osd_layout();
#endif
#if BOARD_HAS_BLACKBOX_STORAGE
        send_blackbox_status();
#endif
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
    if (strcmp(command, "GET_MAIN_LOOP") == 0) {
        last_activity_us = board_micros();
        send_main_loop();
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
    if (strcmp(command, "GET_FILTERS") == 0) {
        last_activity_us = board_micros();
        send_filters();
        send_main_loop();
        return;
    }
    if (strcmp(command, "GET_RECEIVER_CONFIG") == 0) {
        send_receiver_config();
        return;
    }
    if (strcmp(command, "GET_VTX_CONFIG") == 0) {
        send_vtx_config();
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
#if BOARD_HAS_BLACKBOX_STORAGE
        blackbox_sd_diagnostics_t bb;
        blackbox_sd_get_diagnostics(&bb);
        reply("@CFG BLACKBOX_DIAGNOSTICS %06lX %lu %lu %lu %lu %lu %u %u %u %u %u %u %u %u\n",
              (unsigned long)bb.jedec_id,
              (unsigned long)bb.start_calls,
              (unsigned long)bb.start_reject_mask,
              (unsigned long)bb.append_calls,
              (unsigned long)bb.stop_calls,
              (unsigned long)bb.completed_records,
              bb.last_retain, bb.state, bb.operation, bb.queue_count,
              bb.write_bank, bb.retained_bank, bb.erase_active,
              bb.finalise_pending);
#endif
        return;
    }
    if (strcmp(command, "GET_BLACKBOX_STATUS") == 0) {
        if (!flight_control_is_armed() &&
            (blackbox_sd_state() == BLACKBOX_SD_ABSENT ||
             blackbox_sd_state() == BLACKBOX_SD_ERROR)) {
            blackbox_sd_probe();
        }
        send_blackbox_status();
        return;
    }
    if (strcmp(command, "GET_BLACKBOX_CATALOG") == 0) {
        if (flight_control_is_armed() ||
            blackbox_sd_state() == BLACKBOX_SD_RECORDING ||
            blackbox_sd_is_busy()) {
            reply("@CFG ERROR BLACKBOX_RECORDING\n");
            return;
        }
        const uint32_t count = blackbox_sd_flight_count();
        reply("@CFG BLACKBOX_CATALOG %lu\n", (unsigned long)count);
        for (uint32_t i = 0U; i < count; ++i) {
            blackbox_sd_flight_info_t info;
            if (!blackbox_sd_get_flight(i, &info)) break;
            reply("@CFG BLACKBOX_FLIGHT %lu %lu %lu %u\n",
                  (unsigned long)info.flight_id,
                  (unsigned long)info.record_count,
                  (unsigned long)info.block_count,
                  info.stop_flag);
        }
        reply("@CFG BLACKBOX_CATALOG_END\n");
        return;
    }
    if (strcmp(command, "GET_OSD_LAYOUT") == 0) {
        last_activity_us = board_micros();
        send_osd_layout();
        return;
    }
    if (strcmp(command, "GET_VBAT_MULTIPLIER") == 0) {
#if BOARD_HAS_VBAT_CALIBRATION
        last_activity_us = board_micros();
        send_vbat_multiplier();
#else
        reply("@CFG ERROR VBAT_CALIBRATION_UNSUPPORTED\n");
#endif
        return;
    }
    if (strcmp(command, "GET_FLIGHT_LOG_METADATA") == 0) {
        flight_log_metadata_t metadata;
        if (!flight_log_get_metadata(&metadata)) {
            reply("@CFG FLIGHT_LOG_METADATA_UNAVAILABLE\n");
            return;
        }
        reply("@CFG FLIGHT_LOG_METADATA_CORE %lu %lu %lu %lu %lu %lu %lu %u %.2f\n",
              (unsigned long)metadata.version,
              (unsigned long)metadata.main_loop_hz,
              (unsigned long)metadata.gyro_rate_hz,
              (unsigned long)metadata.log_rate_hz,
              (unsigned long)metadata.motor_protocol,
              (unsigned long)metadata.motor_direction_reversed,
              (unsigned long)metadata.receiver_protocol,
              metadata.initial_battery_cells,
              metadata.initial_battery_centivolts / 100.0f);
        reply("@CFG FLIGHT_LOG_METADATA_PIDS %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n",
              metadata.pids[0], metadata.pids[1], metadata.pids[2],
              metadata.pids[3], metadata.pids[4], metadata.pids[5],
              metadata.pids[6], metadata.pids[7], metadata.pids[8]);
        reply("@CFG FLIGHT_LOG_METADATA_TUNING %.2f %.2f %.2f %.4f %.6f %.6f %.6f %.4f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n",
              metadata.rates[0], metadata.rates[1], metadata.rates[2],
              metadata.rates[3], metadata.feedforward[0],
              metadata.feedforward[1], metadata.feedforward[2],
              metadata.tpa[0], metadata.tpa[1], metadata.filters[0],
              metadata.filters[1], metadata.alignment[0],
              metadata.alignment[1], metadata.alignment[2],
              metadata.motor_idle_percent);
        reply("@CFG FLIGHT_LOG_METADATA_END\n");
        return;
    }
    unsigned int metadata_flight;
    if (sscanf(command, "GET_BLACKBOX_METADATA %u", &metadata_flight) == 1) {
        flight_log_metadata_t metadata;
        if (!blackbox_sd_get_metadata((uint32_t)metadata_flight, &metadata)) {
            reply("@CFG BLACKBOX_METADATA_UNAVAILABLE %u\n", metadata_flight);
            return;
        }
        reply("@CFG BLACKBOX_METADATA_CORE %u %lu %lu %lu %lu %lu %lu %lu %u %.2f\n",
              metadata_flight, (unsigned long)metadata.version,
              (unsigned long)metadata.main_loop_hz,
              (unsigned long)metadata.gyro_rate_hz,
              (unsigned long)metadata.log_rate_hz,
              (unsigned long)metadata.motor_protocol,
              (unsigned long)metadata.motor_direction_reversed,
              (unsigned long)metadata.receiver_protocol,
              metadata.initial_battery_cells,
              metadata.initial_battery_centivolts / 100.0f);
        reply("@CFG BLACKBOX_METADATA_PIDS %u %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n",
              metadata_flight, metadata.pids[0], metadata.pids[1],
              metadata.pids[2], metadata.pids[3], metadata.pids[4],
              metadata.pids[5], metadata.pids[6], metadata.pids[7],
              metadata.pids[8]);
        reply("@CFG BLACKBOX_METADATA_TUNING %u %.2f %.2f %.2f %.4f %.6f %.6f %.6f %.4f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n",
              metadata_flight, metadata.rates[0], metadata.rates[1],
              metadata.rates[2], metadata.rates[3], metadata.feedforward[0],
              metadata.feedforward[1], metadata.feedforward[2],
              metadata.tpa[0], metadata.tpa[1], metadata.filters[0],
              metadata.filters[1], metadata.alignment[0],
              metadata.alignment[1], metadata.alignment[2],
              metadata.motor_idle_percent);
        reply("@CFG BLACKBOX_METADATA_END %u\n", metadata_flight);
        return;
    }
    unsigned int blackbox_flight, blackbox_offset, blackbox_count;
    if (sscanf(command, "GET_BLACKBOX_CHUNK %u %u %u",
               &blackbox_flight, &blackbox_offset,
               &blackbox_count) == 3) {
        if (flight_control_is_armed() ||
            blackbox_sd_state() == BLACKBOX_SD_RECORDING ||
            blackbox_sd_is_busy()) {
            reply("@CFG ERROR BLACKBOX_RECORDING\n");
            return;
        }
        if (blackbox_count > BLACKBOX_CHUNK_MAX) {
            blackbox_count = BLACKBOX_CHUNK_MAX;
        }
        uint32_t sent = 0U;
        for (; sent < blackbox_count; ++sent) {
            flight_log_record_t item;
            const uint32_t index = (uint32_t)blackbox_offset + sent;
            if (!blackbox_sd_get_record((uint32_t)blackbox_flight,
                                        index, &item)) break;
            reply("@CFG BLACKBOX_LOG %u %lu "
                  "%d %d %d %d %d %d "
                  "%u %u %u %u %u %u %u %u %u %u %u "
                  "%d %d %d %d %d %d %d %d %d %d %d %d\n",
                  blackbox_flight, (unsigned long)index,
                  item.gyro[0], item.gyro[1], item.gyro[2],
                  item.setpoint[0], item.setpoint[1], item.setpoint[2],
                  item.motor[0], item.motor[1], item.motor[2], item.motor[3],
                  item.throttle, item.flags, item.main_loop_us,
                  item.gyro_loop_us,
                  item.battery_centivolts, item.cell_centivolts,
                  item.battery_cells,
                  item.p_term[0], item.p_term[1], item.p_term[2],
                  item.i_term[0], item.i_term[1], item.i_term[2],
                  item.d_term[0], item.d_term[1], item.d_term[2],
                  item.ff_term[0], item.ff_term[1], item.ff_term[2]);
        }
        reply("@CFG BLACKBOX_CHUNK_END %u %lu\n", blackbox_flight,
              (unsigned long)((uint32_t)blackbox_offset + sent));
        return;
    }
    if (strcmp(command, "CLEAR_BLACKBOX") == 0) {
        if (flight_control_is_armed()) {
            reply("@CFG ERROR ARMED\n");
        } else if (!blackbox_sd_clear()) {
            reply("@CFG ERROR BLACKBOX_BUSY\n");
        } else {
            reply("@CFG OK CLEAR_BLACKBOX\n");
            send_blackbox_status();
        }
        return;
    }
    if (strcmp(command, "BLACKBOX_WRITE_TEST") == 0) {
        if (flight_control_is_armed()) {
            reply("@CFG ERROR ARMED\n");
            return;
        }
        blackbox_sd_write_test_t test;
        if (!blackbox_sd_write_test(&test)) {
            reply("@CFG ERROR BLACKBOX_BUSY\n");
        } else {
            reply("@CFG BLACKBOX_WRITE_TEST %06lX %u %u %u %u %u %02X %02X\n",
                  (unsigned long)test.address, test.before_erased,
                  test.program_ok, test.read_ok, test.verify_ok,
                  test.mismatch_index, test.expected, test.actual);
            send_blackbox_status();
        }
        return;
    }
    if (strcmp(command, "BLACKBOX_SESSION_TEST") == 0) {
        if (flight_control_is_armed()) {
            reply("@CFG ERROR ARMED\n");
        } else if (!blackbox_sd_session_test()) {
            reply("@CFG ERROR BLACKBOX_BUSY\n");
        } else {
            reply("@CFG BLACKBOX_SESSION_TEST STARTED\n");
            send_blackbox_status();
        }
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
            reply("@CFG FLIGHT_LOG %lu "
                  "%d %d %d %d %d %d "
                  "%u %u %u %u %u %u %u %u %u %u %u "
                  "%d %d %d %d %d %d %d %d %d %d %d %d\n",
                  (unsigned long)((uint32_t)log_offset + sent),
                  item.gyro[0], item.gyro[1], item.gyro[2],
                  item.setpoint[0], item.setpoint[1], item.setpoint[2],
                  item.motor[0], item.motor[1],
                  item.motor[2], item.motor[3],
                  item.throttle, item.flags, item.main_loop_us,
                  item.gyro_loop_us,
                  item.battery_centivolts, item.cell_centivolts,
                  item.battery_cells,
                  item.p_term[0], item.p_term[1], item.p_term[2],
                  item.i_term[0], item.i_term[1], item.i_term[2],
                   item.d_term[0], item.d_term[1], item.d_term[2],
                   item.ff_term[0], item.ff_term[1], item.ff_term[2]);
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
    if (strcmp(command, "REBOOT") == 0) {
        if (flight_control_is_armed()) {
            reply("@CFG ERROR ARMED\n");
            return;
        }
        reboot_pending = true;
        reboot_deadline_us = board_micros() + 200000U;
        reply("@CFG OK REBOOT\n");
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
        } else if (arm_mode_active(sbus_get())) {
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
#if BOARD_HAS_VBAT_CALIBRATION
    float vbat_multiplier;
    if (sscanf(command, "SET_VBAT_MULTIPLIER %f", &vbat_multiplier) == 1) {
        settings.vbat_multiplier = vbat_multiplier;
        if (flight_settings_set(&settings)) {
            reply("@CFG OK SET_VBAT_MULTIPLIER\n");
            send_vbat_multiplier();
        } else {
            reply("@CFG ERROR INVALID_VBAT_MULTIPLIER\n");
        }
        return;
    }
#endif
    unsigned int blackbox_enabled;
    if (sscanf(command, "SET_BLACKBOX %u", &blackbox_enabled) == 1) {
#if BOARD_HAS_BLACKBOX_STORAGE
        if (blackbox_enabled > 1U ||
            (blackbox_enabled != 0U &&
             blackbox_sd_state() != BLACKBOX_SD_READY)) {
            reply("@CFG ERROR BLACKBOX_NOT_READY\n");
        } else {
            settings.blackbox_enabled = blackbox_enabled;
            if (!flight_settings_set(&settings)) {
                reply("@CFG ERROR INVALID_BLACKBOX_CONFIG\n");
            } else {
                reply("@CFG OK SET_BLACKBOX\n");
                send_blackbox_status();
            }
        }
#else
        reply("@CFG ERROR BLACKBOX_UNSUPPORTED\n");
#endif
        return;
    }
    unsigned int osd_enabled;
    if (sscanf(command, "SET_OSD_ENABLED %u", &osd_enabled) == 1) {
        settings.osd_enabled = osd_enabled;
        if (osd_enabled > 1U || !flight_settings_set(&settings)) {
            reply("@CFG ERROR INVALID_OSD_CONFIG\n");
        } else {
            reply("@CFG OK SET_OSD_ENABLED\n");
            send_osd_status();
        }
        return;
    }
    char osd_position[20];
    if (sscanf(command, "SET_OSD %u %19s", &osd_enabled, osd_position) == 2) {
        static const char *const names[9] = {
            "TOP_LEFT", "TOP_CENTER", "TOP_RIGHT",
            "CENTER_LEFT", "CENTER", "CENTER_RIGHT",
            "BOTTOM_LEFT", "BOTTOM_CENTER", "BOTTOM_RIGHT",
        };
        uint8_t position = 9U;
        for (uint8_t i = 0U; i < 9U; ++i) {
            if (strcmp(osd_position, names[i]) == 0) position = i;
        }
        settings.osd_enabled = osd_enabled;
        settings.osd_position = position;
        static const uint32_t legacy_layout_positions[9] = {
            31U, 41U, 52U, 181U, 191U, 202U, 331U, 341U, 352U,
        };
        if (position < 9U) {
            settings.osd_element_positions[OSD_ELEMENT_TOTAL_VOLTAGE] =
                legacy_layout_positions[position];
            if (osd_enabled != 0U) settings.osd_element_enabled_mask |= 1U;
            else settings.osd_element_enabled_mask &= ~1U;
        }
        if (osd_enabled > 1U || position > 8U ||
            !flight_settings_set(&settings)) {
            reply("@CFG ERROR INVALID_OSD_CONFIG\n");
        } else {
            reply("@CFG OK SET_OSD\n");
            send_osd_status();
        }
        return;
    }
    unsigned int osd_mask, osd_positions[OSD_ELEMENT_COUNT], vtx_osd_positions[2];
    char osd_pilot[OSD_PILOT_NAME_LENGTH + 1U];
    if (sscanf(command, "SET_OSD_LAYOUT %u %u %u %u %u %u %u %u %12s",
               &osd_mask, &osd_positions[0], &osd_positions[1],
               &osd_positions[2], &osd_positions[3], &osd_positions[4],
               &vtx_osd_positions[0], &vtx_osd_positions[1], osd_pilot) == 9) {
        settings.osd_element_enabled_mask = osd_mask &
            ((1U << OSD_ELEMENT_COUNT) - 1U);
        settings.vtx_osd_enabled_mask = (osd_mask >> OSD_ELEMENT_COUNT) & 3U;
        for (uint8_t i = 0U; i < OSD_ELEMENT_COUNT; ++i)
            settings.osd_element_positions[i] = osd_positions[i];
        settings.vtx_osd_positions[0] = vtx_osd_positions[0];
        settings.vtx_osd_positions[1] = vtx_osd_positions[1];
        if (strcmp(osd_pilot, "-") == 0) osd_pilot[0] = '\0';
        for (size_t i = 0U; osd_pilot[i] != '\0'; ++i)
            if (osd_pilot[i] == '_') osd_pilot[i] = ' ';
        (void)snprintf(settings.osd_pilot_name,
                       sizeof(settings.osd_pilot_name), "%s", osd_pilot);
        reply(flight_settings_set(&settings) ? "@CFG OK SET_OSD_LAYOUT\n" :
              "@CFG ERROR INVALID_OSD_LAYOUT\n");
        send_osd_layout();
        return;
    }
    if (sscanf(command, "SET_OSD_LAYOUT %u %u %u %u %u %u %12s",
               &osd_mask, &osd_positions[0], &osd_positions[1],
               &osd_positions[2], &osd_positions[3], &osd_positions[4],
               osd_pilot) == 7) {
        settings.osd_element_enabled_mask = osd_mask;
        for (uint8_t i = 0U; i < OSD_ELEMENT_COUNT; ++i) {
            settings.osd_element_positions[i] = osd_positions[i];
        }
        if (strcmp(osd_pilot, "-") == 0) osd_pilot[0] = '\0';
        for (size_t i = 0U; osd_pilot[i] != '\0'; ++i) {
            if (osd_pilot[i] == '_') osd_pilot[i] = ' ';
        }
        (void)snprintf(settings.osd_pilot_name,
                       sizeof(settings.osd_pilot_name), "%s", osd_pilot);
        if (!flight_settings_set(&settings)) {
            reply("@CFG ERROR INVALID_OSD_LAYOUT\n");
        } else {
            reply("@CFG OK SET_OSD_LAYOUT\n");
            send_osd_layout();
        }
        return;
    }
    char receiver_protocol[8], receiver_port[8], receiver_order[16];
    unsigned int arm_channel, arm_min, arm_max;
    unsigned int beep_channel, beep_min, beep_max;
    if (sscanf(command, "SET_RECEIVER_CONFIG %7s %7s %15s %u %u %u %u %u %u",
               receiver_protocol, receiver_port, receiver_order, &arm_channel,
               &arm_min, &arm_max, &beep_channel, &beep_min, &beep_max) == 9) {
        const char *required_port =
#if defined(BOARD_FLYWOOF405NANO) || defined(BOARD_FLYWOOF405NANO_ANALOG)
            strcmp(receiver_protocol, "ELRS") == 0 ? "UART4" : "UART5";
#elif defined(BOARD_HDZERO_HALO)
            strcmp(receiver_protocol, "ELRS") == 0 ? "UART1" : "UART2";
#else
            "UART1";
#endif
        if (strcmp(receiver_port, required_port) != 0) {
            reply("@CFG ERROR INVALID_RECEIVER_PORT\n");
            return;
        }
        char compatible[160];
        (void)snprintf(compatible, sizeof(compatible),
                       "SET_RECEIVER_CONFIG %s %s %u %u %u %u %u %u",
                       receiver_protocol, receiver_order, arm_channel, arm_min,
                       arm_max, beep_channel, beep_min, beep_max);
        process(compatible);
        return;
    }
    char vtx_protocol[16], vtx_port[8], vtx_region[4], vtx_band;
    unsigned int vtx_uart, vtx_channel, vtx_power;
    if (sscanf(command, "SET_VTX_CONFIG %15s %7s %3s %c %u %u",
               vtx_protocol, vtx_port, vtx_region, &vtx_band,
               &vtx_channel, &vtx_power) == 6) {
        if (strcmp(vtx_protocol, "OFF") == 0) settings.vtx_protocol = VTX_PROTOCOL_OFF;
        else if (strcmp(vtx_protocol, "SMARTAUDIO") == 0) settings.vtx_protocol = VTX_PROTOCOL_SMARTAUDIO;
        else if (strcmp(vtx_protocol, "TRAMP") == 0) settings.vtx_protocol = VTX_PROTOCOL_TRAMP;
        else if (strcmp(vtx_protocol, "HDZERO_MSP") == 0) settings.vtx_protocol = VTX_PROTOCOL_HDZERO_MSP;
        else { reply("@CFG ERROR INVALID_VTX_PROTOCOL\n"); return; }
        if (sscanf(vtx_port, "UART%u", &vtx_uart) != 1 ||
            (vtx_uart < 1U || vtx_uart > 6U)) {
            reply("@CFG ERROR INVALID_VTX_PORT\n"); return;
        }
        settings.vtx_uart = vtx_uart;
        if (strcmp(vtx_region, "EU") == 0) settings.vtx_region = VTX_REGION_EU;
        else if (strcmp(vtx_region, "US") == 0) settings.vtx_region = VTX_REGION_US;
        else { reply("@CFG ERROR INVALID_VTX_REGION\n"); return; }
        const char *band_position = strchr("ABEFRL", vtx_band);
        if (band_position == NULL || vtx_channel < 1U || vtx_channel > 8U ||
            vtx_power < 1U || vtx_power > 2000U) {
            reply("@CFG ERROR INVALID_VTX_CHANNEL\n"); return;
        }
        settings.vtx_band = (uint32_t)(band_position - "ABEFRL");
        settings.vtx_channel = vtx_channel - 1U;
        settings.vtx_power_mw = vtx_power;
        reply(flight_settings_set(&settings) ? "@CFG OK SET_VTX_CONFIG\n" : "@CFG ERROR INVALID_VTX_CONFIG\n");
        send_vtx_config();
        return;
    }
    if (sscanf(command, "SET_RECEIVER_CONFIG %7s %15s %u %u %u %u %u %u",
               receiver_protocol, receiver_order, &arm_channel, &arm_min, &arm_max,
               &beep_channel, &beep_min, &beep_max) == 8) {
        if (strcmp(receiver_protocol, "SBUS") == 0) {
            settings.receiver_protocol = RECEIVER_PROTOCOL_SBUS;
        } else if (strcmp(receiver_protocol, "ELRS") == 0) {
            settings.receiver_protocol = RECEIVER_PROTOCOL_CRSF;
        } else {
            reply("@CFG ERROR INVALID_RECEIVER_CONFIG\n");
            return;
        }
        if (strcmp(receiver_order, "TAER1234") == 0) {
            settings.receiver_channel_order = RECEIVER_ORDER_TAER1234;
        } else if (strcmp(receiver_order, "AETR1234") == 0) {
            settings.receiver_channel_order = RECEIVER_ORDER_AETR1234;
        } else {
            reply("@CFG ERROR INVALID_RECEIVER_CONFIG\n");
            return;
        }
        if (arm_channel < 5U || arm_channel > 16U ||
            beep_channel < 5U || beep_channel > 16U) {
            reply("@CFG ERROR INVALID_RECEIVER_CONFIG\n");
            return;
        }
        settings.arm_channel = arm_channel - 1U;
        settings.arm_min_us = arm_min;
        settings.arm_max_us = arm_max;
        settings.beep_channel = beep_channel - 1U;
        settings.beep_min_us = beep_min;
        settings.beep_max_us = beep_max;
        reply(flight_settings_set(&settings)
                  ? "@CFG OK SET_RECEIVER_CONFIG\n"
                  : "@CFG ERROR INVALID_RECEIVER_CONFIG\n");
        send_receiver_config();
        return;
    }
    if (sscanf(command, "SET_RECEIVER_CONFIG %15s %u %u %u %u %u %u",
               receiver_order, &arm_channel, &arm_min, &arm_max,
               &beep_channel, &beep_min, &beep_max) == 7) {
        settings.receiver_protocol = RECEIVER_PROTOCOL_SBUS;
        if (strcmp(receiver_order, "TAER1234") == 0)
            settings.receiver_channel_order = RECEIVER_ORDER_TAER1234;
        else if (strcmp(receiver_order, "AETR1234") == 0)
            settings.receiver_channel_order = RECEIVER_ORDER_AETR1234;
        else {
            reply("@CFG ERROR INVALID_RECEIVER_CONFIG\n");
            return;
        }
        if (arm_channel < 5U || arm_channel > 16U ||
            beep_channel < 5U || beep_channel > 16U) {
            reply("@CFG ERROR INVALID_RECEIVER_CONFIG\n");
            return;
        }
        settings.arm_channel = arm_channel - 1U;
        settings.arm_min_us = arm_min;
        settings.arm_max_us = arm_max;
        settings.beep_channel = beep_channel - 1U;
        settings.beep_min_us = beep_min;
        settings.beep_max_us = beep_max;
        reply(flight_settings_set(&settings)
                  ? "@CFG OK SET_RECEIVER_CONFIG\n"
                  : "@CFG ERROR INVALID_RECEIVER_CONFIG\n");
        send_receiver_config();
        return;
    }
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
    if (sscanf(command, "SET_FILTERS %f %f",
               &settings.gyro_lpf_hz,
               &settings.dterm_lpf_hz) == 2) {
        if (flight_settings_set(&settings)) {
            reply("@CFG OK SET_FILTERS\n");
            send_filters();
        } else {
            reply("@CFG ERROR INVALID_FILTERS\n");
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
        send_filters();
        send_main_loop();
        return;
    }
    if (strcmp(command, "RESET_PIDS") == 0) {
        settings = *flight_settings_get();
        flight_settings_reset_tuning_defaults(&settings);
        reply(flight_settings_set(&settings) ? "@CFG OK RESET_PIDS\n"
                                             : "@CFG ERROR RESET_PIDS\n");
        send_pids();
        send_rates();
        send_feedforward();
        send_tpa();
        send_filters();
        return;
    }
    unsigned int main_loop_hz;
    if (sscanf(command, "SET_MAIN_LOOP %u", &main_loop_hz) == 1) {
        settings = *flight_settings_get();
        settings.main_loop_hz = main_loop_hz;
        if (flight_control_is_armed()) {
            reply("@CFG ERROR ARMED\n");
        } else if (flight_settings_set(&settings)) {
            reply("@CFG OK SET_MAIN_LOOP REBOOT_REQUIRED\n");
            send_main_loop();
        } else {
            reply("@CFG ERROR INVALID_MAIN_LOOP\n");
        }
        return;
    }
    char protocol_name[24];
    if (sscanf(command, "SET_MOTOR_PROTOCOL %23s", protocol_name) == 1) {
        settings = *flight_settings_get();
        if (strcmp(protocol_name, "DSHOT300") == 0) {
            settings.motor_protocol = MOTOR_PROTOCOL_DSHOT300;
        } else if (strcmp(protocol_name, "DSHOT600") == 0) {
            settings.motor_protocol = MOTOR_PROTOCOL_DSHOT600;
        } else if (strcmp(protocol_name, "DSHOT1200") == 0) {
            settings.motor_protocol = MOTOR_PROTOCOL_DSHOT1200;
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
    reboot_pending = false;
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
    if (reboot_pending &&
        (int32_t)(board_micros() - reboot_deadline_us) >= 0) {
        NVIC_SystemReset();
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
        arm_mode_active(receiver) ||
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
                         " %u", rx->valid ? rx->channel_us[i] : 0U);
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
        /* Raw XYZ values support stationary calibration diagnostics. */
        used += snprintf(output + used, sizeof(output) - (size_t)used,
                          " %.3f %.3f %.3f %u",
                          imu->gyro_x_dps,
                          imu->gyro_y_dps,
                          imu->gyro_z_dps,
                          flight_control_get_calibration_samples());
    }
    if (used > 0 && (size_t)used < sizeof(output) - 1U) {
        output[used++] = '\n';
        usb_cdc_write((const uint8_t *)output, (size_t)used);
    }
#if BOARD_HAS_BATTERY_VOLTAGE
    static uint32_t last_battery_us;
    const uint32_t now = board_micros();
    if ((uint32_t)(now - last_battery_us) >= 200000U) {
        last_battery_us = now;
        const float voltage = board_battery_voltage();
        reply("@CFG BATTERY_VOLTAGE %.2f\n", voltage >= 1.0f ? voltage : 0.0f);
    }
#endif
    static uint32_t last_osd_status_us;
    const uint32_t osd_status_now = board_micros();
    if ((uint32_t)(osd_status_now - last_osd_status_us) >= 1000000U) {
        last_osd_status_us = osd_status_now;
        send_osd_status();
    }
    static uint32_t last_sbus_diagnostics_us;
    const uint32_t diagnostics_now = board_micros();
    if ((uint32_t)(diagnostics_now - last_sbus_diagnostics_us) >= 500000U) {
        last_sbus_diagnostics_us = diagnostics_now;
        reply("@CFG SBUS_DIAGNOSTICS %u %lu %lu %lu %lu %lu %lu\n",
              rx->valid ? 1U : 0U,
              (unsigned long)(rx->frame_age_us == UINT32_MAX
                                  ? UINT32_MAX : rx->frame_age_us / 1000U),
              (unsigned long)rx->valid_frame_count,
              (unsigned long)rx->uart_error_count,
              (unsigned long)rx->recovery_count,
              (unsigned long)rx->ring_overrun_count,
              (unsigned long)rx->invalid_frame_count);
    }
}
