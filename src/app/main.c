#include "main.h"

#include "board.h"
#include "config_protocol.h"
#include "dshot.h"
#include "flight_control.h"
#include "flight_log.h"
#include "flight_settings.h"
#include "imu.h"
#include "max7456.h"
#include "msp_displayport.h"
#include "osd_tuning_menu.h"
#include "sbus.h"
#include "blackbox_sd.h"
#include "usb_cdc.h"
#include "vtx_tramp.h"

#define IMU_FAILURE_LIMIT 8U

static uint16_t timing_us(uint32_t period_us)
{
    return period_us > UINT16_MAX ? UINT16_MAX : (uint16_t)period_us;
}

static bool task_due(loop_task_t *task, uint32_t loop_hz)
{
    task->phase += task->rate_hz;
    if (task->phase < loop_hz) return false;
    task->phase -= loop_hz;
    return true;
}

static void main_loop_state_init(main_loop_state_t *state)
{
    state->loop_hz = flight_settings_get()->main_loop_hz;
    state->imu_ready = imu_init(state->loop_hz);
    state->loop_cycles = SystemCoreClock / state->loop_hz;
    state->next_loop = DWT->CYCCNT;
    state->service_task = (loop_task_t){0U, 1000U};
    state->telemetry_task = (loop_task_t){0U, 100U};
    state->osd_task = (loop_task_t){0U, 5U};
    state->imu_retry_task = (loop_task_t){0U, 1U};
    state->esc_task = (loop_task_t){
        0U, state->loop_hz < 16000U ? state->loop_hz : 16000U};
    state->imu_task = (loop_task_t){0U, imu_get_gyro_rate_hz()};
    state->previous_loop_us = board_micros();
    state->previous_gyro_update_us = 0U;
    state->loop_window_start_us = state->previous_loop_us;
    state->measured_loop_hz = (float)state->loop_hz;
}

static void increment_osd_retry_ticks(main_loop_state_t *state)
{
    if (state->osd_retry_ticks < 5U) {
        ++state->osd_retry_ticks;
    }
}

static void retry_osd_if_needed(main_loop_state_t *state,
                                float battery_voltage)
{
    if (!max7456_is_available() && !flight_control_is_armed() &&
        battery_voltage >= 3.0f && state->osd_retry_ticks >= 5U) {
        state->osd_retry_ticks = 0U;
        (void)max7456_init();
        state->next_loop = DWT->CYCCNT;
    }
}

static void update_osd_if_due(main_loop_state_t *state)
{
    if (task_due(&state->osd_task, state->loop_hz)) {
        const float battery_voltage = board_battery_voltage();
        flight_log_set_battery_voltage(battery_voltage);
        increment_osd_retry_ticks(state);
        retry_osd_if_needed(state, battery_voltage);
        max7456_update(battery_voltage, flight_control_is_armed(),
                       board_micros());
        msp_displayport_update(battery_voltage, flight_control_is_armed(),
                               board_micros());
    }
}

static void main_loop_step(main_loop_state_t *state)
{
    const uint32_t loop_start_us = board_micros();
    const uint32_t loop_period_us = loop_start_us - state->previous_loop_us;
    state->previous_loop_us = loop_start_us;
    if (loop_period_us > state->max_loop_period_us) {
        state->max_loop_period_us = loop_period_us;
    }
    ++state->loop_window_count;
    const uint32_t loop_window_us =
        loop_start_us - state->loop_window_start_us;
    if (loop_window_us >= 100000U) {
        state->measured_loop_hz =
            (float)state->loop_window_count * 1000000.0f /
            (float)loop_window_us;
        state->loop_window_count = 0U;
        state->loop_window_start_us = loop_start_us;
    }
    state->next_loop += state->loop_cycles;
    const bool service_due = task_due(&state->service_task, state->loop_hz);
    const bool esc_due = task_due(&state->esc_task, state->loop_hz);
    sbus_update();
    const sbus_data_t *receiver = sbus_get();
    const flight_settings_t *settings = flight_settings_get();
    if (service_due) {
        board_status_led_update(receiver->valid,
                                !flight_control_is_calibrated());
        osd_tuning_menu_update(receiver, flight_control_is_armed());
        vtx_tramp_update(flight_control_is_armed());
    }
    const bool tuning_menu_active = osd_tuning_menu_is_active();
    if (service_due) {
        config_protocol_update();
    }
    /* Keep the USB TX endpoint fed at the control-loop rate. Blackbox
     * downloads otherwise drain only one 64-byte packet per 1 kHz service
     * tick, limiting an idle, disarmed controller to about 64 kB/s. */
    usb_cdc_poll();
    const bool radio_beep = receiver->valid &&
        receiver->channel_us[settings->beep_channel] >= settings->beep_min_us &&
        receiver->channel_us[settings->beep_channel] <= settings->beep_max_us;
    if (service_due) {
        board_buzzer_update(radio_beep);
    }

    const bool imu_read_due = task_due(&state->imu_task, state->loop_hz);
    const bool imu_updated =
        state->imu_ready && imu_read_due && imu_read(&state->imu);
    if (imu_updated) {
        state->consecutive_imu_failures = 0U;
        const float control_dt = 1.0f / (float)state->imu_task.rate_hz;
        const uint32_t gyro_period_us = state->previous_gyro_update_us == 0U
            ? 1000000U / state->imu_task.rate_hz
            : loop_start_us - state->previous_gyro_update_us;
        state->previous_gyro_update_us = loop_start_us;
        if (!tuning_menu_active) {
            flight_control_update(
                &state->imu, receiver, control_dt,
                timing_us(loop_period_us), timing_us(gyro_period_us),
                state->motors);
        } else {
            state->motors[0] = state->motors[1] =
                state->motors[2] = state->motors[3] = 0U;
        }
    } else if (imu_read_due) {
        if (state->imu_ready &&
            state->consecutive_imu_failures < IMU_FAILURE_LIMIT) {
            ++state->consecutive_imu_failures;
        }
        if (!state->imu_ready ||
            state->consecutive_imu_failures >= IMU_FAILURE_LIMIT) {
            state->motors[0] = state->motors[1] =
                state->motors[2] = state->motors[3] = 0U;
            flight_control_emergency_stop(FLIGHT_LOG_FLAG_STOP_IMU);
            state->imu_ready = false;
        }
    }
    config_protocol_apply_motor_test(receiver, state->motors);
    if (tuning_menu_active) {
        state->motors[0] = state->motors[1] =
            state->motors[2] = state->motors[3] = 0U;
    }
    if (config_protocol_motor_output_suppressed()) {
        const uint16_t suppressed_motors[4] = {0U, 0U, 0U, 0U};
        dshot_write(suppressed_motors);
    } else if (esc_due) {
        dshot_write(state->motors);
    }
    flight_log_persist_if_ready();
    blackbox_sd_update();
    msp_displayport_process();
    if (service_due) {
        board_battery_update();
    }

    update_osd_if_due(state);
    if (!state->imu_ready &&
        task_due(&state->imu_retry_task, state->loop_hz)) {
            state->imu_ready = imu_init(state->loop_hz);
            state->consecutive_imu_failures = 0U;
            state->previous_gyro_update_us = 0U;
            state->next_loop = DWT->CYCCNT;
    }
    if (task_due(&state->telemetry_task, state->loop_hz)) {
        config_protocol_send_telemetry(
            receiver, &state->imu, state->motors, state->measured_loop_hz,
            state->max_loop_period_us);
        state->max_loop_period_us = 0U;
    }

    while ((int32_t)(DWT->CYCCNT - state->next_loop) < 0) {
        __NOP();
    }
}

int main(void)
{
    board_check_dfu_request();
    board_init();
    blackbox_sd_init();
    config_protocol_init();
    dshot_init();
    flight_settings_init();
    (void)vtx_tramp_init();
    sbus_init();
    flight_control_init();
    max7456_init();
    (void)msp_displayport_init();
    osd_tuning_menu_init();

    main_loop_state_t loop = {0};
    main_loop_state_init(&loop);
    dshot_startup_sequence();
    loop.next_loop = DWT->CYCCNT;
    loop.previous_loop_us = board_micros();
    loop.loop_window_start_us = loop.previous_loop_us;

    while (1) {
        main_loop_step(&loop);
    }
}
