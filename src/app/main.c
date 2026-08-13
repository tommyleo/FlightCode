#include "board.h"
#include "config_protocol.h"
#include "dshot.h"
#include "flight_control.h"
#include "flight_log.h"
#include "flight_settings.h"
#include "imu.h"
#include "max7456.h"
#include "osd_tuning_menu.h"
#include "sbus.h"
#include "blackbox_sd.h"

#define IMU_FAILURE_LIMIT 8U

typedef struct { uint32_t phase; uint32_t rate_hz; } loop_task_t;

static bool task_due(loop_task_t *task, uint32_t loop_hz)
{
    task->phase += task->rate_hz;
    if (task->phase < loop_hz) return false;
    task->phase -= loop_hz;
    return true;
}

int main(void)
{
    board_check_dfu_request();
    board_init();
    blackbox_sd_init();
    config_protocol_init();
    dshot_init();
    flight_settings_init();
    dshot_startup_sequence();
    sbus_init();
    flight_control_init();
    max7456_init();
    osd_tuning_menu_init();

    bool imu_ready = imu_init(flight_settings_get()->main_loop_hz);
    uint8_t consecutive_imu_failures = 0U;

    uint16_t motors[4] = {0};
    const uint32_t loop_hz = flight_settings_get()->main_loop_hz;
    const uint32_t loop_cycles = SystemCoreClock / loop_hz;
    uint32_t next_loop = DWT->CYCCNT;
    uint32_t scheduler_tick = 0U;
    uint8_t osd_retry_ticks = 0U;
    loop_task_t service_task = {0U, 1000U};
    loop_task_t telemetry_task = {0U, 100U};
    loop_task_t osd_task = {0U, 5U};
    loop_task_t imu_retry_task = {0U, 1U};
    loop_task_t esc_task = {0U, loop_hz < 16000U ? loop_hz : 16000U};
#if BOARD_IMU_TYPE == IMU_TYPE_MPU6000
    loop_task_t imu_task = {0U, 8000U};
#else
    loop_task_t imu_task = {0U, loop_hz};
#endif
    uint32_t previous_loop_us = board_micros();
    uint32_t loop_window_start_us = previous_loop_us;
    uint32_t loop_window_count = 0U;
    uint32_t max_loop_period_us = 0U;
    float measured_loop_hz = (float)loop_hz;

    while (1) {
        const uint32_t loop_start_us = board_micros();
        const uint32_t loop_period_us = loop_start_us - previous_loop_us;
        previous_loop_us = loop_start_us;
        if (loop_period_us > max_loop_period_us) {
            max_loop_period_us = loop_period_us;
        }
        ++loop_window_count;
        const uint32_t loop_window_us = loop_start_us - loop_window_start_us;
        if (loop_window_us >= 100000U) {
            measured_loop_hz =
                (float)loop_window_count * 1000000.0f / (float)loop_window_us;
            loop_window_count = 0U;
            loop_window_start_us = loop_start_us;
        }
        next_loop += loop_cycles;
        ++scheduler_tick;
        if (scheduler_tick >= loop_hz) {
            scheduler_tick = 0U;
        }
        const bool service_due = task_due(&service_task, loop_hz);
        const bool esc_due = task_due(&esc_task, loop_hz);
        sbus_update();
        const sbus_data_t *receiver = sbus_get();
        const flight_settings_t *settings = flight_settings_get();
        if (service_due) {
            board_status_led_update(receiver->valid,
                                    !flight_control_is_calibrated());
            osd_tuning_menu_update(receiver, flight_control_is_armed());
        }
        const bool tuning_menu_active = osd_tuning_menu_is_active();
        if (service_due) {
            config_protocol_update();
        }
        const bool radio_beep = receiver->valid &&
            receiver->channel_us[settings->beep_channel] >=
                settings->beep_min_us &&
            receiver->channel_us[settings->beep_channel] <=
                settings->beep_max_us;
        if (service_due) {
            board_buzzer_update(radio_beep);
        }

        static imu_sample_t imu;
        const bool imu_read_due = task_due(&imu_task, loop_hz);
        const bool imu_updated = imu_ready && imu_read_due && imu_read(&imu);
        if (imu_updated) {
            consecutive_imu_failures = 0U;
            const float control_dt = 1.0f / (float)loop_hz;
            if (!tuning_menu_active) {
                flight_control_update(&imu, receiver, control_dt, motors);
            } else {
                motors[0] = motors[1] = motors[2] = motors[3] = 0U;
            }
        } else if (imu_read_due) {
            if (imu_ready &&
                consecutive_imu_failures < IMU_FAILURE_LIMIT) {
                ++consecutive_imu_failures;
            }
            if (!imu_ready ||
                consecutive_imu_failures >= IMU_FAILURE_LIMIT) {
                motors[0] = motors[1] = motors[2] = motors[3] = 0;
                flight_control_emergency_stop(
                    FLIGHT_LOG_FLAG_STOP_IMU);
                imu_ready = false;
            }
        }
        config_protocol_apply_motor_test(receiver, motors);
        if (tuning_menu_active) {
            motors[0] = motors[1] = motors[2] = motors[3] = 0U;
        }
        if (config_protocol_motor_output_suppressed()) {
            const uint16_t suppressed_motors[4] = {0U, 0U, 0U, 0U};
            dshot_write(suppressed_motors);
        } else if (esc_due) {
            dshot_write(motors);
        }
        flight_log_persist_if_ready();
        blackbox_sd_update();
        if (service_due) {
            board_battery_update();
        }

        if (task_due(&osd_task, loop_hz)) {
            const float battery_voltage = board_battery_voltage();
            flight_log_set_battery_voltage(battery_voltage);
            if (osd_retry_ticks < 5U) {
                ++osd_retry_ticks;
            }
            if (!max7456_is_available() && !flight_control_is_armed() &&
                battery_voltage >= 3.0f && osd_retry_ticks >= 5U) {
                osd_retry_ticks = 0U;
                (void)max7456_init();
                next_loop = DWT->CYCCNT;
            }
            max7456_update_battery(battery_voltage);
        }
        if (!imu_ready && task_due(&imu_retry_task, loop_hz)) {
            imu_ready = imu_init(loop_hz);
            consecutive_imu_failures = 0U;
            next_loop = DWT->CYCCNT;
        }
        if (task_due(&telemetry_task, loop_hz)) {
            config_protocol_send_telemetry(receiver, &imu, motors,
                                           measured_loop_hz,
                                           max_loop_period_us);
            max_loop_period_us = 0U;
        }

        while ((int32_t)(DWT->CYCCNT - next_loop) < 0) {
            __NOP();
        }
    }
}
