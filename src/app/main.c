#include "board.h"
#include "config_protocol.h"
#include "dshot.h"
#include "flight_control.h"
#include "flight_log.h"
#include "flight_settings.h"
#include "mpu6000.h"
#include "sbus.h"

#define LOOP_HZ 8000U
#define IMU_FAILURE_LIMIT 8U

int main(void)
{
    board_check_dfu_request();
    board_init();
    config_protocol_init();
    dshot_init();
    flight_settings_init();
    sbus_init();
    flight_control_init();

    bool imu_ready = mpu6000_init();
    uint8_t consecutive_imu_failures = 0U;

    uint16_t motors[4] = {0};
    const uint32_t loop_cycles = SystemCoreClock / LOOP_HZ;
    uint32_t next_loop = DWT->CYCCNT;
    uint32_t last_telemetry_us = 0U;
    uint32_t last_imu_retry_us = board_micros();
    uint32_t previous_loop_us = board_micros();
    uint32_t loop_window_start_us = previous_loop_us;
    uint32_t loop_window_count = 0U;
    uint32_t max_loop_period_us = 0U;
    float measured_loop_hz = (float)LOOP_HZ;

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
        sbus_update();
        const sbus_data_t *receiver = sbus_get();
        const flight_settings_t *settings = flight_settings_get();
        board_buzzer_set(receiver->valid &&
            receiver->channel_us[settings->beep_channel] >=
                settings->beep_min_us &&
            receiver->channel_us[settings->beep_channel] <=
                settings->beep_max_us);
        config_protocol_update();

        imu_sample_t imu = {0};
        if (imu_ready && mpu6000_read(&imu)) {
            consecutive_imu_failures = 0U;
            const float control_dt =
                loop_period_us > 0U ? (float)loop_period_us * 0.000001f
                                    : (1.0f / (float)LOOP_HZ);
            flight_control_update(&imu, receiver, control_dt, motors);
        } else {
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
        if (config_protocol_motor_output_suppressed()) {
            const uint16_t suppressed_motors[4] = {0U, 0U, 0U, 0U};
            dshot_write(suppressed_motors);
        } else {
            dshot_write(motors);
        }
        flight_log_persist_if_ready();

        const uint32_t now_us = board_micros();
        if (!imu_ready &&
            (uint32_t)(now_us - last_imu_retry_us) >= 2000000U) {
            last_imu_retry_us = now_us;
            imu_ready = mpu6000_init();
            consecutive_imu_failures = 0U;
            next_loop = DWT->CYCCNT;
        }
        if ((uint32_t)(now_us - last_telemetry_us) >= 10000U) {
            last_telemetry_us = now_us;
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
