#include "flight_control.h"

#include <math.h>
#include <string.h>

#include "board.h"
#include "dshot.h"
#include "flight_log.h"
#include "flight_settings.h"

#define CALIBRATION_SAMPLES 8000U
#define CALIBRATION_MAX_VARIATION_DPS 3.0f
#define CALIBRATION_MAX_ABSOLUTE_RATE_DPS 10.0f
#define CALIBRATION_MAX_CONSECUTIVE_OUTLIERS 16U
#define CALIBRATION_TIMEOUT_US 5000000U
#define GYRO_LPF_HZ 150.0f
#define DTERM_LPF_HZ 100.0f
#define ARM_THROTTLE_MAX_PERCENT 5.0f

typedef struct {
    float integral;
    float previous_rate;
    float dterm;
} pid_state_t;

typedef struct {
    float value;
    bool initialized;
} pt1_filter_t;

static pid_gains_t roll_gains = {0.090f, 0.200f, 0.0012f};
static pid_gains_t pitch_gains = {0.090f, 0.200f, 0.0012f};
static pid_gains_t yaw_gains = {0.120f, 0.200f, 0.0f};
static pid_state_t roll_state, pitch_state, yaw_state;
static float bias_x, bias_y, bias_z;
static float sum_x, sum_y, sum_z;
static float calibration_reference_x;
static float calibration_reference_y;
static float calibration_reference_z;
static uint16_t calibration_count;
static uint16_t calibration_outliers;
static uint32_t calibration_started_us;
static bool calibrated;
static bool calibration_failed;
static bool armed;
static bool arm_switch_was_low;
static bool arm_calibration_active;
static bool motor_direction_reversed;
static float motor_idle_percent = 3.0f;
static float roll_rate_dps = 500.0f;
static float pitch_rate_dps = 500.0f;
static float yaw_rate_dps = 400.0f;
static float rate_expo = 0.35f;
static float roll_feedforward = 0.025f;
static float pitch_feedforward = 0.025f;
static float yaw_feedforward = 0.015f;
static float tpa_attenuation;
static float tpa_breakpoint_percent = 65.0f;
static bool mixer_saturated;
static pt1_filter_t gyro_filter[3];

static float clampf(float value, float min, float max)
{
    return value < min ? min : (value > max ? max : value);
}

static float stick(uint16_t us)
{
    const float value = ((float)us - 1500.0f) / 500.0f;
    return clampf(value, -1.0f, 1.0f);
}

static float rate_setpoint(uint16_t us, float max_rate_dps)
{
    const float x = stick(us);
    const float curved = (1.0f - rate_expo) * x + rate_expo * x * x * x;
    return curved * max_rate_dps;
}

static float pt1(pt1_filter_t *filter, float input, float cutoff_hz, float dt)
{
    if (!filter->initialized) {
        filter->value = input;
        filter->initialized = true;
        return input;
    }
    const float rc = 1.0f / (2.0f * 3.14159265358979323846f * cutoff_hz);
    filter->value += (dt / (rc + dt)) * (input - filter->value);
    return filter->value;
}

static float pid(pid_state_t *state, const pid_gains_t *gains,
                 float setpoint, float rate, float feedforward,
                 float dt, float limit, float tpa_factor)
{
    const float error = setpoint - rate;
    if (!mixer_saturated || state->integral * error < 0.0f) {
        state->integral =
            clampf(state->integral + gains->ki * error * dt, -30.0f, 30.0f);
    }
    const float derivative = -(rate - state->previous_rate) / dt;
    state->previous_rate = rate;
    const float d_alpha =
        dt / (1.0f / (2.0f * 3.14159265358979323846f * DTERM_LPF_HZ) + dt);
    state->dterm += d_alpha * (derivative - state->dterm);
    return clampf(gains->kp * tpa_factor * error + state->integral +
                      gains->kd * tpa_factor * state->dterm +
                      feedforward * setpoint,
                  -limit, limit);
}

static void reset_controller(void)
{
    memset(&roll_state, 0, sizeof(roll_state));
    memset(&pitch_state, 0, sizeof(pitch_state));
    memset(&yaw_state, 0, sizeof(yaw_state));
    memset(gyro_filter, 0, sizeof(gyro_filter));
    mixer_saturated = false;
}

static void begin_calibration(bool for_arm)
{
    calibrated = false;
    armed = false;
    arm_calibration_active = for_arm;
    calibration_count = 0U;
    calibration_outliers = 0U;
    calibration_started_us = board_micros();
    calibration_failed = false;
    sum_x = sum_y = sum_z = 0.0f;
    calibration_reference_x = calibration_reference_y =
        calibration_reference_z = 0.0f;
    bias_x = bias_y = bias_z = 0.0f;
    reset_controller();
}

static void update_status_led(void)
{
    bool on;
    if (!calibrated && calibration_failed) {
        on = true;
    } else if (!calibrated) {
        on = ((board_micros() / 100000U) & 1U) == 0U;
    } else {
        on = ((board_micros() / 500000U) & 1U) == 0U;
    }
    /* PC13 LED on the MAMBA target is active low. */
    HAL_GPIO_WritePin(STATUS_LED_PORT, STATUS_LED_PIN,
                      on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void flight_control_init(void)
{
    flight_log_init();
    armed = false;
    arm_switch_was_low = false;
    begin_calibration(false);
}

void flight_control_set_gains(const pid_gains_t *roll,
                              const pid_gains_t *pitch,
                              const pid_gains_t *yaw)
{
    if (armed) {
        return;
    }
    roll_gains = *roll;
    pitch_gains = *pitch;
    yaw_gains = *yaw;
    reset_controller();
}

void flight_control_set_motor_direction_reversed(bool reversed)
{
    if (!armed) {
        motor_direction_reversed = reversed;
        reset_controller();
    }
}

void flight_control_set_rates(float roll_max_dps, float pitch_max_dps,
                              float yaw_max_dps, float expo)
{
    if (!armed) {
        roll_rate_dps = roll_max_dps;
        pitch_rate_dps = pitch_max_dps;
        yaw_rate_dps = yaw_max_dps;
        rate_expo = clampf(expo, 0.0f, 0.9f);
        reset_controller();
    }
}

void flight_control_set_feedforward(float roll_ff, float pitch_ff,
                                    float yaw_ff)
{
    if (!armed) {
        roll_feedforward = roll_ff;
        pitch_feedforward = pitch_ff;
        yaw_feedforward = yaw_ff;
        reset_controller();
    }
}

void flight_control_set_tpa(float attenuation, float breakpoint_percent)
{
    if (!armed) {
        tpa_attenuation = clampf(attenuation, 0.0f, 1.0f);
        tpa_breakpoint_percent =
            clampf(breakpoint_percent, 0.0f, 100.0f);
        reset_controller();
    }
}

void flight_control_set_motor_idle_percent(float percent)
{
    if (!armed) {
        motor_idle_percent = clampf(percent, 1.0f, 10.0f);
        reset_controller();
    }
}

void flight_control_update(const imu_sample_t *imu,
                           const sbus_data_t *rx,
                           float dt,
                           uint16_t motors[4])
{
    const flight_settings_t *settings = flight_settings_get();
    const bool aetr = settings->receiver_channel_order ==
                      RECEIVER_ORDER_AETR1234;
    const uint8_t throttle_ch = aetr ? 2U : 0U;
    const uint8_t roll_ch = aetr ? 0U : 1U;
    const uint8_t pitch_ch = aetr ? 1U : 2U;
    const uint8_t yaw_ch = 3U;
    const float measured_dt = dt;
    update_status_led();
    dt = clampf(dt, 0.00005f, 0.0005f);
    memset(motors, 0, sizeof(uint16_t) * 4U);
    const float throttle = clampf(
        ((float)rx->channel_us[throttle_ch] - 1000.0f) / 10.0f,
        0.0f, 100.0f);
    const bool arm_switch = rx->valid &&
        rx->channel_us[settings->arm_channel] >= settings->arm_min_us &&
        rx->channel_us[settings->arm_channel] <= settings->arm_max_us;
    if (!calibrated) {
        if (armed) flight_log_stop(FLIGHT_LOG_FLAG_STOP_DISARM);
        armed = false;
        if (arm_calibration_active &&
            (!rx->valid || !arm_switch ||
             throttle > ARM_THROTTLE_MAX_PERCENT)) {
            arm_calibration_active = false;
            calibration_count = 0U;
            calibration_outliers = 0U;
            sum_x = sum_y = sum_z = 0.0f;
        }
        if ((uint32_t)(board_micros() - calibration_started_us) >=
            CALIBRATION_TIMEOUT_US) {
            calibration_failed = true;
        }
        const bool may_calibrate =
            arm_calibration_active
                ? (rx->valid && arm_switch &&
                   throttle <= ARM_THROTTLE_MAX_PERCENT)
                : !arm_switch;
        if (may_calibrate) {
            if (calibration_count == 0U) {
                calibration_reference_x = imu->gyro_x_dps;
                calibration_reference_y = imu->gyro_y_dps;
                calibration_reference_z = imu->gyro_z_dps;
                sum_x += imu->gyro_x_dps;
                sum_y += imu->gyro_y_dps;
                sum_z += imu->gyro_z_dps;
                calibration_count = 1U;
            } else {
                const bool stationary =
                    fabsf(imu->gyro_x_dps) <
                        CALIBRATION_MAX_ABSOLUTE_RATE_DPS &&
                    fabsf(imu->gyro_y_dps) <
                        CALIBRATION_MAX_ABSOLUTE_RATE_DPS &&
                    fabsf(imu->gyro_z_dps) <
                        CALIBRATION_MAX_ABSOLUTE_RATE_DPS &&
                    fabsf(imu->gyro_x_dps - calibration_reference_x) <
                        CALIBRATION_MAX_VARIATION_DPS &&
                    fabsf(imu->gyro_y_dps - calibration_reference_y) <
                        CALIBRATION_MAX_VARIATION_DPS &&
                    fabsf(imu->gyro_z_dps - calibration_reference_z) <
                        CALIBRATION_MAX_VARIATION_DPS;
                if (!stationary) {
                    if (++calibration_outliers >=
                        CALIBRATION_MAX_CONSECUTIVE_OUTLIERS) {
                        calibration_count = 0U;
                        calibration_outliers = 0U;
                        sum_x = sum_y = sum_z = 0.0f;
                    }
                } else {
                    calibration_outliers = 0U;
                    sum_x += imu->gyro_x_dps;
                    sum_y += imu->gyro_y_dps;
                    sum_z += imu->gyro_z_dps;
                    ++calibration_count;
                }
                if (calibration_count >= CALIBRATION_SAMPLES) {
                    bias_x = sum_x / calibration_count;
                    bias_y = sum_y / calibration_count;
                    bias_z = sum_z / calibration_count;
                    calibrated = true;
                    calibration_failed = false;
                    if (arm_calibration_active) {
                        arm_calibration_active = false;
                        armed = true;
                        reset_controller();
                        flight_log_start();
                    }
                }
            }
        }
        return;
    }

    if (!rx->valid || !arm_switch) {
        if (!arm_switch) {
            arm_switch_was_low = true;
        }
        if (armed) {
            if (!rx->valid) {
                flight_log_stop_rx(
                    rx->loss_reason == SBUS_LOSS_FAILSAFE);
            } else {
                flight_log_stop(FLIGHT_LOG_FLAG_STOP_DISARM);
            }
        }
        armed = false;
        reset_controller();
        return;
    }
    if (!armed) {
        if (!arm_switch_was_low || throttle > ARM_THROTTLE_MAX_PERCENT) {
            return;
        }
        begin_calibration(true);
        return;
    }

    const float rate_roll = pt1(&gyro_filter[0], imu->gyro_x_dps - bias_x,
                                GYRO_LPF_HZ, dt);
    const float rate_pitch = pt1(&gyro_filter[1], imu->gyro_y_dps - bias_y,
                                 GYRO_LPF_HZ, dt);
    const float rate_yaw = pt1(&gyro_filter[2], imu->gyro_z_dps - bias_z,
                               GYRO_LPF_HZ, dt);
    const float setpoint_roll =
        rate_setpoint(rx->channel_us[roll_ch], roll_rate_dps);
    const float setpoint_pitch =
        rate_setpoint(rx->channel_us[pitch_ch], pitch_rate_dps);
    const float setpoint_yaw =
        rate_setpoint(rx->channel_us[yaw_ch], yaw_rate_dps);
    float tpa_factor = 1.0f;
    if (tpa_attenuation > 0.0f &&
        throttle > tpa_breakpoint_percent &&
        tpa_breakpoint_percent < 100.0f) {
        tpa_factor = 1.0f - tpa_attenuation *
            (throttle - tpa_breakpoint_percent) /
            (100.0f - tpa_breakpoint_percent);
    }
    const float roll = pid(&roll_state, &roll_gains,
                           setpoint_roll,
                           rate_roll, roll_feedforward, dt, 35.0f,
                           tpa_factor);
    const float pitch = pid(&pitch_state, &pitch_gains,
                            setpoint_pitch,
                            rate_pitch, pitch_feedforward, dt, 35.0f,
                            tpa_factor);
    const float yaw = pid(&yaw_state, &yaw_gains,
                          setpoint_yaw,
                          rate_yaw, yaw_feedforward, dt, 25.0f,
                          tpa_factor);
    const float mixer_yaw = motor_direction_reversed ? -yaw : yaw;

    float correction[4] = {
        -roll + pitch - mixer_yaw,
        -roll - pitch + mixer_yaw,
        roll + pitch + mixer_yaw,
        roll - pitch - mixer_yaw
    };
    float correction_min = correction[0];
    float correction_max = correction[0];
    for (uint8_t i = 1U; i < 4U; ++i) {
        correction_min = fminf(correction_min, correction[i]);
        correction_max = fmaxf(correction_max, correction[i]);
    }
    const float available = 100.0f - motor_idle_percent;
    const float span = correction_max - correction_min;
    const float scale = span > available ? available / span : 1.0f;
    correction_min *= scale;
    correction_max *= scale;
    const float requested_base =
        motor_idle_percent + throttle * available / 100.0f;
    const float base = clampf(requested_base,
                              motor_idle_percent - correction_min,
                              100.0f - correction_max);
    mixer_saturated = scale < 0.999f || base != requested_base;
    for (uint8_t i = 0; i < 4U; ++i) {
        motors[i] = dshot_from_percent(base + correction[i] * scale);
    }
    const float rates[3] = {rate_roll, rate_pitch, rate_yaw};
    const float setpoints[3] =
        {setpoint_roll, setpoint_pitch, setpoint_yaw};
    const float pid_output[3] = {roll, pitch, yaw};
    uint32_t loop_us = (uint32_t)(measured_dt * 1000000.0f + 0.5f);
    if (loop_us > 65535U) loop_us = 65535U;
    flight_log_record(rates, setpoints, pid_output, motors, throttle,
                      mixer_saturated, (uint16_t)loop_us);
}

bool flight_control_is_armed(void)
{
    return armed;
}

bool flight_control_is_calibrated(void)
{
    return calibrated;
}

uint16_t flight_control_get_calibration_samples(void)
{
    return calibration_count;
}

void flight_control_start_calibration(void)
{
    if (armed) flight_log_stop(FLIGHT_LOG_FLAG_STOP_DISARM);
    arm_switch_was_low = false;
    begin_calibration(false);
}

void flight_control_emergency_stop(uint8_t log_stop_flag)
{
    if (armed) flight_log_stop(log_stop_flag);
    armed = false;
    arm_switch_was_low = false;
    reset_controller();
}

void flight_control_reset_pid_state(void)
{
    reset_controller();
}

void flight_control_get_corrected_imu(const imu_sample_t *raw,
                                      imu_sample_t *corrected)
{
    *corrected = *raw;
    if (calibrated) {
        corrected->gyro_x_dps -= bias_x;
        corrected->gyro_y_dps -= bias_y;
        corrected->gyro_z_dps -= bias_z;
    }
}
