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
#define FLIGHT_PI_F 3.14159265358979323846f
#define YAW_ITERM_RELAX_LPF_HZ 8.0f
#define YAW_ITERM_RELAX_THRESHOLD_DPS 20.0f
#define AIRMODE_ACTIVATION_THROTTLE_PERCENT 10.0f
#define ARM_THROTTLE_MAX_PERCENT 5.0f
#define FEEDFORWARD_CENTER_SCALE 0.70f

typedef struct {
    float integral;
    float previous_rate;
    float dterm;
    float relaxed_setpoint;
} pid_state_t;

typedef struct {
    float value;
    bool initialized;
} pt1_filter_t;

static pid_gains_t roll_gains = {0.1005f, 0.200f, 0.0009f};
static pid_gains_t pitch_gains = {0.1005f, 0.200f, 0.0007f};
static pid_gains_t yaw_gains = {0.155f, 0.250f, 0.0f};
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
static float motor_idle_percent = 5.0f;
static float roll_rate_dps = 420.0f;
static float pitch_rate_dps = 420.0f;
static float yaw_rate_dps = 350.0f;
static float rate_expo = 0.30f;
static float roll_feedforward = 0.025f;
static float pitch_feedforward = 0.025f;
static float yaw_feedforward = 0.015f;
static float tpa_attenuation = 0.20f;
static float tpa_breakpoint_percent = 70.0f;
static bool mixer_saturated;
static bool airmode_active;
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
    const float rc = 1.0f / (2.0f * FLIGHT_PI_F * cutoff_hz);
    filter->value += (dt / (rc + dt)) * (input - filter->value);
    return filter->value;
}

static float progressive_feedforward(float gain, float setpoint,
                                     float max_rate_dps)
{
    const float normalized = clampf(fabsf(setpoint) / max_rate_dps,
                                    0.0f, 1.0f);
    const float smoothstep = normalized * normalized *
                             (3.0f - 2.0f * normalized);
    const float scale = FEEDFORWARD_CENTER_SCALE +
        (1.0f - FEEDFORWARD_CENTER_SCALE) * smoothstep;
    return gain * setpoint * scale;
}

static float pid(pid_state_t *state, const pid_gains_t *gains,
                  float setpoint, float rate, float feedforward,
                  float max_rate_dps, float dt, float limit, float tpa_factor,
                 float dterm_lpf_hz, bool integral_enabled,
                  bool relax_integral,
                  float *p_out, float *i_out,
                  float *d_out, float *ff_out)
{
    const float error = setpoint - rate;
    float integral_factor = 1.0f;
    if (relax_integral) {
        const float relax_rc =
            1.0f / (2.0f * FLIGHT_PI_F *
                    YAW_ITERM_RELAX_LPF_HZ);
        state->relaxed_setpoint +=
            (dt / (relax_rc + dt)) *
            (setpoint - state->relaxed_setpoint);
        const float setpoint_highpass =
            fabsf(setpoint - state->relaxed_setpoint);
        integral_factor = clampf(
            1.0f - setpoint_highpass / YAW_ITERM_RELAX_THRESHOLD_DPS,
            0.0f, 1.0f);
    }
    if (!integral_enabled) {
        /* Do not store corrections while armed on the ground. */
        state->integral = 0.0f;
    } else if (!mixer_saturated || state->integral * error < 0.0f) {
        /* Never slow down unwinding an I term that opposes the error. */
        if (state->integral * error < 0.0f) integral_factor = 1.0f;
        state->integral =
            clampf(state->integral +
                       gains->ki * error * dt * integral_factor,
                   -30.0f, 30.0f);
    }
    const float derivative = -(rate - state->previous_rate) / dt;
    state->previous_rate = rate;
    const float d_alpha =
        dt / (1.0f / (2.0f * FLIGHT_PI_F * dterm_lpf_hz) + dt);
    state->dterm += d_alpha * (derivative - state->dterm);
    *p_out = gains->kp * tpa_factor * error;
    *i_out = state->integral;
    *d_out = gains->kd * tpa_factor * state->dterm;
    *ff_out = progressive_feedforward(feedforward, setpoint,
                                      max_rate_dps);
    return clampf(*p_out + *i_out + *d_out + *ff_out,
                  -limit, limit);
}

static void reset_controller(void)
{
    memset(&roll_state, 0, sizeof(roll_state));
    memset(&pitch_state, 0, sizeof(pitch_state));
    memset(&yaw_state, 0, sizeof(yaw_state));
    memset(gyro_filter, 0, sizeof(gyro_filter));
    mixer_saturated = false;
    airmode_active = false;
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
                           uint16_t main_loop_us,
                           uint16_t gyro_loop_us,
                           uint16_t motors[4])
{
    const flight_settings_t *settings = flight_settings_get();
    const bool aetr = settings->receiver_channel_order ==
                      RECEIVER_ORDER_AETR1234;
    const uint8_t throttle_ch = aetr ? 2U : 0U;
    const uint8_t roll_ch = aetr ? 0U : 1U;
    const uint8_t pitch_ch = aetr ? 1U : 2U;
    const uint8_t yaw_ch = 3U;
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
                                settings->gyro_lpf_hz, dt);
    const float rate_pitch = pt1(&gyro_filter[1], imu->gyro_y_dps - bias_y,
                                 settings->gyro_lpf_hz, dt);
    const float rate_yaw = pt1(&gyro_filter[2], imu->gyro_z_dps - bias_z,
                               settings->gyro_lpf_hz, dt);
    const float setpoint_roll =
        rate_setpoint(rx->channel_us[roll_ch], roll_rate_dps);
    /* Receiver pitch high is nose-down; body pitch positive is nose-up. */
    const float setpoint_pitch =
        -rate_setpoint(rx->channel_us[pitch_ch], pitch_rate_dps);
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
    /*
     * Airmode is latched for the rest of the armed session.  Before its
     * first activation the craft may be handled or the sticks moved while
     * the motors have little authority, so the I term must remain empty.
     * Activate before evaluating the PID loops so the first airborne
     * correction always starts from a known zero integral.
     */
    if (!airmode_active &&
        throttle >= AIRMODE_ACTIVATION_THROTTLE_PERCENT) {
        airmode_active = true;
    }
    float p_term[3], i_term[3], d_term[3], ff_term[3];
    const float roll = pid(&roll_state, &roll_gains,
                           setpoint_roll,
                           rate_roll, roll_feedforward, roll_rate_dps,
                           dt, 35.0f,
                           tpa_factor, settings->dterm_lpf_hz,
                            airmode_active, false,
                            &p_term[0], &i_term[0],
                            &d_term[0], &ff_term[0]);
    const float pitch = pid(&pitch_state, &pitch_gains,
                            setpoint_pitch,
                            rate_pitch, pitch_feedforward, pitch_rate_dps,
                            dt, 35.0f,
                            tpa_factor, settings->dterm_lpf_hz,
                             airmode_active, false,
                             &p_term[1], &i_term[1],
                             &d_term[1], &ff_term[1]);
    const float yaw = pid(&yaw_state, &yaw_gains,
                          setpoint_yaw,
                          rate_yaw, yaw_feedforward, yaw_rate_dps,
                          dt, 25.0f,
                          tpa_factor, settings->dterm_lpf_hz,
                          airmode_active, true,
                          &p_term[2], &i_term[2],
                          &d_term[2], &ff_term[2]);
    const float mixer_yaw = motor_direction_reversed ? -yaw : yaw;
    const float pid_authority = airmode_active ? 1.0f :
        clampf(throttle / AIRMODE_ACTIVATION_THROTTLE_PERCENT, 0.0f, 1.0f);

    /* Quad X: M1 rear-right, M2 front-right, M3 rear-left, M4 front-left. */
    float correction[4] = {
        (-roll - pitch - mixer_yaw) * pid_authority,
        (-roll + pitch + mixer_yaw) * pid_authority,
        (roll - pitch + mixer_yaw) * pid_authority,
        (roll + pitch - mixer_yaw) * pid_authority
    };
    float correction_min = correction[0];
    float correction_max = correction[0];
    for (uint8_t i = 1U; i < 4U; ++i) {
        correction_min = fminf(correction_min, correction[i]);
        correction_max = fmaxf(correction_max, correction[i]);
    }
    const float available = 100.0f - motor_idle_percent;
    const float requested_base =
        motor_idle_percent + throttle * available / 100.0f;
    float scale = 1.0f;
    float base = requested_base;
    if (airmode_active) {
        const float span = correction_max - correction_min;
        if (span > available) scale = available / span;
        correction_min *= scale;
        correction_max *= scale;
        base = clampf(requested_base,
                      motor_idle_percent - correction_min,
                      100.0f - correction_max);
    } else {
        /*
         * Before takeoff the throttle command owns the collective output.
         * Keep its base fixed and reduce all axis corrections by the same
         * factor so none of them can pull a motor below idle or raise the
         * collective through airmode-style base shifting.
         */
        if (correction_min < 0.0f) {
            scale = fminf(scale,
                          (requested_base - motor_idle_percent) /
                              -correction_min);
        }
        if (correction_max > 0.0f) {
            scale = fminf(scale,
                          (100.0f - requested_base) / correction_max);
        }
        scale = clampf(scale, 0.0f, 1.0f);
    }
    /* Scaled corrections have exhausted the authority available in the
     * current mixer mode and should stop I-term accumulation. */
    mixer_saturated = scale < 0.999f;
    for (uint8_t i = 0; i < 4U; ++i) {
        motors[i] = dshot_from_percent(base + correction[i] * scale);
    }
    const float rates[3] = {rate_roll, rate_pitch, rate_yaw};
    const float setpoints[3] =
        {setpoint_roll, setpoint_pitch, setpoint_yaw};
    flight_log_record(rates, setpoints, p_term, i_term, d_term,
                      ff_term, motors, throttle,
                      mixer_saturated, main_loop_us, gyro_loop_us);
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
    /* The guided PID test primes airmode before resetting each test axis. */
    const bool preserve_airmode = armed && airmode_active;
    reset_controller();
    airmode_active = preserve_airmode;
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
