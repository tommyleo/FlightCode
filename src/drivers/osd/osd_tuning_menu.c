#include "osd_tuning_menu.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "board.h"
#include "flight_settings.h"
#include "max7456.h"
#include "vtx_tramp.h"

#define MENU_ENTRY_HOLD_US 800000U
#define MENU_ACTION_REPEAT_US 250000U
#define STICK_LOW_US 1250U
#define STICK_HIGH_US 1750U
#define STICK_CENTER_LOW_US 1350U
#define STICK_CENTER_HIGH_US 1650U
#define MENU_ITEM_COUNT 21U
#define PID_GAIN_STEP 0.00001f
#define PID_GAIN_SCALE 100000.0f

typedef enum {
    ITEM_ROLL_P,
    ITEM_ROLL_I,
    ITEM_ROLL_D,
    ITEM_ROLL_FF,
    ITEM_PITCH_P,
    ITEM_PITCH_I,
    ITEM_PITCH_D,
    ITEM_PITCH_FF,
    ITEM_YAW_P,
    ITEM_YAW_I,
    ITEM_YAW_D,
    ITEM_YAW_FF,
    ITEM_ROLL_RATE,
    ITEM_PITCH_RATE,
    ITEM_YAW_RATE,
    ITEM_EXPO,
    ITEM_TPA,
    ITEM_TPA_BREAKPOINT,
    ITEM_VTX_BAND,
    ITEM_VTX_CHANNEL,
    ITEM_VTX_POWER,
} menu_item_t;

static bool active;
static bool controls_ready;
static uint8_t selected_item;
static uint32_t entry_started_us;
static uint32_t last_action_us;
typedef struct {
    pid_gains_t roll, pitch, yaw;
    float roll_feedforward, pitch_feedforward, yaw_feedforward;
    float roll_rate_dps, pitch_rate_dps, yaw_rate_dps;
    float rate_expo, tpa_attenuation, tpa_breakpoint_percent;
    uint32_t vtx_band, vtx_channel, vtx_power_mw;
} menu_settings_t;
static menu_settings_t edited_settings;

static void load_menu_settings(const flight_settings_t *s)
{
    edited_settings = (menu_settings_t){
        .roll=s->roll,.pitch=s->pitch,.yaw=s->yaw,
        .roll_feedforward=s->roll_feedforward,
        .pitch_feedforward=s->pitch_feedforward,
        .yaw_feedforward=s->yaw_feedforward,
        .roll_rate_dps=s->roll_rate_dps,.pitch_rate_dps=s->pitch_rate_dps,
        .yaw_rate_dps=s->yaw_rate_dps,.rate_expo=s->rate_expo,
        .tpa_attenuation=s->tpa_attenuation,
        .tpa_breakpoint_percent=s->tpa_breakpoint_percent,
        .vtx_band=s->vtx_band,.vtx_channel=s->vtx_channel,
        .vtx_power_mw=s->vtx_power_mw,
    };
}

static float clampf(float value, float minimum, float maximum)
{
    return value < minimum ? minimum :
           (value > maximum ? maximum : value);
}

static float change_pid_gain(float value, int8_t direction, float maximum)
{
    const float steps = roundf(value * PID_GAIN_SCALE) +
        (direction > 0 ? 1.0f : -1.0f);
    return clampf(steps * PID_GAIN_STEP, 0.0f, maximum);
}

static void primary_channels(const flight_settings_t *settings,
                             uint8_t *throttle, uint8_t *roll,
                             uint8_t *pitch, uint8_t *yaw)
{
    const bool aetr = settings->receiver_channel_order ==
                      RECEIVER_ORDER_AETR1234;
    *throttle = aetr ? 2U : 0U;
    *roll = aetr ? 0U : 1U;
    *pitch = aetr ? 1U : 2U;
    *yaw = 3U;
}

static void format_item(char label[20], char value[20])
{
    static const char *const labels[MENU_ITEM_COUNT] = {
        "ROLL P", "ROLL I", "ROLL D", "ROLL FF",
        "PITCH P", "PITCH I", "PITCH D", "PITCH FF",
        "YAW P", "YAW I", "YAW D", "YAW FF",
        "ROLL RATE", "PITCH RATE", "YAW RATE",
        "EXPO", "TPA", "TPA BREAK", "VTX BAND", "VTX CHANNEL", "VTX POWER",
    };
    (void)snprintf(label, 20U, "%s", labels[selected_item]);
    switch ((menu_item_t)selected_item) {
    case ITEM_ROLL_P: (void)snprintf(value, 20U, "%.5f", edited_settings.roll.kp); break;
    case ITEM_ROLL_I: (void)snprintf(value, 20U, "%.5f", edited_settings.roll.ki); break;
    case ITEM_ROLL_D: (void)snprintf(value, 20U, "%.5f", edited_settings.roll.kd); break;
    case ITEM_ROLL_FF: (void)snprintf(value, 20U, "%.3f", edited_settings.roll_feedforward); break;
    case ITEM_PITCH_P: (void)snprintf(value, 20U, "%.5f", edited_settings.pitch.kp); break;
    case ITEM_PITCH_I: (void)snprintf(value, 20U, "%.5f", edited_settings.pitch.ki); break;
    case ITEM_PITCH_D: (void)snprintf(value, 20U, "%.5f", edited_settings.pitch.kd); break;
    case ITEM_PITCH_FF: (void)snprintf(value, 20U, "%.3f", edited_settings.pitch_feedforward); break;
    case ITEM_YAW_P: (void)snprintf(value, 20U, "%.5f", edited_settings.yaw.kp); break;
    case ITEM_YAW_I: (void)snprintf(value, 20U, "%.5f", edited_settings.yaw.ki); break;
    case ITEM_YAW_D: (void)snprintf(value, 20U, "%.5f", edited_settings.yaw.kd); break;
    case ITEM_YAW_FF: (void)snprintf(value, 20U, "%.3f", edited_settings.yaw_feedforward); break;
    case ITEM_ROLL_RATE: (void)snprintf(value, 20U, "%.0f DPS", edited_settings.roll_rate_dps); break;
    case ITEM_PITCH_RATE: (void)snprintf(value, 20U, "%.0f DPS", edited_settings.pitch_rate_dps); break;
    case ITEM_YAW_RATE: (void)snprintf(value, 20U, "%.0f DPS", edited_settings.yaw_rate_dps); break;
    case ITEM_EXPO: (void)snprintf(value, 20U, "%.2f", edited_settings.rate_expo); break;
    case ITEM_TPA: (void)snprintf(value, 20U, "%.0f %%", edited_settings.tpa_attenuation * 100.0f); break;
    case ITEM_TPA_BREAKPOINT: (void)snprintf(value, 20U, "%.0f %%", edited_settings.tpa_breakpoint_percent); break;
    case ITEM_VTX_BAND: (void)snprintf(value, 20U, "%c", "ABEFRL"[edited_settings.vtx_band]); break;
    case ITEM_VTX_CHANNEL: (void)snprintf(value, 20U, "CH %lu", (unsigned long)(edited_settings.vtx_channel + 1U)); break;
    case ITEM_VTX_POWER: (void)snprintf(value, 20U, "%lu MW", (unsigned long)edited_settings.vtx_power_mw); break;
    default: value[0] = '\0'; break;
    }
}

static void render(void)
{
    char label[20];
    char value[20];
    char counter[16];
    format_item(label, value);
    (void)snprintf(counter, sizeof(counter), "%02u/%02u",
                   (unsigned int)selected_item + 1U, MENU_ITEM_COUNT);
    max7456_clear_screen();
    max7456_write_text(1U, 9U, "PID TUNING");
    max7456_write_text(3U, 12U, counter);
    max7456_write_text(5U, 9U, label);
    max7456_write_text(7U, 8U, "< ");
    max7456_write_text(7U, 10U, value);
    max7456_write_text(7U, 22U, " >");
    max7456_write_text(10U, 2U, "PITCH SELECT  ROLL CHANGE");
    max7456_write_text(12U, 3U, "YAW LEFT EXIT RIGHT SAVE");
}

static void change_value(int8_t direction)
{
    const float sign = direction > 0 ? 1.0f : -1.0f;
    switch ((menu_item_t)selected_item) {
    case ITEM_ROLL_P: edited_settings.roll.kp = change_pid_gain(edited_settings.roll.kp, direction, 2.0f); break;
    case ITEM_ROLL_I: edited_settings.roll.ki = change_pid_gain(edited_settings.roll.ki, direction, 2.0f); break;
    case ITEM_ROLL_D: edited_settings.roll.kd = change_pid_gain(edited_settings.roll.kd, direction, 0.05f); break;
    case ITEM_ROLL_FF: edited_settings.roll_feedforward = clampf(edited_settings.roll_feedforward + sign * 0.001f, 0.0f, 1.0f); break;
    case ITEM_PITCH_P: edited_settings.pitch.kp = change_pid_gain(edited_settings.pitch.kp, direction, 2.0f); break;
    case ITEM_PITCH_I: edited_settings.pitch.ki = change_pid_gain(edited_settings.pitch.ki, direction, 2.0f); break;
    case ITEM_PITCH_D: edited_settings.pitch.kd = change_pid_gain(edited_settings.pitch.kd, direction, 0.05f); break;
    case ITEM_PITCH_FF: edited_settings.pitch_feedforward = clampf(edited_settings.pitch_feedforward + sign * 0.001f, 0.0f, 1.0f); break;
    case ITEM_YAW_P: edited_settings.yaw.kp = change_pid_gain(edited_settings.yaw.kp, direction, 2.0f); break;
    case ITEM_YAW_I: edited_settings.yaw.ki = change_pid_gain(edited_settings.yaw.ki, direction, 2.0f); break;
    case ITEM_YAW_D: edited_settings.yaw.kd = change_pid_gain(edited_settings.yaw.kd, direction, 0.05f); break;
    case ITEM_YAW_FF: edited_settings.yaw_feedforward = clampf(edited_settings.yaw_feedforward + sign * 0.001f, 0.0f, 1.0f); break;
    case ITEM_ROLL_RATE: edited_settings.roll_rate_dps = clampf(edited_settings.roll_rate_dps + sign, 100.0f, 1200.0f); break;
    case ITEM_PITCH_RATE: edited_settings.pitch_rate_dps = clampf(edited_settings.pitch_rate_dps + sign, 100.0f, 1200.0f); break;
    case ITEM_YAW_RATE: edited_settings.yaw_rate_dps = clampf(edited_settings.yaw_rate_dps + sign, 100.0f, 1200.0f); break;
    case ITEM_EXPO: edited_settings.rate_expo = clampf(edited_settings.rate_expo + sign * 0.01f, 0.0f, 0.9f); break;
    case ITEM_TPA: edited_settings.tpa_attenuation = clampf(edited_settings.tpa_attenuation + sign * 0.01f, 0.0f, 1.0f); break;
    case ITEM_TPA_BREAKPOINT: edited_settings.tpa_breakpoint_percent = clampf(edited_settings.tpa_breakpoint_percent + sign, 0.0f, 100.0f); break;
    case ITEM_VTX_BAND: edited_settings.vtx_band = direction > 0 ? (edited_settings.vtx_band + 1U) % 6U : (edited_settings.vtx_band + 5U) % 6U; break;
    case ITEM_VTX_CHANNEL: edited_settings.vtx_channel = direction > 0 ? (edited_settings.vtx_channel + 1U) % 8U : (edited_settings.vtx_channel + 7U) % 8U; break;
    case ITEM_VTX_POWER: {
        static const uint16_t powers[] = {25U, 100U, 200U, 400U, 500U, 600U, 1000U};
        uint8_t index = 0U;
        while (index + 1U < sizeof(powers) / sizeof(powers[0]) && powers[index] < edited_settings.vtx_power_mw) ++index;
        index = direction > 0 ? (uint8_t)((index + 1U) % (sizeof(powers) / sizeof(powers[0]))) :
            (uint8_t)((index + sizeof(powers) / sizeof(powers[0]) - 1U) % (sizeof(powers) / sizeof(powers[0])));
        edited_settings.vtx_power_mw = powers[index];
        break;
    }
    default: break;
    }
}

static void close_menu(bool save)
{
    if (save) {
        flight_settings_t settings = *flight_settings_get();
        const bool vtx_changed = settings.vtx_band != edited_settings.vtx_band ||
            settings.vtx_channel != edited_settings.vtx_channel ||
            settings.vtx_power_mw != edited_settings.vtx_power_mw;
        settings.roll=edited_settings.roll;settings.pitch=edited_settings.pitch;settings.yaw=edited_settings.yaw;
        settings.roll_feedforward=edited_settings.roll_feedforward;settings.pitch_feedforward=edited_settings.pitch_feedforward;settings.yaw_feedforward=edited_settings.yaw_feedforward;
        settings.roll_rate_dps=edited_settings.roll_rate_dps;settings.pitch_rate_dps=edited_settings.pitch_rate_dps;settings.yaw_rate_dps=edited_settings.yaw_rate_dps;
        settings.rate_expo=edited_settings.rate_expo;settings.tpa_attenuation=edited_settings.tpa_attenuation;settings.tpa_breakpoint_percent=edited_settings.tpa_breakpoint_percent;
        settings.vtx_band=edited_settings.vtx_band;settings.vtx_channel=edited_settings.vtx_channel;settings.vtx_power_mw=edited_settings.vtx_power_mw;
        if (flight_settings_set(&settings) && flight_settings_save() &&
            vtx_changed) {
            (void)vtx_tramp_init();
        }
    }
    active = false;
    controls_ready = false;
    entry_started_us = 0U;
    max7456_menu_end();
}

void osd_tuning_menu_init(void)
{
    active = false;
    controls_ready = false;
    selected_item = 0U;
    entry_started_us = 0U;
    last_action_us = 0U;
}

bool osd_tuning_menu_is_active(void)
{
    return active;
}

void osd_tuning_menu_update(const sbus_data_t *receiver, bool armed)
{
    if (receiver == NULL || !receiver->valid) {
        entry_started_us = 0U;
        if (active) close_menu(false);
        return;
    }
    const flight_settings_t *settings = flight_settings_get();
    uint8_t throttle_ch, roll_ch, pitch_ch, yaw_ch;
    primary_channels(settings, &throttle_ch, &roll_ch, &pitch_ch, &yaw_ch);
    const uint16_t throttle = receiver->channel_us[throttle_ch];
    const uint16_t roll = receiver->channel_us[roll_ch];
    const uint16_t pitch = receiver->channel_us[pitch_ch];
    const uint16_t yaw = receiver->channel_us[yaw_ch];
    const bool arm_switch =
        receiver->channel_us[settings->arm_channel] >= settings->arm_min_us &&
        receiver->channel_us[settings->arm_channel] <= settings->arm_max_us;
    const uint32_t now = board_micros();

    if (!active) {
        const bool entry_combo = !armed && !arm_switch &&
            throttle >= STICK_HIGH_US && roll >= STICK_HIGH_US;
        if (!entry_combo) {
            entry_started_us = 0U;
            return;
        }
        if (entry_started_us == 0U) entry_started_us = now;
        if ((uint32_t)(now - entry_started_us) < MENU_ENTRY_HOLD_US) return;
        if (!max7456_menu_begin()) return;
        load_menu_settings(settings);
        active = true;
        controls_ready = false;
        selected_item = 0U;
        last_action_us = now;
        render();
        return;
    }

    const bool centered = throttle >= STICK_CENTER_LOW_US &&
        throttle <= STICK_CENTER_HIGH_US &&
        roll >= STICK_CENTER_LOW_US && roll <= STICK_CENTER_HIGH_US &&
        pitch >= STICK_CENTER_LOW_US && pitch <= STICK_CENTER_HIGH_US &&
        yaw >= STICK_CENTER_LOW_US && yaw <= STICK_CENTER_HIGH_US;
    if (!controls_ready) {
        if (centered) controls_ready = true;
        return;
    }
    if ((uint32_t)(now - last_action_us) < MENU_ACTION_REPEAT_US) return;

    bool changed = false;
    if (yaw >= STICK_HIGH_US) {
        close_menu(true);
        return;
    }
    if (yaw <= STICK_LOW_US) {
        close_menu(false);
        return;
    }
    if (pitch >= STICK_HIGH_US) {
        selected_item = selected_item == 0U ?
            MENU_ITEM_COUNT - 1U : selected_item - 1U;
        changed = true;
    } else if (pitch <= STICK_LOW_US) {
        selected_item = (uint8_t)((selected_item + 1U) % MENU_ITEM_COUNT);
        changed = true;
    } else if (roll >= STICK_HIGH_US) {
        change_value(1);
        changed = true;
    } else if (roll <= STICK_LOW_US) {
        change_value(-1);
        changed = true;
    }
    if (changed) {
        last_action_us = now;
        render();
    }
}
