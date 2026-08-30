#include "msp_displayport.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "board.h"
#include "flight_settings.h"

#define MSP_BAUD_RATE 115200U
#define MSP_DISPLAYPORT 182U
#define MSP_DP_HEARTBEAT 0U
#define MSP_DP_RELEASE 1U
#define MSP_DP_CLEAR_SCREEN 2U
#define MSP_DP_WRITE_STRING 3U
#define MSP_DP_DRAW_SCREEN 4U
#define MSP_DP_OPTIONS 5U
#define MSP_DP_OPTION_HDZERO_CENTERED_30X16 2U
#define TX_BUFFER_SIZE 512U
#define SCREEN_COLUMNS 30U
#define SCREEN_ROWS 16U
#define TEXT_LENGTH 29U
#define HEARTBEAT_PERIOD_US 500000U

typedef enum {
    DISPLAYPORT_NOT_CONFIGURED,
    DISPLAYPORT_READY,
    DISPLAYPORT_UART_ERROR,
    DISPLAYPORT_QUEUE_FULL,
} displayport_status_t;

#if BOARD_HAS_DIGITAL_OSD
static UART_HandleTypeDef uart;
static uint8_t tx_buffer[TX_BUFFER_SIZE];
static uint16_t tx_head;
static uint16_t tx_tail;
static uint32_t last_heartbeat_us;
static uint32_t flight_started_us;
static uint32_t flight_duration_us;
static bool available;
static bool previous_armed;
static bool timer_started;
static bool release_sent;
static displayport_status_t status;

static uint16_t buffer_used(void)
{
    return (uint16_t)((tx_head - tx_tail) & (TX_BUFFER_SIZE - 1U));
}

static uint16_t buffer_free(void)
{
    return (uint16_t)(TX_BUFFER_SIZE - 1U - buffer_used());
}

static bool enqueue_byte(uint8_t value)
{
    const uint16_t next = (uint16_t)((tx_head + 1U) & (TX_BUFFER_SIZE - 1U));
    if (next == tx_tail) return false;
    tx_buffer[tx_head] = value;
    tx_head = next;
    return true;
}

static bool enqueue_msp(uint8_t command, const uint8_t *payload, uint8_t length)
{
    if (buffer_free() < (uint16_t)length + 6U) {
        status = DISPLAYPORT_QUEUE_FULL;
        return false;
    }
    uint8_t checksum = length ^ command;
    (void)enqueue_byte('$');
    (void)enqueue_byte('M');
    (void)enqueue_byte('>');
    (void)enqueue_byte(length);
    (void)enqueue_byte(command);
    for (uint8_t i = 0U; i < length; ++i) {
        (void)enqueue_byte(payload[i]);
        checksum ^= payload[i];
    }
    (void)enqueue_byte(checksum);
    if (status == DISPLAYPORT_QUEUE_FULL) status = DISPLAYPORT_READY;
    return true;
}

static bool enqueue_displayport(const uint8_t *payload, uint8_t length)
{
    return enqueue_msp(MSP_DISPLAYPORT, payload, length);
}

static void enqueue_simple(uint8_t subcommand)
{
    (void)enqueue_displayport(&subcommand, 1U);
}

static void enqueue_string(uint32_t position, const char *text)
{
    if (position >= SCREEN_COLUMNS * SCREEN_ROWS || text == NULL ||
        text[0] == '\0') return;
    uint8_t payload[TEXT_LENGTH + 4U];
    size_t length = strlen(text);
    if (length > TEXT_LENGTH) length = TEXT_LENGTH;
    const uint8_t column = (uint8_t)(position % SCREEN_COLUMNS);
    if (length > SCREEN_COLUMNS - column) length = SCREEN_COLUMNS - column;
    payload[0] = MSP_DP_WRITE_STRING;
    payload[1] = (uint8_t)(position / SCREEN_COLUMNS);
    payload[2] = column;
    payload[3] = 0U;
    memcpy(&payload[4], text, length);
    (void)enqueue_displayport(payload, (uint8_t)(length + 4U));
}

static void enqueue_layout(float voltage)
{
    const flight_settings_t *settings = flight_settings_get();
    char total_voltage[16] = "";
    char cell_voltage[16] = "";
    char timer[16];
    char vtx_channel[16];
    char vtx_power[16];
    if (voltage >= 1.0f && voltage < 100.0f) {
        uint8_t cells = (uint8_t)ceilf(voltage / 4.25f);
        if (cells == 0U) cells = 1U;
        (void)snprintf(total_voltage, sizeof(total_voltage), "BAT %.2fV",
                       (double)voltage);
        (void)snprintf(cell_voltage, sizeof(cell_voltage), "CELL %.2fV",
                       (double)(voltage / (float)cells));
    }
    const uint32_t seconds = flight_duration_us / 1000000U;
    (void)snprintf(timer, sizeof(timer), "%02lu:%02lu",
                   (unsigned long)(seconds / 60U),
                   (unsigned long)(seconds % 60U));
    static const char bands[] = "ABEFRL";
    (void)snprintf(vtx_channel, sizeof(vtx_channel), "VTX %c%lu",
                   bands[settings->vtx_band],
                   (unsigned long)(settings->vtx_channel + 1U));
    (void)snprintf(vtx_power, sizeof(vtx_power), "VTX %luMW",
                   (unsigned long)settings->vtx_power_mw);

    const char *elements[OSD_ELEMENT_COUNT] = {
        total_voltage, cell_voltage, timer, "FLIGHTCODE",
        settings->osd_pilot_name,
    };
    enqueue_simple(MSP_DP_CLEAR_SCREEN);
    for (uint8_t i = 0U; i < OSD_ELEMENT_COUNT; ++i) {
        if ((settings->osd_element_enabled_mask & (1U << i)) != 0U)
            enqueue_string(settings->osd_element_positions[i], elements[i]);
    }
    if ((settings->vtx_osd_enabled_mask & 1U) != 0U)
        enqueue_string(settings->vtx_osd_positions[0], vtx_channel);
    if ((settings->vtx_osd_enabled_mask & 2U) != 0U)
        enqueue_string(settings->vtx_osd_positions[1], vtx_power);
    enqueue_simple(MSP_DP_DRAW_SCREEN);
}
#endif

bool msp_displayport_init(void)
{
#if BOARD_HAS_DIGITAL_OSD
    const flight_settings_t *settings = flight_settings_get();
    available = false;
    tx_head = tx_tail = 0U;
    status = DISPLAYPORT_NOT_CONFIGURED;
    if (settings->vtx_protocol != VTX_PROTOCOL_HDZERO_MSP) return false;
    if (!board_uart_tx_init((uint8_t)settings->vtx_uart, MSP_BAUD_RATE, &uart)) {
        status = DISPLAYPORT_UART_ERROR;
        return false;
    }
    available = true;
    status = DISPLAYPORT_READY;
    release_sent = false;
    previous_armed = false;
    timer_started = false;
    last_heartbeat_us = 0U;
    const uint8_t options[] = {
        MSP_DP_OPTIONS, MSP_DP_OPTION_HDZERO_CENTERED_30X16,
    };
    (void)enqueue_displayport(options, sizeof(options));
    enqueue_simple(MSP_DP_HEARTBEAT);
    return true;
#else
    return false;
#endif
}

void msp_displayport_process(void)
{
#if BOARD_HAS_DIGITAL_OSD
    if (!available || tx_tail == tx_head ||
        __HAL_UART_GET_FLAG(&uart, UART_FLAG_TXE) == RESET) return;
#if defined(PLATFORM_STM32H7)
    uart.Instance->TDR = tx_buffer[tx_tail];
#else
    uart.Instance->DR = tx_buffer[tx_tail];
#endif
    tx_tail = (uint16_t)((tx_tail + 1U) & (TX_BUFFER_SIZE - 1U));
#endif
}

void msp_displayport_update(float voltage, bool armed, uint32_t now_us)
{
#if BOARD_HAS_DIGITAL_OSD
    if (!available) return;
    const flight_settings_t *settings = flight_settings_get();
    if ((uint32_t)(now_us - last_heartbeat_us) >= HEARTBEAT_PERIOD_US) {
        enqueue_simple(MSP_DP_HEARTBEAT);
        last_heartbeat_us = now_us;
    }
    if (settings->osd_enabled == 0U) {
        if (!release_sent) {
            enqueue_simple(MSP_DP_RELEASE);
            release_sent = true;
        }
        return;
    }
    release_sent = false;
    if (armed && !previous_armed) {
        flight_started_us = now_us;
        flight_duration_us = 0U;
        timer_started = true;
    } else if (armed && timer_started) {
        flight_duration_us = now_us - flight_started_us;
    }
    previous_armed = armed;
    enqueue_layout(voltage);
#else
    (void)voltage; (void)armed; (void)now_us;
#endif
}

bool msp_displayport_is_available(void)
{
#if BOARD_HAS_DIGITAL_OSD
    return available;
#else
    return false;
#endif
}

bool msp_displayport_is_enabled(void)
{
    return msp_displayport_is_available() &&
        flight_settings_get()->osd_enabled != 0U;
}

const char *msp_displayport_status_name(void)
{
#if BOARD_HAS_DIGITAL_OSD
    switch (status) {
    case DISPLAYPORT_READY: return "MSP_DISPLAYPORT_READY";
    case DISPLAYPORT_UART_ERROR: return "MSP_DISPLAYPORT_UART_ERROR";
    case DISPLAYPORT_QUEUE_FULL: return "MSP_DISPLAYPORT_QUEUE_FULL";
    default: return "MSP_DISPLAYPORT_NOT_CONFIGURED";
    }
#else
    return "MSP_DISPLAYPORT_UNSUPPORTED";
#endif
}

