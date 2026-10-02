#include "vtx_smartaudio.h"
#include <math.h>
#include <string.h>
#include "board.h"
#include "flight_settings.h"

/* TBS SmartAudio rev09: 4800 baud, 8N2, single-wire half duplex.
 * Commands include the sync bytes in CRC; responses exclude them. */
#define BUFFER_SIZE 32U
#define MAX_ATTEMPTS 20U
typedef enum { IDLE, START_DELAY, TRANSMITTING, WAIT_RESPONSE,
               QUERY_DELAY, DONE, FAILED } sa_state_t;
static UART_HandleTypeDef uart;
static sa_state_t state;
static const char *status = "NOT_CONFIGURED";
static uint8_t tx[8], tx_size, tx_position;
static uint8_t rx[BUFFER_SIZE], rx_position, attempts, changes, command;
static uint8_t desired_channel;
static uint16_t desired_frequency, desired_power;
static uint32_t deadline;
static const uint16_t frequencies[5][8] = {
    {5865,5845,5825,5805,5785,5765,5745,5725},
    {5733,5752,5771,5790,5809,5828,5847,5866},
    {5705,5685,5665,5645,5885,5905,5925,5945},
    {5740,5760,5780,5800,5820,5840,5860,5880},
    {5658,5695,5732,5769,5806,5843,5880,5917}
};
static const uint16_t allowed_eu[5][8] = {
    {5865,5845,5825,5805,5785,5765,5745,0},
    {5733,5752,5771,5790,5809,5828,5847,5866},
    {0,0,0,0,0,0,0,0}, {5740,5760,5780,5800,5820,5840,5860,0},
    {0,0,5732,5769,5806,5843,0,0}
};

static uint8_t crc8(const uint8_t *bytes, uint8_t size)
{
    uint8_t crc = 0U;
    for (uint8_t i = 0U; i < size; ++i) {
        crc ^= bytes[i];
        for (uint8_t bit = 0U; bit < 8U; ++bit)
            crc = (uint8_t)((crc << 1) ^ ((crc & 0x80U) ? 0xD5U : 0U));
    }
    return crc;
}
static void fail(const char *reason) { status = reason; state = FAILED; }
const char *vtx_smartaudio_status_name(void) { return status; }

static void send_command(uint8_t code, const uint8_t *payload, uint8_t size)
{
    /* Dummy byte establishes the low level required by the Unify UART. */
    tx[0] = 0U; tx[1] = 0xAAU; tx[2] = 0x55U;
    tx[3] = (uint8_t)((code << 1) | 1U); tx[4] = size;
    if (size != 0U) memcpy(&tx[5], payload, size);
    tx[5U + size] = crc8(&tx[1], (uint8_t)(4U + size));
    tx_size = (uint8_t)(6U + size); tx_position = 0U;
    rx_position = 0U; command = code;
    __HAL_UART_CLEAR_OREFLAG(&uart);
    __HAL_UART_CLEAR_FEFLAG(&uart);
    __HAL_UART_CLEAR_NEFLAG(&uart);
    __HAL_UART_FLUSH_DRREGISTER(&uart);
    if (HAL_HalfDuplex_EnableTransmitter(&uart) != HAL_OK) {
        fail("UART_ERROR"); return;
    }
    state = TRANSMITTING; deadline = HAL_GetTick() + 100U;
}
static void query(void) { send_command(1U, NULL, 0U); }
static void schedule_query(void)
{ state = QUERY_DELAY; deadline = HAL_GetTick() + 200U; }

bool vtx_smartaudio_init(void)
{
    const flight_settings_t *s = flight_settings_get();
    state = IDLE; status = "NOT_CONFIGURED";
    if (s->vtx_protocol != VTX_PROTOCOL_SMARTAUDIO) return false;
    if (s->vtx_band >= 5U || s->vtx_channel >= 8U ||
        s->vtx_power_mw == 0U || s->vtx_power_mw > 2000U ||
        (s->vtx_region == VTX_REGION_EU && !allowed_eu[s->vtx_band][s->vtx_channel]) ||
        (s->vtx_region == VTX_REGION_US &&
         ((s->vtx_band == 2U && (s->vtx_channel == 3U || s->vtx_channel >= 6U))))) {
        fail("INVALID_SETTINGS"); return false;
    }
    desired_channel = (uint8_t)(s->vtx_band * 8U + s->vtx_channel);
    desired_frequency = frequencies[s->vtx_band][s->vtx_channel];
    desired_power = (uint16_t)s->vtx_power_mw;
    if (!board_uart_half_duplex_init((uint8_t)s->vtx_uart, 4800U,
                                    UART_STOPBITS_2, &uart) ||
        HAL_HalfDuplex_EnableReceiver(&uart) != HAL_OK) {
        fail("UART_ERROR"); return false;
    }
    attempts = changes = rx_position = 0U;
    state = START_DELAY; status = "INITIALIZING";
    deadline = HAL_GetTick() + 1000U;
    return true;
}

static void process_settings(uint8_t payload_size)
{
    const uint8_t version = rx[2];
    if (payload_size < 5U || (version != 1U && version != 9U && version != 17U)) return;
    attempts = 0U;
    const uint16_t frequency = (uint16_t)((uint16_t)rx[7] << 8) | rx[8];
    /* SET_CHANNEL also leaves user-frequency mode, preserving band/channel
     * identity even when two bands happen to contain the same frequency. */
    if (rx[4] != desired_channel || (rx[6] & 1U) || frequency != desired_frequency) {
        if (++changes > MAX_ATTEMPTS) { fail("NOT_CONFIRMED"); return; }
        send_command(3U, &desired_channel, 1U); return;
    }
    uint8_t target_power = 0U, actual_power = rx[5];
    if (version == 17U) {
        if (payload_size < 8U) { fail("UNSUPPORTED_POWER"); return; }
        target_power = (uint8_t)lroundf(10.0f * log10f((float)desired_power));
        const uint8_t count = (uint8_t)(rx[10] + 1U);
        bool supported = false;
        if (count > payload_size - 7U || count > BUFFER_SIZE - 11U) {
            fail("UNSUPPORTED_POWER"); return;
        }
        for (uint8_t i = 0U; i < count; ++i)
            if (rx[11U + i] == target_power) supported = true;
        if (!supported) { fail("UNSUPPORTED_POWER"); return; }
        actual_power = rx[9];
    } else {
        static const uint16_t powers[] = {25U, 200U, 500U, 800U};
        static const uint8_t dac[] = {7U, 16U, 25U, 40U};
        uint8_t index = 0U;
        while (index < 4U && powers[index] != desired_power) ++index;
        if (index == 4U) { fail("UNSUPPORTED_POWER"); return; }
        target_power = version == 1U ? dac[index] : index;
    }
    if (actual_power != target_power) {
        if (++changes > MAX_ATTEMPTS) { fail("NOT_CONFIRMED"); return; }
        if (version == 17U) target_power |= 0x80U;
        send_command(2U, &target_power, 1U); return;
    }
    status = "APPLIED"; state = DONE;
}

/* Accept response lengths used by TBS rev09 (CRC included) and by devices
 * with a reserved payload byte (CRC excluded). Never accept command echoes. */
static void receive(uint8_t byte)
{
    if (rx_position == 0U) { if (byte == 0xAAU) rx[rx_position++] = byte; return; }
    if (rx_position == 1U && byte != 0x55U) {
        rx_position = byte == 0xAAU ? 1U : 0U; return;
    }
    rx[rx_position++] = byte;
    if (rx_position < 4U) return;
    const uint8_t length = rx[3];
    if (length > BUFFER_SIZE - 5U) { rx_position = 0U; return; }
    if (rx_position >= 5U &&
        (rx_position == length + 4U || rx_position == length + 5U) &&
        crc8(&rx[2], (uint8_t)(rx_position - 3U)) == byte) {
        const uint8_t size = (uint8_t)(rx_position - 5U);
        rx_position = 0U;
        if (command == 1U) process_settings(size);
        else if (rx[2] == command) schedule_query();
    } else if (rx_position >= length + 5U) rx_position = 0U;
}

void vtx_smartaudio_update(bool armed)
{
    if (state == IDLE || state == DONE || state == FAILED) return;
    const uint32_t now = HAL_GetTick();
    /* Do not start or continue a configuration transaction in flight. */
    if (armed) {
        (void)HAL_HalfDuplex_EnableReceiver(&uart);
        rx_position = 0U; state = START_DELAY; deadline = now + 1000U; return;
    }
    if (state == START_DELAY || state == QUERY_DELAY) {
        if ((int32_t)(now - deadline) >= 0) query();
        return;
    }
    if (state == TRANSMITTING) {
        if (tx_position < tx_size && __HAL_UART_GET_FLAG(&uart, UART_FLAG_TXE) != RESET) {
#if defined(PLATFORM_STM32H7)
            uart.Instance->TDR = tx[tx_position++];
#else
            uart.Instance->DR = tx[tx_position++];
#endif
        } else if (tx_position == tx_size && __HAL_UART_GET_FLAG(&uart, UART_FLAG_TC) != RESET) {
            if (HAL_HalfDuplex_EnableReceiver(&uart) != HAL_OK) { fail("UART_ERROR"); return; }
            state = WAIT_RESPONSE; deadline = now + 300U; return;
        }
        if ((int32_t)(now - deadline) >= 0) fail("UART_ERROR");
        return;
    }
    /* Bound receive work per service tick, including garbage from the line. */
    if (__HAL_UART_GET_FLAG(&uart, UART_FLAG_ORE) != RESET ||
        __HAL_UART_GET_FLAG(&uart, UART_FLAG_FE) != RESET ||
        __HAL_UART_GET_FLAG(&uart, UART_FLAG_NE) != RESET) {
        __HAL_UART_CLEAR_OREFLAG(&uart);
        __HAL_UART_CLEAR_FEFLAG(&uart);
        __HAL_UART_CLEAR_NEFLAG(&uart);
        __HAL_UART_FLUSH_DRREGISTER(&uart);
        rx_position = 0U;
    }
    for (uint8_t i = 0U; i < BUFFER_SIZE && state == WAIT_RESPONSE; ++i) {
        if (__HAL_UART_GET_FLAG(&uart, UART_FLAG_RXNE) == RESET) break;
#if defined(PLATFORM_STM32H7)
        const uint8_t byte = (uint8_t)uart.Instance->RDR;
#else
        const uint8_t byte = (uint8_t)uart.Instance->DR;
#endif
        receive(byte);
    }
    if (state == WAIT_RESPONSE && (int32_t)(now - deadline) >= 0) {
        if (++attempts >= MAX_ATTEMPTS) fail("NO_RESPONSE");
        else schedule_query();
    }
}
