#include "sbus.h"

#include <string.h>

#include "board.h"

#define FRAME_SIZE 25U
#define SIGNAL_TIMEOUT_US 100000U
#define RX_RING_SIZE 256U
#define INVERTER_PROBE_US 400000U
#define RECOVERY_RETRY_US 100000U
#define FAILSAFE_CONFIRM_US 100000U

static uint8_t frame[FRAME_SIZE];
static uint8_t frame_index;
static sbus_data_t data;
static uint8_t irq_byte;
static volatile uint8_t rx_ring[RX_RING_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static uint32_t last_inverter_probe_us;
static bool inverter_locked;
static GPIO_PinState inverter_level;
static uint32_t last_recovery_us;
static volatile bool uart_recovery_pending;
static volatile bool ring_resync_pending;
static bool failsafe_pending;
static uint32_t failsafe_started_us;

static void restart_uart_receive(void)
{
    /*
     * A wrong inverter state produces a continuous stream of parity/framing
     * errors.  Reset the HAL receive state as well as every UART error flag,
     * otherwise Receive_IT can remain BUSY after the inverter is corrected.
     */
    HAL_UART_AbortReceive(&hsbus_uart);
    __HAL_UART_CLEAR_PEFLAG(&hsbus_uart);
    __HAL_UART_CLEAR_FEFLAG(&hsbus_uart);
    __HAL_UART_CLEAR_NEFLAG(&hsbus_uart);
    __HAL_UART_CLEAR_OREFLAG(&hsbus_uart);
    __HAL_UART_FLUSH_DRREGISTER(&hsbus_uart);
    HAL_UART_Receive_IT(&hsbus_uart, &irq_byte, 1U);
}

static void resync_frame(void)
{
    for (uint8_t offset = 1U; offset < FRAME_SIZE; ++offset) {
        if (frame[offset] == 0x0FU) {
            frame_index = (uint8_t)(FRAME_SIZE - offset);
            memmove(frame, &frame[offset], frame_index);
            return;
        }
    }
    frame_index = 0U;
}

static void decode(void)
{
    const uint32_t now_us = board_micros();
    const bool frame_failsafe = (frame[23] & 0x08U) != 0U;
    data.last_frame_us = now_us;
    ++data.valid_frame_count;
    inverter_locked = true;

    if (frame_failsafe) {
        if (!failsafe_pending) {
            failsafe_pending = true;
            failsafe_started_us = now_us;
        }
        data.failsafe =
            (uint32_t)(now_us - failsafe_started_us) >= FAILSAFE_CONFIRM_US;
        return;
    }

    failsafe_pending = false;
    data.failsafe = false;
    const uint8_t *payload = &frame[1];
    uint32_t accumulator = 0;
    uint8_t bits = 0;
    uint8_t offset = 0;

    for (uint8_t channel = 0; channel < SBUS_CHANNEL_COUNT; ++channel) {
        while (bits < 11U) {
            accumulator |= (uint32_t)payload[offset++] << bits;
            bits += 8U;
        }
        const uint16_t raw = accumulator & 0x07FFU;
        data.channel_us[channel] = 880U + (uint16_t)(((uint32_t)raw * 5U + 4U) / 8U);
        accumulator >>= 11U;
        bits -= 11U;
    }

}

void sbus_init(void)
{
    memset(&data, 0, sizeof(data));
    frame_index = 0;
    rx_head = rx_tail = 0U;
    inverter_level = SBUS_INVERTER_ENABLE_LEVEL;
    last_inverter_probe_us = board_micros();
    inverter_locked = false;
    last_recovery_us = 0U;
    failsafe_pending = false;
    failsafe_started_us = 0U;
    HAL_UART_Receive_IT(&hsbus_uart, &irq_byte, 1U);
}

void sbus_update(void)
{
    if (uart_recovery_pending) {
        __disable_irq();
        uart_recovery_pending = false;
        __enable_irq();
        frame_index = 0U;
        rx_tail = rx_head;
        restart_uart_receive();
    }
    if (ring_resync_pending) {
        __disable_irq();
        ring_resync_pending = false;
        __enable_irq();
        frame_index = 0U;
        rx_tail = rx_head;
    }
    while (rx_tail != rx_head) {
        const uint8_t byte = rx_ring[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1U) % RX_RING_SIZE);
        if (frame_index == 0U && byte != 0x0FU) {
            continue;
        }
        frame[frame_index++] = byte;
        if (frame_index == FRAME_SIZE) {
            if (frame[0] == 0x0FU &&
                (frame[24] == 0x00U || (frame[24] & 0x0FU) == 0x04U)) {
                decode();
                frame_index = 0U;
            } else {
                ++data.invalid_frame_count;
                resync_frame();
            }
        }
    }
    const uint32_t now_us = board_micros();
    if (!inverter_locked &&
        (uint32_t)(now_us - last_inverter_probe_us) >= INVERTER_PROBE_US) {
        inverter_level = inverter_level == GPIO_PIN_SET ? GPIO_PIN_RESET
                                                        : GPIO_PIN_SET;
        HAL_GPIO_WritePin(SBUS_INVERTER_PORT, SBUS_INVERTER_PIN, inverter_level);
        last_inverter_probe_us = now_us;
        frame_index = 0U;
        rx_tail = rx_head;
        restart_uart_receive();
    }
    data.frame_age_us = data.last_frame_us == 0U
                            ? UINT32_MAX
                            : now_us - data.last_frame_us;
    if (data.failsafe) {
        data.loss_reason = SBUS_LOSS_FAILSAFE;
    } else if (data.last_frame_us == 0U ||
               data.frame_age_us >= SIGNAL_TIMEOUT_US) {
        data.loss_reason = SBUS_LOSS_TIMEOUT;
    } else {
        data.loss_reason = SBUS_LOSS_NONE;
    }
    data.valid = data.loss_reason == SBUS_LOSS_NONE;

    /*
     * A UART can occasionally stop producing callbacks without raising a
     * recoverable HAL error. Restart reception after a signal timeout, while
     * retaining the inverter polarity that previously decoded valid frames.
     */
    if (inverter_locked && data.loss_reason == SBUS_LOSS_TIMEOUT &&
        (uint32_t)(now_us - last_recovery_us) >= RECOVERY_RETRY_US) {
        last_recovery_us = now_us;
        frame_index = 0U;
        rx_tail = rx_head;
        ++data.recovery_count;
        restart_uart_receive();
    }
}

const sbus_data_t *sbus_get(void)
{
    return &data;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart->Instance != SBUS_UART_INSTANCE) {
        return;
    }
    const uint16_t next = (uint16_t)((rx_head + 1U) % RX_RING_SIZE);
    if (next != rx_tail) {
        rx_ring[rx_head] = irq_byte;
        rx_head = next;
    } else {
        ++data.ring_overrun_count;
        ring_resync_pending = true;
    }
    HAL_UART_Receive_IT(&hsbus_uart, &irq_byte, 1U);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart->Instance == SBUS_UART_INSTANCE) {
        ++data.uart_error_count;
        ++data.recovery_count;
        last_recovery_us = board_micros();
        uart_recovery_pending = true;
    }
}
