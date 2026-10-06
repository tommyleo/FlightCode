#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "sbus.h"

#define CRSF_RC_CHANNELS_PACKED 0x16U
static uint8_t frame[64];
static sbus_data_t data;
static bool inverter_locked;
static bool uart_recovery_pending, ring_resync_pending;
static uint32_t irq_mask;
static uint32_t __get_PRIMASK(void) { return irq_mask; }
static void __disable_irq(void) { irq_mask = 1; }
static void __enable_irq(void) { irq_mask = 0; }
static uint32_t board_micros(void) { return 1234U; }
#include "crsf_decode_under_test.inc"

static void make_frame(uint8_t type, uint8_t length)
{
    memset(frame, 0, sizeof(frame));
    frame[0] = 0xC8;
    frame[1] = length;
    frame[2] = type;
    frame[length + 1U] = crsf_crc8(&frame[2], length - 1U);
}

int main(void)
{
    data.failsafe = true;
    make_frame(0x14, 12); /* Valid link statistics, five messages per second. */
    for (unsigned i = 0; i < 5; ++i) decode_crsf();
    assert(data.invalid_frame_count == 0 && data.valid_frame_count == 0);
    assert(data.last_frame_us == 0 && data.failsafe && !inverter_locked);
    frame[13] ^= 1;
    decode_crsf();
    assert(data.invalid_frame_count == 1);
    make_frame(0x16, 23); /* CRC-valid but incomplete channel payload. */
    decode_crsf();
    assert(data.invalid_frame_count == 2 && data.valid_frame_count == 0);
    data.uart_error_count = 373;
    data.recovery_count = 373;
    data.ring_overrun_count = 339;
    uart_recovery_pending = ring_resync_pending = true;
    make_frame(0x16, 24);
    decode_crsf();
    assert(data.valid_frame_count == 1 && data.last_frame_us == 1234);
    assert(!data.failsafe && inverter_locked && data.channel_us[0] == 988);
    assert(data.uart_error_count == 0 && data.recovery_count == 0);
    assert(data.ring_overrun_count == 0 && data.invalid_frame_count == 0);
    assert(!uart_recovery_pending && !ring_resync_pending && irq_mask == 0);
    data.uart_error_count = 1;
    data.recovery_count = 1;
    data.ring_overrun_count = 1;
    frame[25] ^= 1;
    decode_crsf();
    assert(data.invalid_frame_count == 1);
    data.failsafe = true; /* Reacquisition must retain operational errors. */
    make_frame(0x16, 24);
    decode_crsf();
    assert(data.valid_frame_count == 2 && !data.failsafe);
    assert(data.uart_error_count == 1 && data.recovery_count == 1);
    assert(data.ring_overrun_count == 1 && data.invalid_frame_count == 1);
    make_frame(0x29, 5); /* Other supported-by-protocol traffic. */
    decode_crsf();
    assert(data.invalid_frame_count == 1 && data.valid_frame_count == 2);
    puts("CRSF diagnostics: PASS");
}
