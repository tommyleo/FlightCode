#include "dshot.h"
#include "esc_passthrough_io.h"
#include "board.h"
#include <math.h>

#define FRAME_WORDS 18U
static uint16_t frame[4][FRAME_WORDS] __attribute__((aligned(4)));
static motor_protocol_t protocol = MOTOR_PROTOCOL_DSHOT600;
static uint32_t period_ticks, frame_started_us, frame_time_us;
static bool initialized, frame_active, passthrough;
static dma_channel_type *const channels[4] = {
    DMA1_CHANNEL1, DMA1_CHANNEL2, DMA1_CHANNEL3, DMA1_CHANNEL4
};
static dmamux_channel_type *const mux[4] = {
    DMA1MUX_CHANNEL1, DMA1MUX_CHANNEL2, DMA1MUX_CHANNEL3, DMA1MUX_CHANNEL4
};
static tmr_type *const timers[4] = {TMR1, TMR1, TMR2, TMR2};
static const tmr_channel_select_type outputs[4] = {
    TMR_SELECT_CHANNEL_3, TMR_SELECT_CHANNEL_2,
    TMR_SELECT_CHANNEL_2, TMR_SELECT_CHANNEL_1
};
static volatile uint32_t *const compare[4] = {
    &TMR1->c3dt, &TMR1->c2dt, &TMR2->c2dt, &TMR2->c1dt
};
static const uint16_t pins[4] = {MOTOR_1_PIN, MOTOR_2_PIN, MOTOR_3_PIN, MOTOR_4_PIN};

static uint32_t baud_rate(motor_protocol_t p)
{ return p == MOTOR_PROTOCOL_DSHOT300 ? 300000U : p == MOTOR_PROTOCOL_DSHOT1200 ? 1200000U : 600000U; }
static void stop_output(void)
{
    tmr_dma_request_enable(TMR1, TMR_OVERFLOW_DMA_REQUEST, FALSE);
    tmr_dma_request_enable(TMR2, TMR_OVERFLOW_DMA_REQUEST, FALSE);
    tmr_counter_enable(TMR1, FALSE);
    tmr_counter_enable(TMR2, FALSE);
    for (uint8_t i = 0; i < 4; ++i) {
        dma_channel_enable(channels[i], FALSE);
        tmr_channel_value_set(timers[i], outputs[i], 0);
    }
    tmr_event_sw_trigger(TMR1, TMR_OVERFLOW_SWTRIG);
    tmr_event_sw_trigger(TMR2, TMR_OVERFLOW_SWTRIG);
    frame_active = false;
}
static void configure(void)
{
    period_ticks = BOARD_CORE_CLOCK_HZ / baud_rate(protocol);
    frame_time_us = (FRAME_WORDS * 1000000U + baud_rate(protocol) - 1U) / baud_rate(protocol);
    tmr_base_init(TMR1, period_ticks - 1U, 0U);
    tmr_base_init(TMR2, period_ticks - 1U, 0U);
    tmr_output_config_type output;
    tmr_output_default_para_init(&output);
    output.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
    output.oc_output_state = TRUE;
    output.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
    for (uint8_t i = 0; i < 4; ++i) {
        tmr_output_channel_config(timers[i], outputs[i], &output);
        tmr_output_channel_buffer_enable(timers[i], outputs[i], TRUE);
        tmr_channel_value_set(timers[i], outputs[i], 0);
        dma_init_type d;
        dma_default_para_init(&d);
        d.peripheral_base_addr = (uint32_t)compare[i];
        d.memory_base_addr = (uint32_t)&frame[i][2];
        d.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
        d.buffer_size = FRAME_WORDS - 2;
        d.memory_inc_enable = TRUE;
        d.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
        d.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
        d.priority = DMA_PRIORITY_VERY_HIGH;
        dma_init(channels[i], &d);
        dmamux_init(mux[i], i < 2 ? DMAMUX_DMAREQ_ID_TMR1_OVERFLOW : DMAMUX_DMAREQ_ID_TMR2_OVERFLOW);
    }
    tmr_output_enable(TMR1, TRUE);
    tmr_event_sw_trigger(TMR1, TMR_OVERFLOW_SWTRIG);
    tmr_event_sw_trigger(TMR2, TMR_OVERFLOW_SWTRIG);
}
void dshot_init(void)
{
    crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_TMR2_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
    dmamux_enable(DMA1, TRUE);
    gpio_bits_reset(GPIOA, MOTOR_1_PIN | MOTOR_2_PIN | MOTOR_3_PIN | MOTOR_4_PIN);
    at32_gpio_config(GPIOA, MOTOR_1_PIN | MOTOR_2_PIN | MOTOR_3_PIN | MOTOR_4_PIN,
                      GPIO_MODE_MUX, GPIO_PULL_DOWN, GPIO_MUX_1);
    initialized = true;
    stop_output();
    configure();
}
void dshot_write(const uint16_t values[4])
{
    if (!initialized || passthrough) return;
    /* Never cut off a DMA frame if the scheduler is early or recovering. */
    if (frame_active && (uint32_t)(board_micros() - frame_started_us) < frame_time_us) return;
    stop_output();
    for (uint8_t i = 0; i < 4; ++i) {
        const uint16_t value = values[i] > 2047U ? 2047U : values[i];
        const uint16_t data = value << 1U; /* telemetry request bit is clear */
        const uint16_t packet = (data << 4U) | ((data ^ (data >> 4U) ^ (data >> 8U)) & 15U);
        for (uint8_t bit = 0; bit < 16; ++bit)
            frame[i][bit] = packet & (0x8000U >> bit) ? period_ticks * 3U / 4U : period_ticks * 3U / 8U;
        frame[i][16] = frame[i][17] = 0;
        tmr_channel_value_set(timers[i], outputs[i], frame[i][0]);
    }
    /* Latch bit 0, stage bit 1; overflow DMA then stages bits 2..17. */
    tmr_event_sw_trigger(TMR1, TMR_OVERFLOW_SWTRIG);
    tmr_event_sw_trigger(TMR2, TMR_OVERFLOW_SWTRIG);
    dma_flag_clear(DMA1_GL1_FLAG | DMA1_GL2_FLAG | DMA1_GL3_FLAG | DMA1_GL4_FLAG);
    for (uint8_t i = 0; i < 4; ++i) {
        tmr_channel_value_set(timers[i], outputs[i], frame[i][1]);
        channels[i]->maddr = (uint32_t)&frame[i][2];
        dma_data_number_set(channels[i], FRAME_WORDS - 2);
        dma_channel_enable(channels[i], TRUE);
    }
    tmr_counter_value_set(TMR1, 0); tmr_counter_value_set(TMR2, 0);
    tmr_dma_request_enable(TMR1, TMR_OVERFLOW_DMA_REQUEST, TRUE);
    tmr_dma_request_enable(TMR2, TMR_OVERFLOW_DMA_REQUEST, TRUE);
    frame_started_us = board_micros();
    frame_active = true;
    tmr_counter_enable(TMR1, TRUE); tmr_counter_enable(TMR2, TRUE);
}
uint16_t dshot_from_percent(float percent)
{
    if (!isfinite(percent) || percent <= 0) return 0;
    if (percent > 100) percent = 100;
    return (uint16_t)(48.0f + percent * (1999.0f / 100.0f));
}
bool motor_protocol_set(motor_protocol_t p)
{
    if (p != MOTOR_PROTOCOL_DSHOT300 && p != MOTOR_PROTOCOL_DSHOT600 && p != MOTOR_PROTOCOL_DSHOT1200) return false;
    if (initialized && p != protocol) {
        while (frame_active && (uint32_t)(board_micros() - frame_started_us) < frame_time_us) {}
        stop_output(); protocol = p; configure();
    } else protocol = p;
    return true;
}
motor_protocol_t motor_protocol_get(void) { return protocol; }
const char *motor_protocol_name(motor_protocol_t p)
{ return p == MOTOR_PROTOCOL_DSHOT300 ? "DSHOT300" : p == MOTOR_PROTOCOL_DSHOT600 ? "DSHOT600" : p == MOTOR_PROTOCOL_DSHOT1200 ? "DSHOT1200" : "UNKNOWN"; }
void dshot_startup_sequence(void)
{
    const uint16_t zero[4] = {0};
    const uint32_t start = HAL_GetTick();
    do { dshot_write(zero); HAL_Delay(1); } while ((uint32_t)(HAL_GetTick() - start) < 300U);
}
uint8_t esc_passthrough_count(void) { return 4; }
void esc_passthrough_begin(void)
{
    while (frame_active && (uint32_t)(board_micros() - frame_started_us) < frame_time_us) {}
    stop_output(); passthrough = true;
    gpio_bits_set(GPIOA, MOTOR_1_PIN | MOTOR_2_PIN | MOTOR_3_PIN | MOTOR_4_PIN);
    at32_gpio_config(GPIOA, MOTOR_1_PIN | MOTOR_2_PIN | MOTOR_3_PIN | MOTOR_4_PIN,
                      GPIO_MODE_INPUT, GPIO_PULL_UP, GPIO_MUX_0);
}
void esc_passthrough_end(void) { passthrough = false; dshot_init(); }
void esc_passthrough_input(uint8_t index)
{ if (index < 4) at32_gpio_config(GPIOA, pins[index], GPIO_MODE_INPUT, GPIO_PULL_UP, GPIO_MUX_0); }
void esc_passthrough_output(uint8_t index)
{
    if (index >= 4) return;
    gpio_bits_set(GPIOA, pins[index]);
    at32_gpio_config(GPIOA, pins[index], GPIO_MODE_OUTPUT, GPIO_PULL_UP, GPIO_MUX_0);
}
bool esc_passthrough_read(uint8_t index)
{ return index < 4 && gpio_input_data_bit_read(GPIOA, pins[index]) == SET; }
void esc_passthrough_write(uint8_t index, bool high)
{ if (index < 4) gpio_bits_write(GPIOA, pins[index], high ? TRUE : FALSE); }
uint32_t esc_passthrough_micros(void) { return board_micros(); }
uint32_t esc_passthrough_timing_now(void) { return DWT->CYCCNT; }
void esc_passthrough_wait_until(uint32_t started, uint32_t us)
{ while ((uint32_t)(DWT->CYCCNT - started) < us * (BOARD_CORE_CLOCK_HZ / 1000000U)) __NOP(); }
uint32_t esc_passthrough_critical_enter(void)
{ uint32_t state = __get_PRIMASK(); __disable_irq(); return state; }
void esc_passthrough_critical_exit(uint32_t state) { __set_PRIMASK(state); }
