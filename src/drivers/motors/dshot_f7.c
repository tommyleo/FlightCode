#include "dshot.h"

#include "esc_passthrough_io.h"
#include "board.h"

#define DSHOT_MIN 48U
#define DSHOT_MAX 2047U
#define DSHOT_STARTUP_TIME_MS 500U
#define DSHOT_STARTUP_FRAME_INTERVAL_MS 1U
#define DSHOT_FRAME_WORDS 18U
#define DSHOT_MOTOR_COUNT 4U

static motor_protocol_t active_protocol = MOTOR_PROTOCOL_DSHOT300;
static uint16_t dshot_dma_buffer[2][DSHOT_FRAME_WORDS][2]
    __attribute__((aligned(32)));

static uint32_t dshot_bitrate_hz(void)
{
    if (active_protocol == MOTOR_PROTOCOL_DSHOT1200) return 1200000U;
    if (active_protocol == MOTOR_PROTOCOL_DSHOT600) return 600000U;
    return 300000U;
}

static uint16_t sanitize(uint16_t value)
{
    if (value < DSHOT_MIN) return 0U;
    return value > DSHOT_MAX ? DSHOT_MAX : value;
}

static uint16_t packet(uint16_t value)
{
    const uint16_t payload = (uint16_t)(value << 1U);
    uint16_t checksum_data = payload;
    uint16_t checksum = 0U;
    for (uint8_t i = 0U; i < 3U; ++i) {
        checksum ^= checksum_data;
        checksum_data >>= 4U;
    }
    return (uint16_t)((payload << 4U) | (checksum & 0x0FU));
}

/* TIM1 CH1=M2, CH2=M1; TIM8 CH3=M4, CH4=M3. The DMA buffers live
 * in SRAM (the linker excludes DTCM), with data cache disabled. */
static uint32_t dshot_period_ticks(void)
{
    uint32_t clock = HAL_RCC_GetPCLK2Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE2) != RCC_CFGR_PPRE2_DIV1) clock *= 2U;
    const uint32_t bitrate = dshot_bitrate_hz();
    return (clock + bitrate / 2U) / bitrate;
}

static bool transfer_active(void)
{
    return ((DMA2_Stream5->CR | DMA2_Stream1->CR) & DMA_SxCR_EN) != 0U;
}

static void timer_init(TIM_TypeDef *tim, uint32_t base, bool upper)
{
    tim->CR1 = 0U;
    tim->PSC = 0U;
    tim->ARR = dshot_period_ticks() - 1U;
    tim->CCR1 = tim->CCR2 = tim->CCR3 = tim->CCR4 = 0U;
    const uint32_t pwm = (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE |
                        (6U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    tim->CCMR1 = upper ? 0U : pwm;
    tim->CCMR2 = upper ? pwm : 0U;
    tim->CCER = upper ? TIM_CCER_CC3E | TIM_CCER_CC4E :
                        TIM_CCER_CC1E | TIM_CCER_CC2E;
    tim->BDTR = TIM_BDTR_MOE;
    tim->RCR = 0U;
    tim->DCR = (base << TIM_DCR_DBA_Pos) | (1U << TIM_DCR_DBL_Pos);
    tim->EGR = TIM_EGR_UG;
    tim->SR = 0U;
    tim->DIER = 0U;
    tim->CR1 = TIM_CR1_ARPE;
}

static void dma_init(DMA_Stream_TypeDef *stream, TIM_TypeDef *tim, uint32_t channel)
{
    stream->CR &= ~DMA_SxCR_EN;
    while ((stream->CR & DMA_SxCR_EN) != 0U) { }
    stream->CR = channel | DMA_MEMORY_TO_PERIPH | DMA_MINC_ENABLE |
                 DMA_PDATAALIGN_HALFWORD | DMA_MDATAALIGN_HALFWORD |
                 DMA_NORMAL | DMA_PRIORITY_VERY_HIGH;
    stream->PAR = (uint32_t)&tim->DMAR;
    stream->FCR = DMA_FIFOMODE_DISABLE;
}

static void hardware_init(void)
{
    __HAL_RCC_DMA2_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM8_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {.Pin=MOTOR_1_PIN | MOTOR_2_PIN,
        .Mode=GPIO_MODE_AF_PP, .Pull=GPIO_PULLDOWN,
        .Speed=GPIO_SPEED_FREQ_VERY_HIGH, .Alternate=GPIO_AF1_TIM1};
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = MOTOR_3_PIN | MOTOR_4_PIN;
    gpio.Alternate = GPIO_AF3_TIM8;
    HAL_GPIO_Init(GPIOC, &gpio);
    timer_init(TIM1, 13U, false); /* Burst CCR1/CCR2. */
    timer_init(TIM8, 15U, true);  /* Burst CCR3/CCR4. */
    dma_init(DMA2_Stream5, TIM1, DMA_CHANNEL_6); /* TIM1_UP */
    dma_init(DMA2_Stream1, TIM8, DMA_CHANNEL_7); /* TIM8_UP */
}

void dshot_init(void) { hardware_init(); }

void dshot_write(const uint16_t values[4])
{
    if (transfer_active()) return;
    const uint32_t period = dshot_period_ticks();
    for (uint8_t motor=0U; motor<4U; ++motor) {
        const uint16_t frame = packet(sanitize(values[motor]));
        const uint8_t group = motor / 2U, channel = 1U - motor % 2U;
        for (uint8_t bit=0U; bit<16U; ++bit) {
            dshot_dma_buffer[group][bit][channel] =
                (frame & (1U << (15U - bit))) != 0U ?
                    (uint16_t)((period * 14U + 10U) / 20U) :
                    (uint16_t)((period * 7U + 10U) / 20U);
        }
        dshot_dma_buffer[group][16][channel] = 0U;
        dshot_dma_buffer[group][17][channel] = 0U;
    }
    TIM1->CR1 &= ~TIM_CR1_CEN;
    TIM8->CR1 &= ~TIM_CR1_CEN;
    TIM1->DIER &= ~TIM_DIER_UDE;
    TIM8->DIER &= ~TIM_DIER_UDE;
    DMA2->HIFCR = DMA_HIFCR_CFEIF5 | DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CTEIF5 |
                   DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTCIF5;
    DMA2->LIFCR = DMA_LIFCR_CFEIF1 | DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CTEIF1 |
                   DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTCIF1;
    DMA2_Stream5->M0AR = (uint32_t)dshot_dma_buffer[0];
    DMA2_Stream1->M0AR = (uint32_t)dshot_dma_buffer[1];
    DMA2_Stream5->NDTR = DMA2_Stream1->NDTR = DSHOT_FRAME_WORDS * 2U;
    TIM1->CNT = TIM8->CNT = 0U;
    TIM1->SR = TIM8->SR = 0U;
    __DMB();
    DMA2_Stream5->CR |= DMA_SxCR_EN;
    DMA2_Stream1->CR |= DMA_SxCR_EN;
    TIM1->DIER |= TIM_DIER_UDE;
    TIM8->DIER |= TIM_DIER_UDE;
    TIM1->CR1 |= TIM_CR1_CEN;
    TIM8->CR1 |= TIM_CR1_CEN;
}

void dshot_startup_sequence(void)
{
    static const uint16_t stopped[4] = {0U, 0U, 0U, 0U};
    const uint32_t started = HAL_GetTick();
    do {
        dshot_write(stopped);
        HAL_Delay(DSHOT_STARTUP_FRAME_INTERVAL_MS);
    } while ((uint32_t)(HAL_GetTick() - started) < DSHOT_STARTUP_TIME_MS);
}

bool motor_protocol_set(motor_protocol_t protocol)
{
    if (protocol != MOTOR_PROTOCOL_DSHOT300 &&
        protocol != MOTOR_PROTOCOL_DSHOT600 &&
        protocol != MOTOR_PROTOCOL_DSHOT1200) return false;
    if (protocol == active_protocol) return true;
    active_protocol = protocol;
    hardware_init();
    return true;
}

motor_protocol_t motor_protocol_get(void) { return active_protocol; }

const char *motor_protocol_name(motor_protocol_t protocol)
{
    if (protocol == MOTOR_PROTOCOL_DSHOT1200) return "DSHOT1200";
    if (protocol == MOTOR_PROTOCOL_DSHOT600) return "DSHOT600";
    return "DSHOT300";
}

static GPIO_TypeDef *const am32_ports[4] = {
    MOTOR_1_PORT, MOTOR_2_PORT, MOTOR_3_PORT, MOTOR_4_PORT
};
static const uint16_t am32_pins[4] = {
    MOTOR_1_PIN, MOTOR_2_PIN, MOTOR_3_PIN, MOTOR_4_PIN
};

uint8_t esc_passthrough_count(void) { return 4U; }

void esc_passthrough_begin(void)
{
    TIM1->DIER &= ~TIM_DIER_UDE;
    TIM8->DIER &= ~TIM_DIER_UDE;
    TIM1->CR1 &= ~TIM_CR1_CEN;
    TIM8->CR1 &= ~TIM_CR1_CEN;
    DMA2_Stream5->CR &= ~DMA_SxCR_EN;
    DMA2_Stream1->CR &= ~DMA_SxCR_EN;
    while (((DMA2_Stream5->CR | DMA2_Stream1->CR) & DMA_SxCR_EN) != 0U) { }
    for (uint8_t i = 0U; i < 4U; ++i) {
        HAL_GPIO_WritePin(am32_ports[i], am32_pins[i], GPIO_PIN_SET);
        GPIO_InitTypeDef gpio = {
            .Pin = am32_pins[i], .Mode = GPIO_MODE_INPUT,
            .Pull = GPIO_PULLUP, .Speed = GPIO_SPEED_FREQ_VERY_HIGH
        };
        HAL_GPIO_Init(am32_ports[i], &gpio);
    }
}

void esc_passthrough_end(void) { hardware_init(); }

void esc_passthrough_input(uint8_t index)
{
    GPIO_InitTypeDef gpio = {
        .Pin = am32_pins[index], .Mode = GPIO_MODE_INPUT,
        .Pull = GPIO_PULLUP, .Speed = GPIO_SPEED_FREQ_VERY_HIGH
    };
    HAL_GPIO_Init(am32_ports[index], &gpio);
}

void esc_passthrough_output(uint8_t index)
{
    HAL_GPIO_WritePin(am32_ports[index], am32_pins[index], GPIO_PIN_SET);
    GPIO_InitTypeDef gpio = {
        .Pin = am32_pins[index], .Mode = GPIO_MODE_OUTPUT_PP,
        .Pull = GPIO_PULLUP, .Speed = GPIO_SPEED_FREQ_VERY_HIGH
    };
    HAL_GPIO_Init(am32_ports[index], &gpio);
}

bool esc_passthrough_read(uint8_t index)
{
    return HAL_GPIO_ReadPin(am32_ports[index], am32_pins[index]) == GPIO_PIN_SET;
}

void esc_passthrough_write(uint8_t index, bool high)
{
    HAL_GPIO_WritePin(am32_ports[index], am32_pins[index],
                      high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

uint32_t esc_passthrough_micros(void) { return board_micros(); }

uint32_t esc_passthrough_timing_now(void) { return DWT->CYCCNT; }

void esc_passthrough_wait_until(uint32_t started, uint32_t offset_us)
{
    const uint32_t cycles_per_us = SystemCoreClock / 1000000U;
    while ((uint32_t)(DWT->CYCCNT - started) < offset_us * cycles_per_us) { }
}

uint32_t esc_passthrough_critical_enter(void)
{
    const uint32_t state = __get_PRIMASK();
    __disable_irq();
    return state;
}

void esc_passthrough_critical_exit(uint32_t state)
{
    if (state == 0U) __enable_irq();
}

uint16_t dshot_from_percent(float percent)
{
    if (percent <= 0.0f) return 0U;
    if (percent > 100.0f) percent = 100.0f;
    return DSHOT_MIN + (uint16_t)(percent *
        (float)(DSHOT_MAX - DSHOT_MIN) / 100.0f);
}
