#include "dshot.h"

#include "board.h"

#define DSHOT_MIN 48U
#define DSHOT_MAX 2047U
#define DSHOT_STARTUP_TIME_MS 500U
#define DSHOT_STARTUP_FRAME_INTERVAL_MS 1U
#define DSHOT_FRAME_WORDS 18U
#define DSHOT_MOTOR_COUNT 4U

static motor_protocol_t active_protocol = MOTOR_PROTOCOL_DSHOT300;
static uint16_t dshot_dma_buffer[DSHOT_FRAME_WORDS][DSHOT_MOTOR_COUNT]
    __attribute__((aligned(32)));

static uint32_t dshot_period_ticks(void)
{
    if (active_protocol == MOTOR_PROTOCOL_DSHOT1200) return 10U;
    if (active_protocol == MOTOR_PROTOCOL_DSHOT600) return 20U;
    return 40U;
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

static uint32_t timer_clock_hz(void)
{
    uint32_t clock = HAL_RCC_GetPCLK1Freq();
    if ((RCC->D2CFGR & RCC_D2CFGR_D2PPRE1) != RCC_APB1_DIV1) clock *= 2U;
    return clock;
}

static bool transfer_active(void)
{
    return (DMA1_Stream0->CR & DMA_SxCR_EN) != 0U;
}

static void hardware_init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {
        .Pin = MOTOR_1_PIN | MOTOR_2_PIN | MOTOR_3_PIN | MOTOR_4_PIN,
        .Mode = GPIO_MODE_AF_PP,
        .Pull = GPIO_PULLDOWN,
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
        .Alternate = GPIO_AF2_TIM3,
    };
    HAL_GPIO_Init(GPIOC, &gpio);

    const uint32_t psc = timer_clock_hz() / 12000000U - 1U;
    const uint32_t period = dshot_period_ticks();
    TIM3->CR1 = 0U;
    TIM3->PSC = psc;
    TIM3->ARR = period - 1U;
    TIM3->CCR1 = TIM3->CCR2 = TIM3->CCR3 = TIM3->CCR4 = 0U;
    TIM3->CCMR1 = (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE |
                  (6U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    TIM3->CCMR2 = (6U << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE |
                  (6U << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;
    TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E |
                 TIM_CCER_CC3E | TIM_CCER_CC4E;
    /* DMA burst base CCR1, four half-word transfers per update event. */
    TIM3->DCR = (13U << TIM_DCR_DBA_Pos) | (3U << TIM_DCR_DBL_Pos);
    TIM3->EGR = TIM_EGR_UG;
    TIM3->SR = 0U;
    TIM3->DIER = 0U;
    TIM3->CR1 = TIM_CR1_ARPE;

    DMA1_Stream0->CR &= ~DMA_SxCR_EN;
    while ((DMA1_Stream0->CR & DMA_SxCR_EN) != 0U) {
    }
    DMAMUX1_Channel0->CCR = DMA_REQUEST_TIM3_UP;
    DMA1_Stream0->CR = DMA_MEMORY_TO_PERIPH | DMA_PINC_DISABLE |
                       DMA_MINC_ENABLE | DMA_PDATAALIGN_HALFWORD |
                       DMA_MDATAALIGN_HALFWORD | DMA_NORMAL |
                       DMA_PRIORITY_VERY_HIGH;
    DMA1_Stream0->PAR = (uint32_t)&TIM3->DMAR;
    DMA1_Stream0->FCR = DMA_FIFOMODE_DISABLE;
}

void dshot_init(void) { hardware_init(); }

void dshot_write(const uint16_t values[4])
{
    if (transfer_active()) return;
    const uint32_t period = dshot_period_ticks();
    for (uint8_t motor = 0U; motor < DSHOT_MOTOR_COUNT; ++motor) {
        const uint16_t frame = packet(sanitize(values[motor]));
        for (uint8_t bit = 0U; bit < 16U; ++bit) {
            dshot_dma_buffer[bit][motor] =
                (frame & (1U << (15U - bit))) != 0U
                    ? (uint16_t)((period * 3U + 2U) / 4U)
                    : (uint16_t)((period * 3U + 4U) / 8U);
        }
        dshot_dma_buffer[16][motor] = 0U;
        dshot_dma_buffer[17][motor] = 0U;
    }

    TIM3->CR1 &= ~TIM_CR1_CEN;
    TIM3->DIER &= ~TIM_DIER_UDE;
    DMA1_Stream0->CR &= ~DMA_SxCR_EN;
    DMA1->LIFCR = DMA_LIFCR_CFEIF0 | DMA_LIFCR_CDMEIF0 |
                  DMA_LIFCR_CTEIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTCIF0;
    DMA1_Stream0->M0AR = (uint32_t)dshot_dma_buffer;
    DMA1_Stream0->NDTR = DSHOT_FRAME_WORDS * DSHOT_MOTOR_COUNT;
    TIM3->CNT = 0U;
    TIM3->SR = 0U;
    __DMB();
    DMA1_Stream0->CR |= DMA_SxCR_EN;
    TIM3->DIER |= TIM_DIER_UDE;
    TIM3->CR1 |= TIM_CR1_CEN;
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

uint16_t dshot_from_percent(float percent)
{
    if (percent <= 0.0f) return 0U;
    if (percent > 100.0f) percent = 100.0f;
    return DSHOT_MIN + (uint16_t)(percent *
        (float)(DSHOT_MAX - DSHOT_MIN) / 100.0f);
}
