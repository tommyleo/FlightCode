#include "dshot.h"

#include "board.h"

#define DSHOT_MIN 48U
#define DSHOT_MAX 2047U
#define DSHOT_STARTUP_TIME_MS 500U
#define DSHOT_STARTUP_FRAME_INTERVAL_MS 1U

static motor_protocol_t active_protocol = MOTOR_PROTOCOL_DSHOT300;

enum { DSHOT_FRAME_WORDS = 18 };
static uint32_t dshot_dma_buffer[4][DSHOT_FRAME_WORDS];

#if BOARD_MOTOR_OUTPUT_LAYOUT == MOTOR_OUTPUT_LAYOUT_TIM2_TIM3
static DMA_Stream_TypeDef *const dshot_dma_streams[4] = {
    DMA1_Stream7, DMA1_Stream2, DMA1_Stream6, DMA1_Stream1,
};
#else
static DMA_Stream_TypeDef *const dshot_dma_streams[4] = {
    DMA1_Stream6, DMA1_Stream4, DMA1_Stream0, DMA1_Stream3,
};
#endif

static bool dshot_dma_transfer_active(void)
{
    for (uint8_t motor = 0U; motor < 4U; ++motor) {
        if ((dshot_dma_streams[motor]->CR & DMA_SxCR_EN) != 0U) {
            return true;
        }
    }
    return false;
}

static uint16_t dshot_sanitize_value(uint16_t value)
{
    if (value < DSHOT_MIN) {
        return 0U;
    }
    return value > DSHOT_MAX ? DSHOT_MAX : value;
}

static uint32_t timer_clock_hz(void)
{
    uint32_t clock = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_HCLK_DIV1) {
        clock *= 2U;
    }
    return clock;
}

static void dma_stream_setup(DMA_Stream_TypeDef *stream, uint32_t channel,
                             volatile uint32_t *peripheral)
{
    stream->CR &= ~DMA_SxCR_EN;
    while ((stream->CR & DMA_SxCR_EN) != 0U) {
    }
    stream->CR = channel | DMA_MEMORY_TO_PERIPH |
                 DMA_PINC_DISABLE | DMA_MINC_ENABLE |
                 DMA_PDATAALIGN_WORD | DMA_MDATAALIGN_WORD |
                 DMA_NORMAL | DMA_PRIORITY_VERY_HIGH;
    stream->PAR = (uint32_t)peripheral;
    stream->FCR = DMA_FIFOMODE_DISABLE;
}

static void hardware_dshot_init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
#if BOARD_MOTOR_OUTPUT_LAYOUT == MOTOR_OUTPUT_LAYOUT_TIM2_TIM3_TIM4
    __HAL_RCC_TIM4_CLK_ENABLE();
#endif

    GPIO_InitTypeDef gpio = {
        .Mode = GPIO_MODE_AF_PP, .Pull = GPIO_PULLDOWN,
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
    };
#if BOARD_MOTOR_OUTPUT_LAYOUT == MOTOR_OUTPUT_LAYOUT_TIM2_TIM3
    gpio.Pin = MOTOR_1_PIN | MOTOR_2_PIN; gpio.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin = MOTOR_3_PIN | MOTOR_4_PIN; gpio.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &gpio);
#else
    gpio.Pin = MOTOR_1_PIN; gpio.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(MOTOR_1_PORT, &gpio);
    gpio.Pin = MOTOR_2_PIN; gpio.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(MOTOR_2_PORT, &gpio);
    gpio.Pin = MOTOR_3_PIN | MOTOR_4_PIN; gpio.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &gpio);
#endif

    const uint32_t psc = timer_clock_hz() / 12000000U - 1U;
#if BOARD_MOTOR_OUTPUT_LAYOUT == MOTOR_OUTPUT_LAYOUT_TIM2_TIM3
    TIM3->PSC = psc; TIM3->ARR = 39U; TIM3->CCR3 = TIM3->CCR4 = 0U;
    TIM3->CCMR2 = (6U << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE |
                  (6U << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;
    TIM3->CCER = TIM_CCER_CC3E | TIM_CCER_CC4E; TIM3->EGR = TIM_EGR_UG;
    TIM3->DIER = 0U;
    TIM3->CR1 = TIM_CR1_ARPE;

    TIM2->PSC = psc; TIM2->ARR = 39U; TIM2->CCR3 = TIM2->CCR4 = 0U;
    TIM2->CCMR2 = (6U << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE |
                  (6U << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;
    TIM2->CCER = TIM_CCER_CC3E | TIM_CCER_CC4E; TIM2->EGR = TIM_EGR_UG;
    TIM2->DIER = 0U;
    TIM2->CR1 = TIM_CR1_ARPE;

    dma_stream_setup(DMA1_Stream7, DMA_CHANNEL_5, &TIM3->CCR3);
    dma_stream_setup(DMA1_Stream2, DMA_CHANNEL_5, &TIM3->CCR4);
    dma_stream_setup(DMA1_Stream6, DMA_CHANNEL_3, &TIM2->CCR4);
    dma_stream_setup(DMA1_Stream1, DMA_CHANNEL_3, &TIM2->CCR3);
#else
    TIM2->PSC = psc; TIM2->ARR = 39U; TIM2->CCR2 = 0U;
    TIM2->CCMR1 = (6U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    TIM2->CCER = TIM_CCER_CC2E; TIM2->EGR = TIM_EGR_UG;
    TIM2->DIER = 0U; TIM2->CR1 = TIM_CR1_ARPE;

    TIM3->PSC = psc; TIM3->ARR = 39U; TIM3->CCR1 = 0U;
    TIM3->CCMR1 = (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
    TIM3->CCER = TIM_CCER_CC1E; TIM3->EGR = TIM_EGR_UG;
    TIM3->DIER = 0U; TIM3->CR1 = TIM_CR1_ARPE;

    TIM4->PSC = psc; TIM4->ARR = 39U; TIM4->CCR1 = TIM4->CCR2 = 0U;
    TIM4->CCMR1 = (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE |
                  (6U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    TIM4->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E; TIM4->EGR = TIM_EGR_UG;
    TIM4->DIER = 0U;
    TIM4->CR1 = TIM_CR1_ARPE;

    dma_stream_setup(DMA1_Stream6, DMA_CHANNEL_3, &TIM2->CCR2);
    dma_stream_setup(DMA1_Stream4, DMA_CHANNEL_5, &TIM3->CCR1);
    dma_stream_setup(DMA1_Stream0, DMA_CHANNEL_2, &TIM4->CCR1);
    dma_stream_setup(DMA1_Stream3, DMA_CHANNEL_2, &TIM4->CCR2);
#endif
}

static void dma_prepare(DMA_Stream_TypeDef *stream, uint32_t *data)
{
    stream->CR &= ~DMA_SxCR_EN;
    while ((stream->CR & DMA_SxCR_EN) != 0U) {
    }
    stream->M0AR = (uint32_t)data;
    stream->NDTR = DSHOT_FRAME_WORDS;
}

static void dshot_timers_stop(void)
{
#if BOARD_MOTOR_OUTPUT_LAYOUT == MOTOR_OUTPUT_LAYOUT_TIM2_TIM3
    TIM3->DIER &= ~(TIM_DIER_CC3DE | TIM_DIER_CC4DE);
    TIM2->DIER &= ~(TIM_DIER_CC3DE | TIM_DIER_CC4DE);
    TIM3->CR1 &= ~TIM_CR1_CEN;
    TIM2->CR1 &= ~TIM_CR1_CEN;
#else
    TIM2->DIER &= ~TIM_DIER_CC2DE;
    TIM3->DIER &= ~TIM_DIER_CC1DE;
    TIM4->DIER &= ~(TIM_DIER_CC1DE | TIM_DIER_CC2DE);
    TIM2->CR1 &= ~TIM_CR1_CEN;
    TIM3->CR1 &= ~TIM_CR1_CEN;
    TIM4->CR1 &= ~TIM_CR1_CEN;
#endif
}

static void dshot_dma_flags_clear(void)
{
#if BOARD_MOTOR_OUTPUT_LAYOUT == MOTOR_OUTPUT_LAYOUT_TIM2_TIM3
    DMA1->LIFCR = DMA_LIFCR_CFEIF1 | DMA_LIFCR_CDMEIF1 |
                  DMA_LIFCR_CTEIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTCIF1 |
                  DMA_LIFCR_CFEIF2 | DMA_LIFCR_CDMEIF2 |
                  DMA_LIFCR_CTEIF2 | DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTCIF2;
    DMA1->HIFCR = DMA_HIFCR_CFEIF6 | DMA_HIFCR_CDMEIF6 |
                  DMA_HIFCR_CTEIF6 | DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTCIF6 |
                  DMA_HIFCR_CFEIF7 | DMA_HIFCR_CDMEIF7 |
                  DMA_HIFCR_CTEIF7 | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTCIF7;
#else
    DMA1->LIFCR = DMA_LIFCR_CFEIF0 | DMA_LIFCR_CDMEIF0 |
                  DMA_LIFCR_CTEIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTCIF0 |
                  DMA_LIFCR_CFEIF3 | DMA_LIFCR_CDMEIF3 |
                  DMA_LIFCR_CTEIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTCIF3;
    DMA1->HIFCR = DMA_HIFCR_CFEIF4 | DMA_HIFCR_CDMEIF4 |
                  DMA_HIFCR_CTEIF4 | DMA_HIFCR_CHTIF4 | DMA_HIFCR_CTCIF4 |
                  DMA_HIFCR_CFEIF6 | DMA_HIFCR_CDMEIF6 |
                  DMA_HIFCR_CTEIF6 | DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTCIF6;
#endif
}

static void dshot_timers_start(void)
{
#if BOARD_MOTOR_OUTPUT_LAYOUT == MOTOR_OUTPUT_LAYOUT_TIM2_TIM3
    TIM3->CNT = 0U;
    TIM2->CNT = 0U;
    TIM3->SR = 0U;
    TIM2->SR = 0U;
    TIM3->DIER |= TIM_DIER_CC3DE | TIM_DIER_CC4DE;
    TIM2->DIER |= TIM_DIER_CC3DE | TIM_DIER_CC4DE;
    __DMB();
    TIM3->CR1 |= TIM_CR1_CEN;
    TIM2->CR1 |= TIM_CR1_CEN;
#else
    TIM2->CNT = 0U;
    TIM3->CNT = 0U;
    TIM4->CNT = 0U;
    TIM2->SR = 0U;
    TIM3->SR = 0U;
    TIM4->SR = 0U;
    TIM2->DIER |= TIM_DIER_CC2DE;
    TIM3->DIER |= TIM_DIER_CC1DE;
    TIM4->DIER |= TIM_DIER_CC1DE | TIM_DIER_CC2DE;
    __DMB();
    TIM2->CR1 |= TIM_CR1_CEN;
    TIM3->CR1 |= TIM_CR1_CEN;
    TIM4->CR1 |= TIM_CR1_CEN;
#endif
}

static uint16_t packet(uint16_t value)
{
    const uint16_t payload = value << 1U;
    uint16_t checksum_data = payload;
    uint16_t checksum = 0;
    for (uint8_t i = 0; i < 3U; ++i) {
        checksum ^= checksum_data;
        checksum_data >>= 4U;
    }
    return (payload << 4U) | (checksum & 0x0FU);
}

static void oneshot125_init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
#if BOARD_MOTOR_OUTPUT_LAYOUT == MOTOR_OUTPUT_LAYOUT_TIM2_TIM3_TIM4
    __HAL_RCC_TIM4_CLK_ENABLE();
#endif
    GPIO_InitTypeDef gpio = {
        .Mode = GPIO_MODE_AF_PP, .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
    };
#if BOARD_MOTOR_OUTPUT_LAYOUT == MOTOR_OUTPUT_LAYOUT_TIM2_TIM3
    gpio.Pin = MOTOR_1_PIN | MOTOR_2_PIN; gpio.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin = MOTOR_3_PIN | MOTOR_4_PIN; gpio.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &gpio);
#else
    gpio.Pin = MOTOR_1_PIN; gpio.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(MOTOR_1_PORT, &gpio);
    gpio.Pin = MOTOR_2_PIN; gpio.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(MOTOR_2_PORT, &gpio);
    gpio.Pin = MOTOR_3_PIN | MOTOR_4_PIN; gpio.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &gpio);
#endif

    const uint32_t psc = timer_clock_hz() / 1000000U - 1U;
#if BOARD_MOTOR_OUTPUT_LAYOUT == MOTOR_OUTPUT_LAYOUT_TIM2_TIM3
    TIM3->PSC = psc; TIM3->ARR = 499U; TIM3->CCR3 = TIM3->CCR4 = 125U;
    TIM3->CCMR2 = (6U << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE |
                  (6U << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;
    TIM3->CCER = TIM_CCER_CC3E | TIM_CCER_CC4E; TIM3->EGR = TIM_EGR_UG;
    TIM3->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
    TIM2->PSC = psc; TIM2->ARR = 499U; TIM2->CCR3 = TIM2->CCR4 = 125U;
    TIM2->CCMR2 = (6U << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE |
                  (6U << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;
    TIM2->CCER = TIM_CCER_CC3E | TIM_CCER_CC4E; TIM2->EGR = TIM_EGR_UG;
    TIM2->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
#else
    TIM2->PSC = psc; TIM2->ARR = 499U; TIM2->CCR2 = 125U;
    TIM2->CCMR1 = (6U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    TIM2->CCER = TIM_CCER_CC2E; TIM2->EGR = TIM_EGR_UG;
    TIM2->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
    TIM3->PSC = psc; TIM3->ARR = 499U; TIM3->CCR1 = 125U;
    TIM3->CCMR1 = (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
    TIM3->CCER = TIM_CCER_CC1E; TIM3->EGR = TIM_EGR_UG;
    TIM3->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
    TIM4->PSC = psc; TIM4->ARR = 499U; TIM4->CCR1 = TIM4->CCR2 = 125U;
    TIM4->CCMR1 = (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE |
                  (6U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    TIM4->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E; TIM4->EGR = TIM_EGR_UG;
    TIM4->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
#endif
}

static void multishot_init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
#if BOARD_MOTOR_OUTPUT_LAYOUT == MOTOR_OUTPUT_LAYOUT_TIM2_TIM3_TIM4
    __HAL_RCC_TIM4_CLK_ENABLE();
#endif
    GPIO_InitTypeDef gpio = {
        .Mode = GPIO_MODE_AF_PP, .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
    };
#if BOARD_MOTOR_OUTPUT_LAYOUT == MOTOR_OUTPUT_LAYOUT_TIM2_TIM3
    gpio.Pin = MOTOR_1_PIN | MOTOR_2_PIN; gpio.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin = MOTOR_3_PIN | MOTOR_4_PIN; gpio.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &gpio);
#else
    gpio.Pin = MOTOR_1_PIN; gpio.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(MOTOR_1_PORT, &gpio);
    gpio.Pin = MOTOR_2_PIN; gpio.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(MOTOR_2_PORT, &gpio);
    gpio.Pin = MOTOR_3_PIN | MOTOR_4_PIN; gpio.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &gpio);
#endif

    const uint32_t psc = timer_clock_hz() / 12000000U - 1U;
#if BOARD_MOTOR_OUTPUT_LAYOUT == MOTOR_OUTPUT_LAYOUT_TIM2_TIM3
    TIM3->PSC = psc; TIM3->ARR = 1499U; TIM3->CCR3 = TIM3->CCR4 = 60U;
    TIM3->CCMR2 = (6U << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE |
                  (6U << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;
    TIM3->CCER = TIM_CCER_CC3E | TIM_CCER_CC4E; TIM3->EGR = TIM_EGR_UG;
    TIM3->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
    TIM2->PSC = psc; TIM2->ARR = 1499U; TIM2->CCR3 = TIM2->CCR4 = 60U;
    TIM2->CCMR2 = (6U << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE |
                  (6U << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;
    TIM2->CCER = TIM_CCER_CC3E | TIM_CCER_CC4E; TIM2->EGR = TIM_EGR_UG;
    TIM2->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
#else
    TIM2->PSC = psc; TIM2->ARR = 1499U; TIM2->CCR2 = 60U;
    TIM2->CCMR1 = (6U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    TIM2->CCER = TIM_CCER_CC2E; TIM2->EGR = TIM_EGR_UG;
    TIM2->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
    TIM3->PSC = psc; TIM3->ARR = 1499U; TIM3->CCR1 = 60U;
    TIM3->CCMR1 = (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
    TIM3->CCER = TIM_CCER_CC1E; TIM3->EGR = TIM_EGR_UG;
    TIM3->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
    TIM4->PSC = psc; TIM4->ARR = 1499U; TIM4->CCR1 = TIM4->CCR2 = 60U;
    TIM4->CCMR1 = (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE |
                  (6U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    TIM4->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E; TIM4->EGR = TIM_EGR_UG;
    TIM4->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
#endif
}

void dshot_init(void)
{
    hardware_dshot_init();
}

void dshot_write(const uint16_t values[4])
{
    if (active_protocol == MOTOR_PROTOCOL_DSHOT300) {
        /*
         * Non interrompere un frame ancora in corso. Un loop anticipato o
         * ravvicinato allungherebbe un impulso e potrebbe trasformare lo STOP
         * in un comando DShot valido. Betaflight lascia terminare il DMA e
         * mantiene l'uscita bassa prima di preparare il frame successivo.
         */
        if (dshot_dma_transfer_active()) {
            return;
        }
        for (uint8_t motor = 0U; motor < 4U; ++motor) {
            const uint16_t frame = packet(
                dshot_sanitize_value(values[motor]));
            for (uint8_t bit = 0U; bit < 16U; ++bit) {
                dshot_dma_buffer[motor][bit] =
                    (frame & (1U << (15U - bit))) != 0U ? 30U : 15U;
            }
            dshot_dma_buffer[motor][16] = 0U;
            dshot_dma_buffer[motor][17] = 0U;
        }
        /*
         * Stage every channel while the timer DMA requests are disabled.
         * Starting free-running channels one by one can lose the first compare
         * event on a channel that happens to be at the end of its period.
         */
        dshot_timers_stop();
        for (uint8_t motor = 0U; motor < 4U; ++motor) {
            dma_prepare(dshot_dma_streams[motor], dshot_dma_buffer[motor]);
        }
        dshot_dma_flags_clear();
        __DMB();
        for (uint8_t motor = 0U; motor < 4U; ++motor) {
            dshot_dma_streams[motor]->CR |= DMA_SxCR_EN;
        }
        __DMB();
        dshot_timers_start();
        return;
    }
    if (active_protocol == MOTOR_PROTOCOL_ONESHOT125) {
        uint16_t pulse[4];
        for (uint8_t i = 0U; i < 4U; ++i) {
            float percent = values[i] == 0U ? 0.0f :
                (float)(values[i] - DSHOT_MIN) /
                (float)(DSHOT_MAX - DSHOT_MIN);
            if (percent < 0.0f) percent = 0.0f;
            if (percent > 1.0f) percent = 1.0f;
            pulse[i] = 125U + (uint16_t)(percent * 125.0f);
        }
#if BOARD_MOTOR_OUTPUT_LAYOUT == MOTOR_OUTPUT_LAYOUT_TIM2_TIM3
        TIM3->CCR3 = pulse[0];
        TIM3->CCR4 = pulse[1];
        TIM2->CCR4 = pulse[2];
        TIM2->CCR3 = pulse[3];
#else
        TIM2->CCR2 = pulse[0];
        TIM3->CCR1 = pulse[1];
        TIM4->CCR1 = pulse[2];
        TIM4->CCR2 = pulse[3];
#endif
        return;
    }
    if (active_protocol == MOTOR_PROTOCOL_MULTISHOT) {
        uint16_t pulse[4];
        for (uint8_t i = 0U; i < 4U; ++i) {
            float percent = values[i] == 0U ? 0.0f :
                (float)(values[i] - DSHOT_MIN) /
                (float)(DSHOT_MAX - DSHOT_MIN);
            if (percent < 0.0f) percent = 0.0f;
            if (percent > 1.0f) percent = 1.0f;
            pulse[i] = 60U + (uint16_t)(percent * 240.0f);
        }
#if BOARD_MOTOR_OUTPUT_LAYOUT == MOTOR_OUTPUT_LAYOUT_TIM2_TIM3
        TIM3->CCR3 = pulse[0];
        TIM3->CCR4 = pulse[1];
        TIM2->CCR4 = pulse[2];
        TIM2->CCR3 = pulse[3];
#else
        TIM2->CCR2 = pulse[0];
        TIM3->CCR1 = pulse[1];
        TIM4->CCR1 = pulse[2];
        TIM4->CCR2 = pulse[3];
#endif
        return;
    }

}

void dshot_startup_sequence(void)
{
    if (active_protocol != MOTOR_PROTOCOL_DSHOT300) {
        return;
    }

    static const uint16_t stopped[4] = {0U, 0U, 0U, 0U};
    const uint32_t started_ms = HAL_GetTick();
    do {
        dshot_write(stopped);
        HAL_Delay(DSHOT_STARTUP_FRAME_INTERVAL_MS);
    } while ((uint32_t)(HAL_GetTick() - started_ms) < DSHOT_STARTUP_TIME_MS);
}

bool motor_protocol_set(motor_protocol_t protocol)
{
    if (protocol != MOTOR_PROTOCOL_DSHOT300 &&
        protocol != MOTOR_PROTOCOL_ONESHOT125 &&
        protocol != MOTOR_PROTOCOL_MULTISHOT) {
        return false;
    }
    if (protocol == active_protocol) {
        return true;
    }
    TIM3->CR1 = 0U;
    TIM2->CR1 = 0U;
#if BOARD_MOTOR_OUTPUT_LAYOUT == MOTOR_OUTPUT_LAYOUT_TIM2_TIM3_TIM4
    TIM4->CR1 = 0U;
#endif
    active_protocol = protocol;
    if (protocol == MOTOR_PROTOCOL_ONESHOT125) {
        oneshot125_init();
    } else if (protocol == MOTOR_PROTOCOL_MULTISHOT) {
        multishot_init();
    } else {
        hardware_dshot_init();
    }
    return true;
}

motor_protocol_t motor_protocol_get(void)
{
    return active_protocol;
}

const char *motor_protocol_name(motor_protocol_t protocol)
{
    if (protocol == MOTOR_PROTOCOL_ONESHOT125) return "ONESHOT125";
    if (protocol == MOTOR_PROTOCOL_MULTISHOT) return "MULTISHOT";
    return "DSHOT300";
}

uint16_t dshot_from_percent(float percent)
{
    if (percent <= 0.0f) return 0U;
    if (percent > 100.0f) percent = 100.0f;
    return DSHOT_MIN + (uint16_t)(percent * (float)(DSHOT_MAX - DSHOT_MIN) / 100.0f);
}
