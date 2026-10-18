/* SEQURE H743 diagnostic: continuous ADC3 scan, rank 1 VBAT / rank 2 current.
 * DMA1 stream 1 is separate from motor stream 0 and SPI stream 4.
 * The buffer is in AXI SRAM (.bss), never in DMA-inaccessible DTCM.
 */
#define SEQURE_ADC_DMA_WORDS 256U
static DMA_HandleTypeDef hdma_battery;
static uint16_t battery_dma_buffer[SEQURE_ADC_DMA_WORDS] __attribute__((aligned(32)));
static volatile uint32_t dma_voltage_sum, dma_current_sum, dma_samples, dma_last_us;
static volatile uint32_t battery_adc_errors;
static float battery_voltage_filtered;
static float battery_voltage_multiplier = 1.0f;
static float battery_current_filtered;
static uint32_t current_adc_raw, current_adc_count, current_adc_last_us;

static void sequre_adc_dma_start(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();
    hdma_battery.Instance = DMA1_Stream1;
    hdma_battery.Init.Request = DMA_REQUEST_ADC3;
    hdma_battery.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_battery.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_battery.Init.MemInc = DMA_MINC_ENABLE;
    hdma_battery.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_battery.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_battery.Init.Mode = DMA_CIRCULAR;
    hdma_battery.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_battery.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_battery) != HAL_OK) board_fatal_error();
    __HAL_LINKDMA(&hadc_battery, DMA_Handle, hdma_battery);
    /* Each half is cache-line aligned and exclusively owned by the CPU
     * during its callback. Drop dirty startup lines before DMA writes. */
    SCB_CleanInvalidateDCache_by_Addr((uint32_t *)battery_dma_buffer, sizeof battery_dma_buffer);
    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
    if (HAL_ADC_Start_DMA(&hadc_battery, (uint32_t *)battery_dma_buffer,
                          SEQURE_ADC_DMA_WORDS) != HAL_OK) board_fatal_error();
}

void DMA1_Stream1_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_battery);
}

/* BEGIN DMA PROCESSING (also compiled by the host regression). */
static void sequre_adc_dma_block(uint32_t offset)
{
    uint16_t *block = &battery_dma_buffer[offset];
    const uint32_t words = SEQURE_ADC_DMA_WORDS / 2U;
    SCB_InvalidateDCache_by_Addr((uint32_t *)block, words * sizeof(uint16_t));
    uint32_t voltage = 0U, current = 0U;
    for (uint32_t i = 0U; i < words; i += 2U) {
        voltage += block[i];
        current += block[i + 1U];
    }
    dma_voltage_sum = voltage;
    dma_current_sum = current;
    dma_samples += words / 2U;
    /* SysTick can be pending at this interrupt priority. Use its monotonic
     * millisecond count rather than combining it with the running counter. */
    dma_last_us = HAL_GetTick() * 1000U;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *adc)
{
    if (adc == &hadc_battery) sequre_adc_dma_block(0U);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *adc)
{
    if (adc == &hadc_battery) sequre_adc_dma_block(SEQURE_ADC_DMA_WORDS / 2U);
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *adc)
{
    if (adc == &hadc_battery) ++battery_adc_errors;
}

void board_battery_update(void)
{
    /* Copy one coherent completed-block snapshot, without holding up DMA. */
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    const uint32_t samples = dma_samples;
    const uint32_t voltage = dma_voltage_sum, current = dma_current_sum;
    const uint32_t sampled_us = dma_last_us;
    __set_PRIMASK(mask);
    if (samples == current_adc_count) return;
    const bool first = current_adc_count == 0U;
    const float elapsed_s = (float)(sampled_us - current_adc_last_us) * 0.000001f;
    const float alpha = first ? 1.0f : elapsed_s / (0.100f + elapsed_s);
    const float pairs = (float)(SEQURE_ADC_DMA_WORDS / 4U);
    const float raw_current = (float)current / pairs;
    const float measured_current = raw_current * 3300.0f / 4095.0f * 10.0f / CURRENT_METER_SCALE;
    battery_current_filtered += alpha * (measured_current - battery_current_filtered);
    const float measured_voltage = (float)voltage / pairs * 3.3f / 4095.0f *
                                   BATTERY_VOLTAGE_DIVIDER * battery_voltage_multiplier;
    /* Eight voltage samples at 20 Hz with alpha 0.15 gave about 2.5 seconds. */
    const float voltage_alpha = first ? 1.0f : elapsed_s / (2.5f + elapsed_s);
    battery_voltage_filtered += voltage_alpha * (measured_voltage - battery_voltage_filtered);
    current_adc_raw = (uint32_t)(raw_current + 0.5f);
    current_adc_count = samples;
    current_adc_last_us = sampled_us;
}
/* END DMA PROCESSING */
