#include "blackbox_sd.h"

#include <string.h>

#include "board.h"

#define SD_BLOCK_SIZE 512U
#define SD_QUEUE_BLOCKS 8U
#define SD_RECORDS_PER_BLOCK 12U
#define SD_DATA_OFFSET_SECTORS 2048U
#define SD_BLOCK_MAGIC 0x42423446U /* F4BB */

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t flight_id;
    uint32_t sequence;
    uint32_t timestamp_us;
    uint16_t record_count;
    uint8_t stop_flag;
    uint8_t version;
    uint32_t checksum;
    uint8_t reserved[8];
    flight_log_record_t records[SD_RECORDS_PER_BLOCK];
} blackbox_block_t;

_Static_assert(sizeof(blackbox_block_t) == SD_BLOCK_SIZE,
               "blackbox SD block must be exactly one sector");

static blackbox_sd_state_t state;
static bool enabled;
static bool high_capacity;
static uint32_t sector_count;
static uint32_t next_sector;
static uint32_t flight_id;
static uint32_t sequence;
static uint32_t written_bytes;
static uint32_t dropped_records;
static blackbox_block_t queue[SD_QUEUE_BLOCKS];
static uint8_t queue_read;
static uint8_t queue_write;
static uint8_t queue_count;
static blackbox_block_t current;
typedef enum { WRITE_IDLE, WRITE_DMA, WRITE_RESPONSE, WRITE_BUSY } write_state_t;
static volatile bool dma_complete;
static write_state_t write_state;
static uint32_t write_deadline_us;

static void select_card(bool selected)
{
#if BOARD_HAS_SDCARD
    HAL_GPIO_WritePin(SDCARD_CS_PORT, SDCARD_CS_PIN,
                      selected ? GPIO_PIN_RESET : GPIO_PIN_SET);
#else
    (void)selected;
#endif
}

static uint8_t transfer(uint8_t value)
{
#if BOARD_HAS_SDCARD
    uint8_t received = 0xFFU;
    if (HAL_SPI_TransmitReceive(&SDCARD_SPI_HANDLE, &value, &received, 1U,
                                2U) != HAL_OK) return 0U;
    return received;
#else
    (void)value;
    return 0U;
#endif
}

static uint8_t command(uint8_t cmd, uint32_t arg, uint8_t crc)
{
    transfer(0xFFU);
    transfer((uint8_t)(0x40U | cmd));
    transfer((uint8_t)(arg >> 24)); transfer((uint8_t)(arg >> 16));
    transfer((uint8_t)(arg >> 8)); transfer((uint8_t)arg); transfer(crc);
    for (uint8_t i = 0U; i < 10U; ++i) {
        const uint8_t response = transfer(0xFFU);
        if ((response & 0x80U) == 0U) return response;
    }
    return 0xFFU;
}

static bool card_present(void)
{
#if BOARD_HAS_SDCARD
    return HAL_GPIO_ReadPin(SDCARD_DETECT_PORT, SDCARD_DETECT_PIN) ==
           SDCARD_DETECT_PRESENT_LEVEL;
#else
    return false;
#endif
}

static bool initialise_card(void)
{
#if !BOARD_HAS_SDCARD
    return false;
#else
    select_card(false);
    for (uint8_t i = 0U; i < 10U; ++i) transfer(0xFFU);
    select_card(true);
    if (command(0U, 0U, 0x95U) != 1U) { select_card(false); return false; }
    const uint8_t r7 = command(8U, 0x1AAU, 0x87U);
    high_capacity = r7 == 1U;
    if (high_capacity) for (uint8_t i = 0U; i < 4U; ++i) transfer(0xFFU);
    const uint32_t deadline = board_micros() + 1000000U;
    uint8_t response = 1U;
    while (response != 0U && (int32_t)(board_micros() - deadline) < 0) {
        (void)command(55U, 0U, 0x01U);
        response = command(41U, high_capacity ? 0x40000000U : 0U, 0x01U);
    }
    if (response != 0U) { select_card(false); return false; }
    if (high_capacity) {
        if (command(58U, 0U, 0x01U) != 0U) { select_card(false); return false; }
        const uint8_t ocr0 = transfer(0xFFU);
        high_capacity = (ocr0 & 0x40U) != 0U;
        for (uint8_t i = 0U; i < 3U; ++i) transfer(0xFFU);
    } else if (command(16U, SD_BLOCK_SIZE, 0x01U) != 0U) {
        select_card(false); return false;
    }
    if (command(9U, 0U, 0x01U) != 0U) { select_card(false); return false; }
    uint8_t token = 0xFFU;
    for (uint16_t i = 0U; i < 1000U && token == 0xFFU; ++i) token = transfer(0xFFU);
    if (token != 0xFEU) { select_card(false); return false; }
    uint8_t csd[16];
    for (uint8_t i = 0U; i < sizeof(csd); ++i) csd[i] = transfer(0xFFU);
    transfer(0xFFU); transfer(0xFFU); select_card(false); transfer(0xFFU);
    if ((csd[0] >> 6) == 1U) {
        const uint32_t csize = ((uint32_t)(csd[7] & 0x3FU) << 16) |
                               ((uint32_t)csd[8] << 8) | csd[9];
        sector_count = (csize + 1U) * 1024U;
    } else {
        const uint32_t read_bl_len = csd[5] & 0x0FU;
        const uint32_t csize = ((uint32_t)(csd[6] & 3U) << 10) |
                               ((uint32_t)csd[7] << 2) | (csd[8] >> 6);
        const uint32_t mult = ((uint32_t)(csd[9] & 3U) << 1) | (csd[10] >> 7);
        sector_count = (csize + 1U) << (mult + read_bl_len - 7U);
    }
    SDCARD_SPI_HANDLE.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    return HAL_SPI_Init(&SDCARD_SPI_HANDLE) == HAL_OK &&
           sector_count > SD_DATA_OFFSET_SECTORS + 1024U;
#endif
}

static uint32_t checksum_block(const blackbox_block_t *block)
{
    const uint8_t *bytes = (const uint8_t *)block;
    uint32_t hash = 2166136261U;
    for (uint32_t i = 0U; i < sizeof(*block); ++i) {
        if (i >= 20U && i < 24U) continue;
        hash = (hash ^ bytes[i]) * 16777619U;
    }
    return hash;
}

static bool begin_write_sector(uint32_t sector, const void *data)
{
#if !BOARD_HAS_SDCARD
    (void)sector; (void)data; return false;
#else
    select_card(true);
    const uint32_t address = high_capacity ? sector : sector * SD_BLOCK_SIZE;
    if (command(24U, address, 0x01U) != 0U) { select_card(false); return false; }
    transfer(0xFFU); transfer(0xFEU);
    dma_complete = false;
    if (HAL_SPI_Transmit_DMA(&SDCARD_SPI_HANDLE, (uint8_t *)data,
                             SD_BLOCK_SIZE) != HAL_OK) {
        select_card(false); return false;
    }
    write_state = WRITE_DMA;
    write_deadline_us = board_micros() + 250000U;
    return true;
#endif
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *spi)
{
#if BOARD_HAS_SDCARD
    if (spi->Instance == SDCARD_SPI_HANDLE.Instance) dma_complete = true;
#else
    (void)spi;
#endif
}

static void queue_current(void)
{
    if (current.record_count == 0U) return;
    current.checksum = checksum_block(&current);
    if (queue_count >= SD_QUEUE_BLOCKS) {
        dropped_records += current.record_count;
    } else {
        queue[queue_write] = current;
        queue_write = (uint8_t)((queue_write + 1U) % SD_QUEUE_BLOCKS);
        ++queue_count;
    }
    memset(&current, 0, sizeof(current));
}

void blackbox_sd_init(void)
{
    state = BOARD_HAS_SDCARD ? BLACKBOX_SD_ABSENT : BLACKBOX_SD_UNSUPPORTED;
    enabled = false; sector_count = 0U; next_sector = SD_DATA_OFFSET_SECTORS;
    flight_id = 0U; sequence = 0U; written_bytes = 0U;
    dropped_records = 0U; queue_read = queue_write = queue_count = 0U;
    write_state = WRITE_IDLE; dma_complete = false;
    memset(&current, 0, sizeof(current));
    if (card_present()) {
        state = BLACKBOX_SD_INITIALIZING;
        state = initialise_card() ? BLACKBOX_SD_READY : BLACKBOX_SD_ERROR;
    }
}

void blackbox_sd_probe(void)
{
#if BOARD_HAS_SDCARD
    if (state == BLACKBOX_SD_RECORDING || write_state != WRITE_IDLE) return;
    if (!card_present()) {
        state = BLACKBOX_SD_ABSENT;
        enabled = false;
        sector_count = 0U;
        return;
    }
    state = BLACKBOX_SD_INITIALIZING;
    sector_count = 0U;
    state = initialise_card() ? BLACKBOX_SD_READY : BLACKBOX_SD_ERROR;
#endif
}

void blackbox_sd_update(void)
{
#if !BOARD_HAS_SDCARD
    return;
#else
    if (!card_present()) { state = BLACKBOX_SD_ABSENT; enabled = false; return; }
    if (state != BLACKBOX_SD_READY && state != BLACKBOX_SD_RECORDING) return;
    if (write_state == WRITE_IDLE) {
        if (queue_count > 0U &&
            !begin_write_sector(next_sector, &queue[queue_read])) {
            state = BLACKBOX_SD_ERROR; enabled = false;
        }
        return;
    }
    if ((int32_t)(board_micros() - write_deadline_us) >= 0) {
        HAL_SPI_Abort(&SDCARD_SPI_HANDLE); select_card(false);
        write_state = WRITE_IDLE; state = BLACKBOX_SD_ERROR; enabled = false;
        return;
    }
    if (write_state == WRITE_DMA && dma_complete) {
        dma_complete = false;
        __HAL_SPI_CLEAR_OVRFLAG(&SDCARD_SPI_HANDLE);
        transfer(0xFFU); transfer(0xFFU);
        write_state = WRITE_RESPONSE;
    }
    if (write_state == WRITE_RESPONSE) {
        const uint8_t response = transfer(0xFFU);
        if (response == 0xFFU) return;
        if ((response & 0x1FU) != 0x05U) {
            select_card(false); write_state = WRITE_IDLE;
            state = BLACKBOX_SD_ERROR; enabled = false; return;
        }
        write_state = WRITE_BUSY;
    }
    if (write_state == WRITE_BUSY && transfer(0xFFU) != 0U) {
        select_card(false); transfer(0xFFU); write_state = WRITE_IDLE;
        queue_read = (uint8_t)((queue_read + 1U) % SD_QUEUE_BLOCKS);
        --queue_count; ++next_sector; written_bytes += SD_BLOCK_SIZE;
        if (next_sector >= sector_count) next_sector = SD_DATA_OFFSET_SECTORS;
    }
#endif
}

void blackbox_sd_set_enabled(bool value) { enabled = value && state == BLACKBOX_SD_READY; }
bool blackbox_sd_is_enabled(void) { return enabled; }
blackbox_sd_state_t blackbox_sd_state(void) { return state; }
uint32_t blackbox_sd_capacity_mb(void) { return sector_count / 2048U; }
uint32_t blackbox_sd_written_bytes(void) { return written_bytes; }
uint32_t blackbox_sd_dropped_records(void) { return dropped_records; }

const char *blackbox_sd_state_name(void)
{
    static const char *const names[] = {"UNSUPPORTED","ABSENT","INITIALIZING","READY","RECORDING","ERROR"};
    return names[(unsigned)state <= BLACKBOX_SD_ERROR ? state : BLACKBOX_SD_ERROR];
}

void blackbox_sd_start(void)
{
    if (!enabled || state != BLACKBOX_SD_READY) return;
    ++flight_id; sequence = 0U; state = BLACKBOX_SD_RECORDING;
}

void blackbox_sd_append(const flight_log_record_t *record)
{
    if (state != BLACKBOX_SD_RECORDING || record == NULL) return;
    if (current.record_count == 0U) {
        current.magic = SD_BLOCK_MAGIC; current.flight_id = flight_id;
        current.sequence = sequence++; current.timestamp_us = board_micros();
        current.version = 1U;
    }
    current.records[current.record_count++] = *record;
    if (current.record_count >= SD_RECORDS_PER_BLOCK) queue_current();
}

void blackbox_sd_stop(uint8_t stop_flag, bool retain)
{
    if (state != BLACKBOX_SD_RECORDING) return;
    current.stop_flag = stop_flag;
    if (retain) queue_current();
    else { memset(&current, 0, sizeof(current)); queue_read = queue_write = queue_count = 0U; }
    state = BLACKBOX_SD_READY;
}
