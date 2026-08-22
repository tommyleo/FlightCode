#include "blackbox_sd.h"

#include <stddef.h>
#include <string.h>

#include "board.h"

#if !BOARD_HAS_SDCARD && !BOARD_HAS_DATAFLASH

void blackbox_sd_init(void) {}
void blackbox_sd_probe(void) {}
void blackbox_sd_update(void) {}
void blackbox_sd_set_enabled(bool enabled) { (void)enabled; }
bool blackbox_sd_is_enabled(void) { return false; }
bool blackbox_sd_is_busy(void) { return false; }
blackbox_sd_state_t blackbox_sd_state(void) { return BLACKBOX_SD_UNSUPPORTED; }
const char *blackbox_sd_state_name(void) { return "UNSUPPORTED"; }
uint32_t blackbox_sd_capacity_mb(void) { return 0U; }
uint32_t blackbox_sd_written_bytes(void) { return 0U; }
uint32_t blackbox_sd_dropped_records(void) { return 0U; }
uint32_t blackbox_sd_total_bytes(void) { return 0U; }
uint32_t blackbox_sd_flight_count(void) { return 0U; }
bool blackbox_sd_get_flight(uint32_t index, blackbox_sd_flight_info_t *info)
{ (void)index; (void)info; return false; }
bool blackbox_sd_get_record(uint32_t flight_id, uint32_t record_index,
                            flight_log_record_t *record)
{ (void)flight_id; (void)record_index; (void)record; return false; }
bool blackbox_sd_get_metadata(uint32_t flight_id, flight_log_metadata_t *metadata)
{ (void)flight_id; (void)metadata; return false; }
bool blackbox_sd_clear(void) { return false; }
void blackbox_sd_start(const flight_log_metadata_t *metadata) { (void)metadata; }
void blackbox_sd_append(const flight_log_record_t *record) { (void)record; }
void blackbox_sd_stop(uint8_t stop_flag, bool retain)
{ (void)stop_flag; (void)retain; }
void blackbox_sd_get_diagnostics(blackbox_sd_diagnostics_t *diagnostics)
{ if (diagnostics != NULL) memset(diagnostics, 0, sizeof(*diagnostics)); }
void blackbox_sd_restore_diagnostics(
    const blackbox_sd_diagnostics_t *diagnostics) { (void)diagnostics; }
bool blackbox_sd_write_test(blackbox_sd_write_test_t *result)
{ if (result != NULL) memset(result, 0, sizeof(*result)); return false; }
bool blackbox_sd_session_test(void) { return false; }

#elif BOARD_HAS_SDCARD

#define SD_BLOCK_SIZE 512U
#define SD_QUEUE_BLOCKS 8U
#define SD_RECORDS_PER_BLOCK 12U
#define SD_DATA_OFFSET_SECTORS 2048U
#define SD_CATALOG_SECTOR (SD_DATA_OFFSET_SECTORS - 1U)
#define SD_BLOCK_MAGIC 0x42423446U /* F4BB */
#define SD_CATALOG_MAGIC 0x58494246U /* FBIX */
#define SD_CATALOG_VERSION 2U
#define SD_CATALOG_FLIGHTS 20U

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

typedef struct __attribute__((packed)) {
    uint32_t flight_id;
    uint32_t start_sector;
    uint32_t block_count;
    uint32_t record_count;
    uint8_t stop_flag;
    uint8_t reserved[3];
} blackbox_catalog_entry_t;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t version;
    uint32_t generation;
    uint32_t next_sector;
    uint32_t next_flight_id;
    uint32_t flight_count;
    blackbox_catalog_entry_t flights[SD_CATALOG_FLIGHTS];
    uint32_t checksum;
    uint8_t reserved[84];
} blackbox_catalog_t;

_Static_assert(sizeof(blackbox_catalog_t) == SD_BLOCK_SIZE,
               "blackbox catalog must be exactly one sector");

static blackbox_sd_state_t state;
static bool enabled;
static bool high_capacity;
static uint32_t sector_count;
static uint32_t next_sector;
static uint32_t flight_id;
static uint32_t sequence;
static uint32_t written_bytes;
static uint32_t dropped_records;
static blackbox_catalog_t catalog;
static uint32_t current_start_sector;
static uint32_t current_block_count;
static uint32_t current_record_count;
static bool catalog_commit_pending;
static bool writing_catalog;
static blackbox_block_t read_cache;
static uint32_t read_cache_sector;
static bool read_cache_valid;
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

static bool read_sector(uint32_t sector, void *data)
{
#if !BOARD_HAS_SDCARD
    (void)sector; (void)data; return false;
#else
    if (data == NULL || write_state != WRITE_IDLE) return false;
    select_card(true);
    const uint32_t address = high_capacity ? sector : sector * SD_BLOCK_SIZE;
    if (command(17U, address, 0x01U) != 0U) {
        select_card(false); transfer(0xFFU); return false;
    }
    uint8_t token = 0xFFU;
    const uint32_t deadline = board_micros() + 100000U;
    while (token == 0xFFU &&
           (int32_t)(board_micros() - deadline) < 0) {
        token = transfer(0xFFU);
    }
    if (token != 0xFEU) {
        select_card(false); transfer(0xFFU); return false;
    }
    uint8_t *const bytes = (uint8_t *)data;
    for (uint32_t i = 0U; i < SD_BLOCK_SIZE; ++i) {
        bytes[i] = transfer(0xFFU);
    }
    transfer(0xFFU); transfer(0xFFU);
    select_card(false); transfer(0xFFU);
    return true;
#endif
}

static uint32_t checksum_bytes(const void *data, uint32_t size,
                               uint32_t skip_offset)
{
    const uint8_t *const bytes = (const uint8_t *)data;
    uint32_t hash = 2166136261U;
    for (uint32_t i = 0U; i < size; ++i) {
        if (i >= skip_offset && i < skip_offset + sizeof(uint32_t)) continue;
        hash = (hash ^ bytes[i]) * 16777619U;
    }
    return hash;
}

static uint32_t checksum_block(const blackbox_block_t *block)
{
    return checksum_bytes(block, sizeof(*block),
                          (uint32_t)offsetof(blackbox_block_t, checksum));
}

static uint32_t checksum_catalog(const blackbox_catalog_t *value)
{
    return checksum_bytes(value, sizeof(*value),
                          (uint32_t)offsetof(blackbox_catalog_t, checksum));
}

static void reset_catalog(void)
{
    memset(&catalog, 0, sizeof(catalog));
    catalog.magic = SD_CATALOG_MAGIC;
    catalog.version = SD_CATALOG_VERSION;
    catalog.next_sector = SD_DATA_OFFSET_SECTORS;
    catalog.next_flight_id = 1U;
    next_sector = catalog.next_sector;
    flight_id = 0U;
}

static void load_catalog(void)
{
    blackbox_catalog_t stored;
    if (!read_sector(SD_CATALOG_SECTOR, &stored) ||
        stored.magic != SD_CATALOG_MAGIC ||
        stored.version != SD_CATALOG_VERSION ||
        stored.flight_count > SD_CATALOG_FLIGHTS ||
        stored.next_sector < SD_DATA_OFFSET_SECTORS ||
        stored.next_sector >= sector_count ||
        stored.next_flight_id == 0U ||
        stored.checksum != checksum_catalog(&stored)) {
        reset_catalog();
        return;
    }
    catalog = stored;
    next_sector = catalog.next_sector;
}

static void append_catalog_entry(uint8_t stop_flag)
{
    if (catalog.flight_count >= SD_CATALOG_FLIGHTS) {
        memmove(&catalog.flights[0], &catalog.flights[1],
                sizeof(catalog.flights[0]) * (SD_CATALOG_FLIGHTS - 1U));
        catalog.flight_count = SD_CATALOG_FLIGHTS - 1U;
    }
    blackbox_catalog_entry_t *const entry =
        &catalog.flights[catalog.flight_count++];
    memset(entry, 0, sizeof(*entry));
    entry->flight_id = flight_id;
    entry->start_sector = current_start_sector;
    entry->block_count = current_block_count;
    entry->record_count = current_record_count;
    entry->stop_flag = stop_flag;
    catalog.next_flight_id = flight_id + 1U;
    ++catalog.generation;
    catalog_commit_pending = true;
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
    const uint16_t records = current.record_count;
    current.checksum = checksum_block(&current);
    if (queue_count >= SD_QUEUE_BLOCKS) {
        dropped_records += records;
    } else {
        queue[queue_write] = current;
        queue_write = (uint8_t)((queue_write + 1U) % SD_QUEUE_BLOCKS);
        ++queue_count;
        ++current_block_count;
        current_record_count += records;
    }
    memset(&current, 0, sizeof(current));
}

static bool queue_metadata(const flight_log_metadata_t *metadata)
{
    if (metadata == NULL || sizeof(*metadata) > sizeof(current.records) ||
        queue_count >= SD_QUEUE_BLOCKS) return false;
    memset(&current, 0, sizeof(current));
    current.magic = SD_BLOCK_MAGIC;
    current.flight_id = flight_id;
    current.sequence = sequence++;
    current.timestamp_us = board_micros();
    current.version = 5U;
    memcpy(current.records, metadata, sizeof(*metadata));
    current.checksum = checksum_block(&current);
    queue[queue_write] = current;
    queue_write = (uint8_t)((queue_write + 1U) % SD_QUEUE_BLOCKS);
    ++queue_count;
    ++current_block_count;
    memset(&current, 0, sizeof(current));
    return true;
}

void blackbox_sd_init(void)
{
    state = BOARD_HAS_SDCARD ? BLACKBOX_SD_ABSENT : BLACKBOX_SD_UNSUPPORTED;
    enabled = false; sector_count = 0U; next_sector = SD_DATA_OFFSET_SECTORS;
    flight_id = 0U; sequence = 0U; written_bytes = 0U;
    dropped_records = 0U; queue_read = queue_write = queue_count = 0U;
    write_state = WRITE_IDLE; dma_complete = false;
    catalog_commit_pending = false; writing_catalog = false;
    read_cache_valid = false; read_cache_sector = 0U;
    current_start_sector = SD_DATA_OFFSET_SECTORS;
    current_block_count = current_record_count = 0U;
    reset_catalog();
    memset(&current, 0, sizeof(current));
    if (card_present()) {
        state = BLACKBOX_SD_INITIALIZING;
        if (initialise_card()) {
            load_catalog();
            state = BLACKBOX_SD_READY;
        } else {
            state = BLACKBOX_SD_ERROR;
        }
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
    queue_read = queue_write = queue_count = 0U;
    catalog_commit_pending = false;
    writing_catalog = false;
    read_cache_valid = false;
    memset(&current, 0, sizeof(current));
    if (initialise_card()) {
        load_catalog();
        state = BLACKBOX_SD_READY;
    } else {
        state = BLACKBOX_SD_ERROR;
    }
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
        if (queue_count > 0U) {
            if (next_sector >= sector_count ||
                !begin_write_sector(next_sector, &queue[queue_read])) {
                state = BLACKBOX_SD_ERROR; enabled = false;
            } else {
                writing_catalog = false;
            }
        } else if (catalog_commit_pending) {
            catalog.next_sector = next_sector;
            catalog.checksum = checksum_catalog(&catalog);
            if (!begin_write_sector(SD_CATALOG_SECTOR, &catalog)) {
                state = BLACKBOX_SD_ERROR; enabled = false;
            } else {
                writing_catalog = true;
            }
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
        if (writing_catalog) {
            writing_catalog = false;
            catalog_commit_pending = false;
        } else {
            queue_read = (uint8_t)((queue_read + 1U) % SD_QUEUE_BLOCKS);
            --queue_count; ++next_sector; written_bytes += SD_BLOCK_SIZE;
        }
    }
#endif
}

void blackbox_sd_set_enabled(bool value) { enabled = value && state == BLACKBOX_SD_READY; }
bool blackbox_sd_is_enabled(void) { return enabled; }
bool blackbox_sd_is_busy(void)
{
    return write_state != WRITE_IDLE || queue_count != 0U ||
           catalog_commit_pending;
}
blackbox_sd_state_t blackbox_sd_state(void) { return state; }
uint32_t blackbox_sd_capacity_mb(void) { return sector_count / 2048U; }
uint32_t blackbox_sd_written_bytes(void) { return written_bytes; }
uint32_t blackbox_sd_dropped_records(void) { return dropped_records; }

uint32_t blackbox_sd_total_bytes(void)
{
    uint32_t blocks = 0U;
    for (uint32_t i = 0U; i < catalog.flight_count; ++i) {
        blocks += catalog.flights[i].block_count;
    }
    return blocks * SD_BLOCK_SIZE;
}

uint32_t blackbox_sd_flight_count(void) { return catalog.flight_count; }

bool blackbox_sd_get_flight(uint32_t index,
                            blackbox_sd_flight_info_t *info)
{
    if (info == NULL || index >= catalog.flight_count) return false;
    const blackbox_catalog_entry_t *const entry =
        &catalog.flights[catalog.flight_count - 1U - index];
    info->flight_id = entry->flight_id;
    info->record_count = entry->record_count;
    info->block_count = entry->block_count;
    info->stop_flag = entry->stop_flag;
    return true;
}

bool blackbox_sd_get_record(uint32_t requested_flight_id,
                            uint32_t record_index,
                            flight_log_record_t *record)
{
    if (record == NULL || state != BLACKBOX_SD_READY ||
        write_state != WRITE_IDLE || queue_count != 0U ||
        catalog_commit_pending) return false;
    const blackbox_catalog_entry_t *entry = NULL;
    for (uint32_t i = 0U; i < catalog.flight_count; ++i) {
        if (catalog.flights[i].flight_id == requested_flight_id) {
            entry = &catalog.flights[i];
            break;
        }
    }
    if (entry == NULL || record_index >= entry->record_count) return false;
    uint32_t metadata_blocks = 0U;
    if ((!read_cache_valid || read_cache_sector != entry->start_sector) &&
        !read_sector(entry->start_sector, &read_cache)) return false;
    read_cache_valid = true;
    read_cache_sector = entry->start_sector;
    if (read_cache.magic == SD_BLOCK_MAGIC &&
        read_cache.flight_id == requested_flight_id &&
        read_cache.version == 5U && read_cache.record_count == 0U &&
        read_cache.checksum == checksum_block(&read_cache)) {
        metadata_blocks = 1U;
    }
    const uint32_t block_index = metadata_blocks +
        record_index / SD_RECORDS_PER_BLOCK;
    const uint32_t record_in_block = record_index % SD_RECORDS_PER_BLOCK;
    if (block_index >= entry->block_count) return false;
    const uint32_t sector = entry->start_sector + block_index;
    if ((!read_cache_valid || read_cache_sector != sector) &&
        !read_sector(sector, &read_cache)) return false;
    read_cache_valid = true;
    read_cache_sector = sector;
    if (read_cache.magic != SD_BLOCK_MAGIC ||
        read_cache.flight_id != requested_flight_id ||
        read_cache.sequence != block_index ||
        read_cache.version != 4U ||
        read_cache.record_count > SD_RECORDS_PER_BLOCK ||
        record_in_block >= read_cache.record_count ||
        read_cache.checksum != checksum_block(&read_cache)) return false;
    *record = read_cache.records[record_in_block];
    return true;
}

bool blackbox_sd_get_metadata(uint32_t requested_flight_id,
                              flight_log_metadata_t *metadata)
{
    if (metadata == NULL || state != BLACKBOX_SD_READY ||
        blackbox_sd_is_busy()) return false;
    const blackbox_catalog_entry_t *entry = NULL;
    for (uint32_t i = 0U; i < catalog.flight_count; ++i) {
        if (catalog.flights[i].flight_id == requested_flight_id) {
            entry = &catalog.flights[i]; break;
        }
    }
    if (entry == NULL || !read_sector(entry->start_sector, &read_cache) ||
        read_cache.magic != SD_BLOCK_MAGIC ||
        read_cache.flight_id != requested_flight_id ||
        read_cache.version != 5U || read_cache.record_count != 0U ||
        read_cache.checksum != checksum_block(&read_cache)) return false;
    memcpy(metadata, read_cache.records, sizeof(*metadata));
    read_cache_valid = true;
    read_cache_sector = entry->start_sector;
    return metadata->version == FLIGHT_LOG_METADATA_VERSION;
}

bool blackbox_sd_clear(void)
{
    if (state != BLACKBOX_SD_READY || write_state != WRITE_IDLE ||
        queue_count != 0U) return false;
    const uint32_t generation = catalog.generation + 1U;
    reset_catalog();
    catalog.generation = generation;
    written_bytes = 0U;
    dropped_records = 0U;
    catalog_commit_pending = true;
    read_cache_valid = false;
    return true;
}

const char *blackbox_sd_state_name(void)
{
    static const char *const names[] = {"UNSUPPORTED","ABSENT","INITIALIZING","READY","RECORDING","ERROR"};
    return names[(unsigned)state <= BLACKBOX_SD_ERROR ? state : BLACKBOX_SD_ERROR];
}

void blackbox_sd_start(const flight_log_metadata_t *metadata)
{
    if (!enabled || state != BLACKBOX_SD_READY ||
        write_state != WRITE_IDLE || queue_count != 0U ||
        catalog_commit_pending) return;
    if (next_sector >= sector_count) {
        state = BLACKBOX_SD_ERROR;
        enabled = false;
        return;
    }
    flight_id = catalog.next_flight_id;
    sequence = 0U;
    current_start_sector = next_sector;
    current_block_count = 0U;
    current_record_count = 0U;
    memset(&current, 0, sizeof(current));
    read_cache_valid = false;
    if (!queue_metadata(metadata)) return;
    state = BLACKBOX_SD_RECORDING;
}

void blackbox_sd_append(const flight_log_record_t *record)
{
    if (state != BLACKBOX_SD_RECORDING || record == NULL) return;
    if (current.record_count == 0U) {
        current.magic = SD_BLOCK_MAGIC; current.flight_id = flight_id;
        current.sequence = sequence++; current.timestamp_us = board_micros();
        current.version = 4U;
    }
    current.records[current.record_count++] = *record;
    if (current.record_count >= SD_RECORDS_PER_BLOCK) queue_current();
}

void blackbox_sd_stop(uint8_t stop_flag, bool retain)
{
    if (state != BLACKBOX_SD_RECORDING) return;
    current.stop_flag = stop_flag;
    if (retain) {
        queue_current();
        if (current_record_count > 0U) append_catalog_entry(stop_flag);
    } else {
        memset(&current, 0, sizeof(current));
    }
    state = BLACKBOX_SD_READY;
}

void blackbox_sd_get_diagnostics(blackbox_sd_diagnostics_t *diagnostics)
{
    if (diagnostics != NULL) memset(diagnostics, 0, sizeof(*diagnostics));
}
void blackbox_sd_restore_diagnostics(
    const blackbox_sd_diagnostics_t *diagnostics) { (void)diagnostics; }
bool blackbox_sd_write_test(blackbox_sd_write_test_t *result)
{ if (result != NULL) memset(result, 0, sizeof(*result)); return false; }
bool blackbox_sd_session_test(void) { return false; }

#endif
