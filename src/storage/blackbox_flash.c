#include "blackbox_sd.h"

#include <stddef.h>
#include <string.h>

#include "board.h"

#if BOARD_HAS_DATAFLASH

#define FLASH_BYTES (16U * 1024U * 1024U)
#define BANK_BYTES (FLASH_BYTES / 2U)
#define BANK_HEADER_BYTES 256U
#define ERASE_BLOCK_BYTES (64U * 1024U)
#define PAGE_BYTES 256U
#define QUEUE_RECORDS 16U
#define HEADER_MAGIC 0x42424646U /* FFBB */
#define HEADER_VERSION 1U
#define HEADER_VALID 0xA5U
#define CMD_JEDEC_ID 0x9FU
#define CMD_READ 0x03U
#define CMD_STATUS 0x05U
#define CMD_WRITE_ENABLE 0x06U
#define CMD_ENABLE_RESET 0x66U
#define CMD_RESET_DEVICE 0x99U
#define CMD_PAGE_PROGRAM 0x02U
#define CMD_BLOCK_ERASE 0xD8U
#define STATUS_BUSY 0x01U
#define STATUS_WRITE_ENABLED 0x02U

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t version;
    uint32_t flight_id;
    uint32_t record_count;
    uint8_t stop_flag;
    uint8_t valid;
    uint16_t record_size;
    uint8_t reserved[BANK_HEADER_BYTES - 20U];
} flash_header_t;

_Static_assert(sizeof(flash_header_t) == BANK_HEADER_BYTES,
               "dataflash header must occupy one page");

static blackbox_sd_state_t state;
static bool available;
static bool recording;
static bool finalise_pending;
static bool retain_pending;
static bool operation_waiting;
static uint32_t operation_deadline_us;
static bool erase_active;
static bool clear_second_pending;
static bool start_pending;
static bool enabled;
static uint8_t retained_bank;
static uint8_t write_bank;
static uint8_t erase_bank;
static uint32_t erase_address;
static uint32_t next_flight_id;
static uint32_t record_count;
static uint32_t written_bytes;
static uint32_t dropped_records;
static uint8_t stop_flag_pending;
static flight_log_record_t queue[QUEUE_RECORDS];
static uint8_t queue_read;
static uint8_t queue_write;
static uint8_t queue_count;
static uint8_t record_program_offset;
typedef enum { OP_NONE, OP_HEADER, OP_RECORD, OP_FINAL, OP_ERASE } flash_op_t;
typedef enum {
    ERR_NONE = 0, ERR_RESET, ERR_JEDEC, ERR_HEADER_READ,
    ERR_STATUS_TIMEOUT, ERR_ERASE_WREN, ERR_PROGRAM_WREN
} flash_error_t;
static flash_op_t operation;
static blackbox_sd_diagnostics_t diagnostics;

static void fail(flash_error_t code, uint32_t address, uint8_t status)
{
    diagnostics.error_code = (uint8_t)code;
    diagnostics.error_operation = (uint8_t)operation;
    diagnostics.error_address = address;
    diagnostics.last_status = status;
    operation_waiting = false;
    operation = OP_NONE;
    state = BLACKBOX_SD_ERROR;
}

static void begin_recording(void);

static uint32_t bank_address(uint8_t bank)
{
    return (uint32_t)bank * BANK_BYTES;
}

static void select_flash(bool selected)
{
    HAL_GPIO_WritePin(DATAFLASH_CS_PORT, DATAFLASH_CS_PIN,
                      selected ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static bool transmit(const uint8_t *data, uint16_t length)
{
    return HAL_SPI_Transmit(&DATAFLASH_SPI_HANDLE, (uint8_t *)data,
                            length, 2U) == HAL_OK;
}

static uint8_t status_register(void)
{
    uint8_t tx[2] = {CMD_STATUS, 0xFFU};
    uint8_t rx[2] = {0xFFU, 0xFFU};
    select_flash(true);
    const bool ok = HAL_SPI_TransmitReceive(&DATAFLASH_SPI_HANDLE, tx, rx,
                                             sizeof(tx), 2U) == HAL_OK;
    select_flash(false);
    return ok ? rx[1] : 0xFFU;
}

static bool write_enable(void)
{
    const uint8_t command = CMD_WRITE_ENABLE;
    select_flash(true);
    const bool ok = transmit(&command, 1U);
    select_flash(false);
    if (!ok) return false;

    /* WEL is latched when CS rises after WREN.  Check it once: retrying here
     * could otherwise stall the 8 kHz control loop for up to one millisecond
     * when the flash is absent or faulty.  The state machine reports failure
     * and disables logging instead of compromising flight control timing. */
    const uint8_t status = status_register();
    return status != 0xFFU && (status & STATUS_WRITE_ENABLED) != 0U;
}

static bool reset_device(void)
{
    const uint8_t commands[2] = {CMD_ENABLE_RESET, CMD_RESET_DEVICE};
    for (uint32_t i = 0U; i < sizeof(commands); ++i) {
        select_flash(true);
        const bool ok = transmit(&commands[i], 1U);
        select_flash(false);
        if (!ok) return false;
    }
    HAL_Delay(1U);
    const uint32_t deadline = board_micros() + 100000U;
    while ((int32_t)(board_micros() - deadline) < 0) {
        const uint8_t status = status_register();
        if (status != 0xFFU && (status & STATUS_BUSY) == 0U) return true;
    }
    return false;
}

static bool read_bytes(uint32_t address, void *destination, uint32_t length)
{
    uint8_t command[4] = {CMD_READ, (uint8_t)(address >> 16),
                          (uint8_t)(address >> 8), (uint8_t)address};
    select_flash(true);
    const bool ok = transmit(command, sizeof(command)) &&
        HAL_SPI_Receive(&DATAFLASH_SPI_HANDLE, destination,
                        (uint16_t)length, 100U) == HAL_OK;
    select_flash(false);
    return ok;
}

static bool start_program(uint32_t address, const void *source, uint8_t length)
{
    if (!write_enable()) return false;
    uint8_t command[4] = {CMD_PAGE_PROGRAM, (uint8_t)(address >> 16),
                          (uint8_t)(address >> 8), (uint8_t)address};
    select_flash(true);
    const bool ok = transmit(command, sizeof(command)) &&
                    transmit(source, length);
    select_flash(false);
    operation_waiting = ok;
    if (ok) operation_deadline_us = board_micros() + 1000000U;
    return ok;
}

static bool start_erase(uint32_t address)
{
    if (!write_enable()) return false;
    uint8_t command[4] = {CMD_BLOCK_ERASE, (uint8_t)(address >> 16),
                          (uint8_t)(address >> 8), (uint8_t)address};
    select_flash(true);
    const bool ok = transmit(command, sizeof(command));
    select_flash(false);
    operation_waiting = ok;
    if (ok) operation_deadline_us = board_micros() + 5000000U;
    return ok;
}

static bool header_valid(const flash_header_t *header)
{
    return header->magic == HEADER_MAGIC &&
           header->version == HEADER_VERSION &&
           header->valid == HEADER_VALID &&
           header->record_size == sizeof(flight_log_record_t) &&
           header->record_count <=
               (BANK_BYTES - BANK_HEADER_BYTES) /
                   sizeof(flight_log_record_t);
}

static bool header_erased(const flash_header_t *header)
{
    const uint8_t *bytes = (const uint8_t *)header;
    for (uint32_t i = 0U; i < sizeof(*header); ++i) {
        if (bytes[i] != 0xFFU) return false;
    }
    return true;
}

static void prepare_bank(uint8_t bank)
{
    erase_active = true;
    erase_bank = bank;
    /* Erase backwards: an erased header proves the whole bank is ready. */
    erase_address = bank_address(bank) + BANK_BYTES - ERASE_BLOCK_BYTES;
    state = BLACKBOX_SD_INITIALIZING;
}

void blackbox_sd_init(void)
{
    state = BLACKBOX_SD_UNSUPPORTED;
    memset(&diagnostics, 0, sizeof(diagnostics));
    available = false;
    enabled = true;
    recording = false;
    finalise_pending = false;
    operation_waiting = false;
    operation_deadline_us = 0U;
    operation = OP_NONE;
    erase_active = false;
    clear_second_pending = false;
    start_pending = false;
    retained_bank = 0xFFU;
    queue_read = queue_write = queue_count = 0U;
    record_program_offset = 0U;
    written_bytes = dropped_records = 0U;

    /* Match the W25Q128 startup sequence used by Betaflight.  This also
     * recovers a chip left in an incomplete command state after a reset. */
    if (!reset_device()) {
        fail(ERR_RESET, 0U, status_register());
        return;
    }

    uint8_t command = CMD_JEDEC_ID;
    uint8_t id[3] = {0xFFU, 0xFFU, 0xFFU};
    select_flash(true);
    const bool id_ok = transmit(&command, 1U) &&
        HAL_SPI_Receive(&DATAFLASH_SPI_HANDLE, id, sizeof(id), 10U) == HAL_OK;
    select_flash(false);
    /* Capacity code 0x18 is 128 Mbit / 16 MiB; accept all JEDEC vendors. */
    if (!id_ok || id[0] == 0U || id[0] == 0xFFU || id[2] != 0x18U) {
        fail(ERR_JEDEC, 0U, id[0]);
        return;
    }
    diagnostics.jedec_id = ((uint32_t)id[0] << 16) |
                           ((uint32_t)id[1] << 8) | id[2];
    available = true;

    flash_header_t headers[2];
    if (!read_bytes(bank_address(0U), &headers[0], sizeof(headers[0])) ||
        !read_bytes(bank_address(1U), &headers[1], sizeof(headers[1]))) {
        fail(ERR_HEADER_READ, 0U, status_register());
        return;
    }
    const bool valid0 = header_valid(&headers[0]);
    const bool valid1 = header_valid(&headers[1]);
    if (valid0 || valid1) {
        retained_bank = valid1 &&
            (!valid0 || headers[1].flight_id > headers[0].flight_id) ? 1U : 0U;
        const flash_header_t *latest = &headers[retained_bank];
        next_flight_id = latest->flight_id + 1U;
        record_count = latest->record_count;
        stop_flag_pending = latest->stop_flag;
    } else {
        next_flight_id = 1U;
        record_count = 0U;
    }
    uint8_t candidate = retained_bank == 0U ? 1U : 0U;
    if (retained_bank > 1U) {
        /* With no retained flight, use either already-erased bank at once. */
        if (header_erased(&headers[0])) candidate = 0U;
        else if (header_erased(&headers[1])) candidate = 1U;
    }
    if (header_erased(&headers[candidate])) {
        write_bank = candidate;
        state = BLACKBOX_SD_READY;
    } else prepare_bank(candidate);
}

void blackbox_sd_probe(void) { if (!available) blackbox_sd_init(); }

void blackbox_sd_update(void)
{
    if (!available || state == BLACKBOX_SD_ERROR) return;
    if (operation_waiting) {
        const uint8_t status = status_register();
        diagnostics.last_status = status;
        if (status == 0xFFU || (status & STATUS_BUSY) != 0U) {
            if ((int32_t)(board_micros() - operation_deadline_us) < 0)
                return;
            fail(ERR_STATUS_TIMEOUT,
                 operation == OP_ERASE ? erase_address :
                 bank_address(write_bank) + BANK_HEADER_BYTES +
                    record_count * sizeof(flight_log_record_t) +
                    record_program_offset,
                 status);
            return;
        }
        operation_waiting = false;
        const flash_op_t completed = operation;
        operation = OP_NONE;
        if (completed == OP_ERASE) {
            const uint32_t first = bank_address(erase_bank);
            if (erase_address > first) {
                erase_address -= ERASE_BLOCK_BYTES;
            } else {
                erase_active = false;
                if (clear_second_pending && erase_bank == 0U) {
                    clear_second_pending = false;
                    prepare_bank(1U);
                } else {
                    write_bank = erase_bank;
                    state = BLACKBOX_SD_READY;
                    if (start_pending) {
                        start_pending = false;
                        begin_recording();
                    }
                }
            }
        } else if (completed == OP_RECORD) {
            const uint32_t address = bank_address(write_bank) +
                BANK_HEADER_BYTES + record_count * sizeof(flight_log_record_t) +
                record_program_offset;
            const uint8_t remaining =
                (uint8_t)(sizeof(flight_log_record_t) - record_program_offset);
            const uint16_t page_remaining =
                PAGE_BYTES - (address & (PAGE_BYTES - 1U));
            if (remaining > page_remaining) {
                record_program_offset += (uint8_t)page_remaining;
            } else {
                record_program_offset = 0U;
                queue_read = (uint8_t)((queue_read + 1U) % QUEUE_RECORDS);
                --queue_count;
                ++record_count;
                written_bytes += sizeof(flight_log_record_t);
            }
        } else if (completed == OP_FINAL) {
            recording = false;
            const uint8_t old = retained_bank;
            retained_bank = write_bank;
            state = BLACKBOX_SD_READY;
            if (old <= 1U && old != retained_bank) {
                prepare_bank(old);
            } else {
                /*
                 * This is the first retained flight. Move the next recording
                 * to the other, factory-erased bank; never reuse the bank we
                 * have just committed (NOR flash cannot change 0 bits to 1).
                 */
                const uint8_t candidate = retained_bank == 0U ? 1U : 0U;
                flash_header_t header;
                if (read_bytes(bank_address(candidate), &header,
                               sizeof(header)) && header_erased(&header)) {
                    write_bank = candidate;
                } else {
                    prepare_bank(candidate);
                }
            }
        }
    }

    if (operation_waiting) return;
    if (erase_active) {
        if (!start_erase(erase_address))
            fail(ERR_ERASE_WREN, erase_address, status_register());
        else operation = OP_ERASE;
        return;
    }
    if (queue_count > 0U) {
        const uint32_t address = bank_address(write_bank) +
            BANK_HEADER_BYTES + record_count * sizeof(flight_log_record_t) +
            record_program_offset;
        const uint8_t remaining =
            (uint8_t)(sizeof(flight_log_record_t) - record_program_offset);
        const uint16_t page_remaining =
            PAGE_BYTES - (address & (PAGE_BYTES - 1U));
        const uint8_t length = remaining < page_remaining
            ? remaining : (uint8_t)page_remaining;
        if (!start_program(address,
                (const uint8_t *)&queue[queue_read] + record_program_offset,
                length)) fail(ERR_PROGRAM_WREN, address, status_register());
        else operation = OP_RECORD;
        return;
    }
    if (finalise_pending) {
        if (!retain_pending) {
            finalise_pending = false;
            recording = false;
            prepare_bank(write_bank);
            return;
        }
        uint8_t final_data[6];
        memcpy(final_data, &record_count, sizeof(record_count));
        final_data[4] = stop_flag_pending;
        final_data[5] = HEADER_VALID;
        if (!start_program(bank_address(write_bank) +
                           offsetof(flash_header_t, record_count),
                           final_data, sizeof(final_data))) {
            fail(ERR_PROGRAM_WREN,
                 bank_address(write_bank) +
                    offsetof(flash_header_t, record_count),
                 status_register());
            return;
        }
        operation = OP_FINAL;
        finalise_pending = false;
    }
}

void blackbox_sd_set_enabled(bool value) { (void)value; enabled = available; }
bool blackbox_sd_is_enabled(void) { return enabled && available; }
bool blackbox_sd_is_busy(void)
{
    return erase_active || operation_waiting || finalise_pending ||
           queue_count > 0U;
}
blackbox_sd_state_t blackbox_sd_state(void) { return state; }
const char *blackbox_sd_state_name(void)
{
    static const char *const names[] = {"UNSUPPORTED", "ABSENT", "INITIALIZING",
                                        "READY", "RECORDING", "ERROR"};
    return names[(unsigned)state <= BLACKBOX_SD_ERROR ? state
                                                       : BLACKBOX_SD_ERROR];
}
uint32_t blackbox_sd_capacity_mb(void) { return available ? 16U : 0U; }
uint32_t blackbox_sd_written_bytes(void) { return written_bytes; }
uint32_t blackbox_sd_dropped_records(void) { return dropped_records; }
uint32_t blackbox_sd_total_bytes(void)
{
    if (retained_bank > 1U) return 0U;
    flash_header_t header;
    return read_bytes(bank_address(retained_bank), &header, sizeof(header)) &&
           header_valid(&header)
        ? header.record_count * sizeof(flight_log_record_t) : 0U;
}
uint32_t blackbox_sd_flight_count(void) { return retained_bank <= 1U ? 1U : 0U; }

bool blackbox_sd_get_flight(uint32_t index, blackbox_sd_flight_info_t *info)
{
    if (index != 0U || retained_bank > 1U || info == NULL ||
        blackbox_sd_is_busy()) return false;
    flash_header_t header;
    if (!read_bytes(bank_address(retained_bank), &header, sizeof(header)) ||
        !header_valid(&header)) return false;
    info->flight_id = header.flight_id;
    info->record_count = header.record_count;
    info->block_count = (header.record_count * sizeof(flight_log_record_t) +
                         PAGE_BYTES - 1U) / PAGE_BYTES;
    info->stop_flag = header.stop_flag;
    return true;
}

bool blackbox_sd_get_record(uint32_t flight_id, uint32_t index,
                            flight_log_record_t *record)
{
    if (retained_bank > 1U || record == NULL || blackbox_sd_is_busy())
        return false;
    flash_header_t header;
    if (!read_bytes(bank_address(retained_bank), &header, sizeof(header)) ||
        !header_valid(&header) || header.flight_id != flight_id ||
        index >= header.record_count) return false;
    return read_bytes(bank_address(retained_bank) + BANK_HEADER_BYTES +
                      index * sizeof(*record), record, sizeof(*record));
}

bool blackbox_sd_clear(void)
{
    if (recording || blackbox_sd_is_busy()) return false;
    retained_bank = 0xFFU;
    record_count = 0U;
    next_flight_id = 1U;
    clear_second_pending = true;
    prepare_bank(0U);
    return true;
}

static void begin_recording(void)
{
    flash_header_t header;
    memset(&header, 0xFF, sizeof(header));
    header.magic = HEADER_MAGIC;
    header.version = HEADER_VERSION;
    header.flight_id = next_flight_id++;
    header.record_size = sizeof(flight_log_record_t);
    record_count = 0U;
    written_bytes = dropped_records = 0U;
    queue_read = queue_write = queue_count = 0U;
    record_program_offset = 0U;
    finalise_pending = false;
    if (!start_program(bank_address(write_bank), &header, 20U)) {
        fail(ERR_PROGRAM_WREN, bank_address(write_bank), status_register());
        return;
    }
    operation = OP_HEADER;
    recording = true;
    state = BLACKBOX_SD_RECORDING;
}

void blackbox_sd_start(void)
{
    ++diagnostics.start_calls;
    diagnostics.start_reject_mask = 0U;
    if (!enabled) diagnostics.start_reject_mask |= 1U;
    if (state != BLACKBOX_SD_READY) diagnostics.start_reject_mask |= 2U;
    if (erase_active) diagnostics.start_reject_mask |= 4U;
    if (write_bank > 1U) diagnostics.start_reject_mask |= 8U;
    if (write_bank == retained_bank) diagnostics.start_reject_mask |= 16U;
    if (diagnostics.start_reject_mask != 0U) {
        /* A bank erase may finish while this same armed flight is running. */
        if (enabled && erase_active &&
            (diagnostics.start_reject_mask & ~(2U | 4U)) == 0U) {
            start_pending = true;
        }
        return;
    }
    start_pending = false;
    begin_recording();
}

void blackbox_sd_append(const flight_log_record_t *record)
{
    const uint32_t maximum =
        (BANK_BYTES - BANK_HEADER_BYTES) / sizeof(flight_log_record_t);
    ++diagnostics.append_calls;
    if (!recording || finalise_pending || record == NULL) return;
    if (record_count + queue_count >= maximum || queue_count >= QUEUE_RECORDS) {
        ++dropped_records;
        return;
    }
    queue[queue_write] = *record;
    queue_write = (uint8_t)((queue_write + 1U) % QUEUE_RECORDS);
    ++queue_count;
}

void blackbox_sd_stop(uint8_t stop_flag, bool retain)
{
    ++diagnostics.stop_calls;
    diagnostics.last_retain = retain ? 1U : 0U;
    if (!recording) { start_pending = false; return; }
    stop_flag_pending = stop_flag;
    retain_pending = retain && (record_count + queue_count > 0U);
    finalise_pending = true;
}

void blackbox_sd_get_diagnostics(blackbox_sd_diagnostics_t *result)
{
    if (result == NULL) return;
    diagnostics.completed_records = record_count;
    diagnostics.state = (uint8_t)state;
    diagnostics.operation = (uint8_t)operation;
    diagnostics.queue_count = queue_count;
    diagnostics.write_bank = write_bank;
    diagnostics.retained_bank = retained_bank;
    diagnostics.erase_active = erase_active ? 1U : 0U;
    diagnostics.finalise_pending = finalise_pending ? 1U : 0U;
    *result = diagnostics;
}

void blackbox_sd_restore_diagnostics(
    const blackbox_sd_diagnostics_t *saved)
{
    if (saved == NULL || saved->jedec_id == 0U) return;
    /* Keep the current hardware state but restore the last flight counters. */
    const uint32_t current_jedec = diagnostics.jedec_id;
    diagnostics = *saved;
    diagnostics.jedec_id = current_jedec != 0U
        ? current_jedec : saved->jedec_id;
}

bool blackbox_sd_write_test(blackbox_sd_write_test_t *result)
{
    if (result == NULL || state != BLACKBOX_SD_READY || recording ||
        blackbox_sd_is_busy() || write_bank > 1U ||
        write_bank == retained_bank) return false;

    memset(result, 0, sizeof(*result));
    result->address = bank_address(write_bank) + 128U;
    uint8_t before[32];
    uint8_t after[32];
    static const uint8_t pattern[32] = {
        0x46,0x6C,0x69,0x67,0x68,0x74,0x43,0x6F,
        0x64,0x65,0x2D,0x42,0x42,0x2D,0x54,0x45,
        0x53,0x54,0x2D,0xA5,0x5A,0x3C,0xC3,0x96,
        0x69,0x00,0xFF,0x12,0x34,0x56,0x78,0xE7
    };
    if (!read_bytes(result->address, before, sizeof(before))) return true;
    result->before_erased = 1U;
    for (uint8_t i = 0U; i < sizeof(before); ++i) {
        if (before[i] != 0xFFU) { result->before_erased = 0U; break; }
    }

    result->program_ok = start_program(result->address, pattern,
                                       sizeof(pattern)) ? 1U : 0U;
    if (result->program_ok) {
        const uint32_t deadline = board_micros() + 1000000U;
        while ((status_register() & 1U) != 0U &&
               (int32_t)(board_micros() - deadline) < 0) HAL_Delay(1U);
        operation_waiting = false;
        operation = OP_NONE;
        result->read_ok = read_bytes(result->address, after,
                                     sizeof(after)) ? 1U : 0U;
    }
    result->verify_ok = result->read_ok;
    result->mismatch_index = 0xFFU;
    if (result->read_ok) {
        for (uint8_t i = 0U; i < sizeof(pattern); ++i) {
            if (after[i] != pattern[i]) {
                result->verify_ok = 0U;
                result->mismatch_index = i;
                result->expected = pattern[i];
                result->actual = after[i];
                break;
            }
        }
    }
    /* The test dirtied only the inactive bank; prepare it again afterwards. */
    prepare_bank(write_bank);
    return true;
}

bool blackbox_sd_session_test(void)
{
    if (state != BLACKBOX_SD_READY || recording || blackbox_sd_is_busy() ||
        write_bank > 1U || write_bank == retained_bank) return false;
    flight_log_record_t record;
    memset(&record, 0, sizeof(record));
    record.gyro[0] = 123;
    record.gyro[1] = -456;
    record.gyro[2] = 789;
    record.throttle = 20U;
    record.loop_us = 125U;
    blackbox_sd_start();
    if (!recording) return false;
    blackbox_sd_append(&record);
    blackbox_sd_stop(FLIGHT_LOG_FLAG_STOP_DISARM, true);
    return true;
}

#endif
