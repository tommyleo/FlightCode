#include "max7456.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "board.h"

#define REG_VM0 0x00U
#define REG_DMM 0x04U
#define REG_DMAH 0x05U
#define REG_DMAL 0x06U
#define REG_DMDI 0x07U
#define REG_CMM 0x08U
#define REG_CMAH 0x09U
#define REG_CMAL 0x0AU
#define REG_CMDI 0x0BU
#define REG_OSDM_READ 0x8CU
#define REG_STAT 0xA0U
#define REG_CMDO_READ 0xC0U
#define VM0_RESET 0x02U
#define VM0_ENABLE 0x08U
#define VM0_PAL 0x40U
#define DMM_CLEAR 0x04U
#define CMM_READ_NVM 0x50U
#define CMM_WRITE_NVM 0xA0U
#define STAT_NVM_BUSY 0x20U
#define STAT_PAL 0x01U
#define STAT_NTSC 0x02U
#define STAT_LOS 0x04U
#define SCREEN_COLUMNS 30U
#define BATTERY_FIELD_LENGTH 7U
#define BATTERY_CRITICAL_CELL_VOLTAGE 3.50f
#define BATTERY_BLINK_HALF_PERIOD_US 400000U

static bool available;
static bool font_ready;
static uint8_t probe_value = 0xFFU;
static uint8_t probe_spi_mode = 0xFFU;
static bool video_pal = true;
static bool enabled;
static bool menu_active;
static uint8_t selected_position = 4U;
static uint8_t battery_column = 11U;
static uint8_t battery_row = 7U;
static uint8_t detected_battery_cells;
static char previous[BATTERY_FIELD_LENGTH + 1U];

/* Compact 5x7 font: blank, digits, punctuation, and uppercase letters. */
static const uint8_t font_5x7[][7] = {
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E},
    {0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E},
    {0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F},
    {0x1E, 0x01, 0x01, 0x0E, 0x01, 0x01, 0x1E},
    {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02},
    {0x1F, 0x10, 0x10, 0x1E, 0x01, 0x01, 0x1E},
    {0x0E, 0x10, 0x10, 0x1E, 0x11, 0x11, 0x0E},
    {0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08},
    {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E},
    {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x01, 0x0E},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C},
    {0x11, 0x11, 0x11, 0x11, 0x0A, 0x0A, 0x04},
    {0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11}, /* A */
    {0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E}, /* B */
    {0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E}, /* C */
    {0x1E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1E}, /* D */
    {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F}, /* E */
    {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10}, /* F */
    {0x0E, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0F}, /* G */
    {0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11}, /* H */
    {0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E}, /* I */
    {0x07, 0x02, 0x02, 0x02, 0x12, 0x12, 0x0C}, /* J */
    {0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11}, /* K */
    {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F}, /* L */
    {0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11}, /* M */
    {0x11, 0x19, 0x19, 0x15, 0x13, 0x13, 0x11}, /* N */
    {0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E}, /* O */
    {0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10}, /* P */
    {0x0E, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0D}, /* Q */
    {0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11}, /* R */
    {0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E}, /* S */
    {0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04}, /* T */
    {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E}, /* U */
    {0x11, 0x11, 0x11, 0x11, 0x0A, 0x0A, 0x04}, /* V */
    {0x11, 0x11, 0x11, 0x15, 0x15, 0x1B, 0x11}, /* W */
    {0x11, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x11}, /* X */
    {0x11, 0x11, 0x0A, 0x04, 0x04, 0x04, 0x04}, /* Y */
    {0x1F, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1F}, /* Z */
    {0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00}, /* - */
    {0x10, 0x08, 0x04, 0x02, 0x04, 0x08, 0x10}, /* > */
    {0x01, 0x02, 0x04, 0x08, 0x04, 0x02, 0x01}, /* < */
    {0x19, 0x1A, 0x02, 0x04, 0x08, 0x0B, 0x13}, /* % */
    {0x01, 0x02, 0x02, 0x04, 0x08, 0x08, 0x10}, /* / */
};

static uint8_t transfer(uint8_t address, uint8_t value)
{
#if BOARD_HAS_OSD
    uint8_t tx[2] = {address, value};
    uint8_t rx[2] = {0};
    HAL_GPIO_WritePin(MAX7456_CS_PORT, MAX7456_CS_PIN, GPIO_PIN_RESET);
    const HAL_StatusTypeDef result =
        HAL_SPI_TransmitReceive(&OSD_SPI_HANDLE, tx, rx, sizeof(tx), 10U);
    HAL_GPIO_WritePin(MAX7456_CS_PORT, MAX7456_CS_PIN, GPIO_PIN_SET);
    return result == HAL_OK ? rx[1] : 0xFFU;
#else
    (void)address;
    (void)value;
    return 0xFFU;
#endif
}

static void write_register(uint8_t address, uint8_t value)
{
    (void)transfer(address, value);
}

static void end_pending_transaction(void)
{
#if BOARD_HAS_OSD
    const uint8_t end = 0xFFU;
    HAL_GPIO_WritePin(MAX7456_CS_PORT, MAX7456_CS_PIN, GPIO_PIN_RESET);
    (void)HAL_SPI_Transmit(&OSD_SPI_HANDLE, &end, 1U, 10U);
    HAL_GPIO_WritePin(MAX7456_CS_PORT, MAX7456_CS_PIN, GPIO_PIN_SET);
    HAL_Delay(1U);
#endif
}

static bool configure_spi_mode(uint8_t mode)
{
#if BOARD_HAS_OSD
    static const uint32_t polarity[4] = {
        SPI_POLARITY_LOW, SPI_POLARITY_LOW,
        SPI_POLARITY_HIGH, SPI_POLARITY_HIGH,
    };
    static const uint32_t phase[4] = {
        SPI_PHASE_1EDGE, SPI_PHASE_2EDGE,
        SPI_PHASE_1EDGE, SPI_PHASE_2EDGE,
    };
    (void)HAL_SPI_DeInit(&OSD_SPI_HANDLE);
    OSD_SPI_HANDLE.Init.CLKPolarity = polarity[mode];
    OSD_SPI_HANDLE.Init.CLKPhase = phase[mode];
    OSD_SPI_HANDLE.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
    return HAL_SPI_Init(&OSD_SPI_HANDLE) == HAL_OK;
#else
    (void)mode;
    return false;
#endif
}

static bool wait_nvm(void)
{
    for (uint32_t timeout = 0U; timeout < 250U; ++timeout) {
        if ((transfer(REG_STAT, 0xFFU) & STAT_NVM_BUSY) == 0U) return true;
        HAL_Delay(1U);
    }
    return false;
}

static bool glyph_pixel_is_white(uint8_t character, int8_t x, int8_t y)
{
    if (x < 1 || x >= 11 || y < 2 || y >= 16) return false;
    const uint8_t source_x = (uint8_t)(x - 1) / 2U;
    const uint8_t source_y = (uint8_t)(y - 2) / 2U;
    return (font_5x7[character][source_y] &
            (uint8_t)(1U << (4U - source_x))) != 0U;
}

static void build_glyph(uint8_t character, uint8_t glyph[54])
{
    for (uint8_t y = 0U; y < 18U; ++y) {
        for (uint8_t group = 0U; group < 3U; ++group) {
            uint8_t packed = 0U;
            for (uint8_t part = 0U; part < 4U; ++part) {
                const uint8_t x = (uint8_t)(group * 4U + part);
                uint8_t pixel = 0x01U; /* Transparent. */
                if (glyph_pixel_is_white(character, (int8_t)x, (int8_t)y)) {
                    pixel = 0x02U; /* White fill. */
                } else {
                    for (int8_t dy = -1; dy <= 1 && pixel != 0x00U; ++dy) {
                        for (int8_t dx = -1; dx <= 1; ++dx) {
                            if (glyph_pixel_is_white(character,
                                                     (int8_t)x + dx,
                                                     (int8_t)y + dy)) {
                                pixel = 0x00U; /* Black outline. */
                                break;
                            }
                        }
                    }
                }
                packed |= (uint8_t)(pixel << (6U - part * 2U));
            }
            glyph[y * 3U + group] = packed;
        }
    }
}

static bool glyph_matches(uint8_t character, const uint8_t glyph[54])
{
    write_register(REG_CMAH, character);
    write_register(REG_CMM, CMM_READ_NVM);
    if (!wait_nvm()) return false;
    for (uint8_t i = 0U; i < 54U; ++i) {
        write_register(REG_CMAL, i);
        if (transfer(REG_CMDO_READ, 0xFFU) != glyph[i]) return false;
    }
    return true;
}

static bool install_font(void)
{
    uint8_t glyph[54];
    for (uint8_t character = 0U;
         character < (uint8_t)(sizeof(font_5x7) / sizeof(font_5x7[0]));
         ++character) {
        build_glyph(character, glyph);
        if (glyph_matches(character, glyph)) continue;
        write_register(REG_CMAH, character);
        for (uint8_t i = 0U; i < 54U; ++i) {
            write_register(REG_CMAL, i);
            write_register(REG_CMDI, glyph[i]);
        }
        write_register(REG_CMM, CMM_WRITE_NVM);
        if (!wait_nvm()) return false;
    }
    return true;
}

static uint8_t font_character(char character)
{
    if (character >= '0' && character <= '9') {
        return (uint8_t)(character - '0' + 1);
    }
    if (character == '.') return 11U;
    if (character == 'V') return 12U;
    if (character >= 'A' && character <= 'Z') {
        return (uint8_t)(character - 'A' + 13);
    }
    if (character == '-') return 39U;
    if (character == '>') return 40U;
    if (character == '<') return 41U;
    if (character == '%') return 42U;
    if (character == '/') return 43U;
    return 0U;
}

static void write_character(uint16_t position, uint8_t character)
{
    write_register(REG_DMAH, (uint8_t)(position >> 8U));
    write_register(REG_DMAL, (uint8_t)position);
    write_register(REG_DMDI, character);
}

static void clear_battery_field(void)
{
    const uint16_t start = battery_row * SCREEN_COLUMNS + battery_column;
    for (uint8_t i = 0U; i < BATTERY_FIELD_LENGTH; ++i) {
        write_character((uint16_t)(start + i), 0U);
    }
}

static void select_position(uint8_t position)
{
    static const uint8_t columns[3] = {1U, 11U, 22U};
    static const uint8_t pal_rows[3] = {1U, 7U, 14U};
    static const uint8_t ntsc_rows[3] = {1U, 6U, 11U};
    battery_column = columns[position % 3U];
    battery_row = (video_pal ? pal_rows : ntsc_rows)[position / 3U];
}

bool max7456_init(void)
{
#if BOARD_HAS_OSD
    HAL_Delay(50U);
    static const uint8_t modes[4] = {3U, 0U, 1U, 2U};
    for (uint8_t candidate = 0U; candidate < 4U; ++candidate) {
        const uint8_t mode = modes[candidate];
        if (!configure_spi_mode(mode)) continue;
        for (uint8_t attempt = 0U; attempt < 3U; ++attempt) {
            end_pending_transaction();
            probe_value = transfer(REG_OSDM_READ, 0xFFU);
            if (probe_value == 0x1BU) {
                probe_spi_mode = mode;
                break;
            }
            HAL_Delay(10U);
        }
        if (probe_spi_mode != 0xFFU) break;
    }
    if (probe_value != 0x1BU) return false;
    available = true;
    write_register(REG_VM0, VM0_RESET);
    HAL_Delay(100U);
    font_ready = install_font();
    HAL_Delay(100U);
    const uint8_t video_status = transfer(REG_STAT, 0xFFU);
    if ((video_status & STAT_LOS) == 0U) {
        if ((video_status & STAT_NTSC) != 0U) video_pal = false;
        else if ((video_status & STAT_PAL) != 0U) video_pal = true;
    }
    const uint8_t video_mode = video_pal ? VM0_PAL : 0U;
    select_position(selected_position);
    write_register(REG_DMM, DMM_CLEAR);
    HAL_Delay(1U);
    write_register(REG_VM0, video_mode | (enabled ? VM0_ENABLE : 0U));
    memset(previous, 0, sizeof(previous));
    return true;
#else
    return false;
#endif
}

bool max7456_font_is_ready(void)
{
    return font_ready;
}

uint8_t max7456_probe_value(void)
{
    return probe_value;
}

uint8_t max7456_probe_spi_mode(void)
{
    return probe_spi_mode;
}

bool max7456_video_is_pal(void)
{
    return video_pal;
}

bool max7456_is_available(void)
{
    return available;
}

bool max7456_is_enabled(void)
{
    return enabled;
}

uint8_t max7456_position(void)
{
    return selected_position;
}

bool max7456_set_config(bool requested_enabled, uint8_t position)
{
    if (position > 8U) return false;
    if (available && enabled && !menu_active) clear_battery_field();
    selected_position = position;
    select_position(position);
    enabled = requested_enabled;
    memset(previous, 0, sizeof(previous));
    if (!available) return true;
    if (!menu_active) {
        write_register(REG_VM0, (video_pal ? VM0_PAL : 0U) |
                                (enabled ? VM0_ENABLE : 0U));
    }
    return true;
}

void max7456_update_battery(float voltage)
{
    if (!available || !enabled || menu_active) return;
    char text[BATTERY_FIELD_LENGTH + 1U];
    memset(text, ' ', BATTERY_FIELD_LENGTH);
    text[BATTERY_FIELD_LENGTH] = '\0';
    bool show_voltage = true;
    if (voltage >= 1.0f && voltage < 100.0f) {
        uint8_t candidate_cells = (uint8_t)ceilf(voltage / 4.25f);
        if (candidate_cells > 8U) candidate_cells = 8U;
        if (candidate_cells > detected_battery_cells) {
            detected_battery_cells = candidate_cells;
        }
        const float cell_voltage =
            voltage / (float)detected_battery_cells;
        const bool critical =
            cell_voltage < BATTERY_CRITICAL_CELL_VOLTAGE;
        if (critical) {
            show_voltage = ((board_micros() / BATTERY_BLINK_HALF_PERIOD_US) &
                            1U) == 0U;
        }
        if (show_voltage) {
            char value[BATTERY_FIELD_LENGTH + 1U];
            const int length =
                snprintf(value, sizeof(value), "%5.2fV", cell_voltage);
            if (length > 0) memcpy(text, value, (size_t)length);
        }
    } else {
        detected_battery_cells = 0U;
    }
    if (memcmp(text, previous, BATTERY_FIELD_LENGTH) == 0) return;
    const uint16_t start = battery_row * SCREEN_COLUMNS + battery_column;
    for (uint8_t i = 0U; i < BATTERY_FIELD_LENGTH; ++i) {
        write_character((uint16_t)(start + i), font_character(text[i]));
    }
    memcpy(previous, text, sizeof(previous));
}

void max7456_clear_screen(void)
{
    if (!available) return;
    write_register(REG_DMM, DMM_CLEAR);
    HAL_Delay(1U);
}

void max7456_write_text(uint8_t row, uint8_t column, const char *text)
{
    if (!available || text == NULL || row >= 16U || column >= SCREEN_COLUMNS) {
        return;
    }
    uint16_t position = (uint16_t)row * SCREEN_COLUMNS + column;
    while (*text != '\0' && column++ < SCREEN_COLUMNS) {
        write_character(position++, font_character(*text++));
    }
}

bool max7456_menu_begin(void)
{
    if (!available || !font_ready) return false;
    menu_active = true;
    write_register(REG_VM0, (video_pal ? VM0_PAL : 0U) | VM0_ENABLE);
    max7456_clear_screen();
    return true;
}

void max7456_menu_end(void)
{
    if (!menu_active) return;
    max7456_clear_screen();
    menu_active = false;
    memset(previous, 0, sizeof(previous));
    write_register(REG_VM0, (video_pal ? VM0_PAL : 0U) |
                            (enabled ? VM0_ENABLE : 0U));
}
