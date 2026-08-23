#include "max7456.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "board.h"
#include "font_flightcode.h"
#include "flight_settings.h"

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
#define SCREEN_ROWS 16U
#define OSD_ELEMENT_COUNT 5U
#define OSD_TEXT_LENGTH 12U
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
static uint32_t layout_enabled_mask = 1U;
static uint32_t layout_positions[OSD_ELEMENT_COUNT] = {
    31U, 61U, 51U, 340U, 369U,
};
static char pilot_name[OSD_TEXT_LENGTH + 1U] = "PILOT";
static uint8_t detected_battery_cells;
static bool previous_armed;
static bool timer_started;
static uint32_t flight_started_us;
static uint32_t flight_duration_us;
static char previous_screen[SCREEN_COLUMNS * SCREEN_ROWS];
static bool render_cache_valid;

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
#if BOARD_OSD_SHARES_DATAFLASH_SPI
    /* The MAX7456 and W25Q128 on the analog Flywoo share SPI3.  Mode 0 is
     * supported by both devices, so keep one bus configuration at runtime. */
    if (mode != 0U) return false;
#endif
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
#if BOARD_OSD_SHARES_DATAFLASH_SPI
    OSD_SPI_HANDLE.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
#else
    OSD_SPI_HANDLE.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
#endif
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

static bool glyph_pixel_is_white(const flightcode_font_glyph_t *glyph,
                                 int8_t x, int8_t y)
{
    if (x < 3 || x >= 8 || y < 5 || y >= 12) return false;
    return (glyph->rows[(uint8_t)(y - 5)] &
            (uint8_t)(1U << (7 - x))) != 0U;
}

static void build_glyph(const flightcode_font_glyph_t *source,
                        uint8_t glyph[54])
{
    for (uint8_t y = 0U; y < 18U; ++y) {
        for (uint8_t group = 0U; group < 3U; ++group) {
            uint8_t packed = 0U;
            for (uint8_t part = 0U; part < 4U; ++part) {
                const uint8_t x = (uint8_t)(group * 4U + part);
                uint8_t pixel = 0x01U;
                if (glyph_pixel_is_white(source, (int8_t)x, (int8_t)y)) {
                    pixel = 0x02U;
                } else {
                    for (int8_t dy = -1; dy <= 1 && pixel != 0x00U; ++dy) {
                        for (int8_t dx = -1; dx <= 1; ++dx) {
                            if (glyph_pixel_is_white(source,
                                                     (int8_t)x + dx,
                                                     (int8_t)y + dy)) {
                                pixel = 0x00U;
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

static bool install_font(void)
{
    uint8_t glyph[54];
    for (uint8_t attempt = 0U; attempt < 3U; ++attempt) {
        bool all_glyphs_ready = true;
        for (size_t i = 0U; i < FLIGHTCODE_FONT_GLYPH_COUNT; ++i) {
            const flightcode_font_glyph_t *source =
                &flightcode_font_glyphs[i];
            build_glyph(source, glyph);
            if (glyph_matches(source->character, glyph)) continue;
            write_register(REG_CMAH, source->character);
            for (uint8_t byte = 0U; byte < 54U; ++byte) {
                write_register(REG_CMAL, byte);
                write_register(REG_CMDI, glyph[byte]);
            }
            write_register(REG_CMM, CMM_WRITE_NVM);
            HAL_Delay(1U);
            if (!wait_nvm()) {
                all_glyphs_ready = false;
                continue;
            }
            HAL_Delay(1U);
            if (!glyph_matches(source->character, glyph)) {
                all_glyphs_ready = false;
            }
        }
        if (all_glyphs_ready) return true;
        HAL_Delay(5U);
    }
    return false;
}

static uint8_t font_character(char character)
{
    const uint8_t code = (uint8_t)character;
    if (code >= FLIGHTCODE_GLYPH_CLOCK &&
        code <= FLIGHTCODE_GLYPH_BATTERY_THREE) {
        return code;
    }
    if (character == ' ' || character == '%' || character == '-' ||
        character == '.' || character == '/' || character == ':' || character == '<' ||
        character == '>' ||
        (character >= '0' && character <= '9') ||
        (character >= 'A' && character <= 'Z')) {
        return (uint8_t)character;
    }
    return (uint8_t)' ';
}

static uint8_t battery_glyph(float cell_voltage)
{
    if (cell_voltage >= 4.00f) return FLIGHTCODE_GLYPH_BATTERY_THREE;
    if (cell_voltage >= 3.75f) return FLIGHTCODE_GLYPH_BATTERY_TWO;
    if (cell_voltage >= 3.50f) return FLIGHTCODE_GLYPH_BATTERY_ONE;
    return FLIGHTCODE_GLYPH_BATTERY_EMPTY;
}

static void write_character(uint16_t position, uint8_t character)
{
    write_register(REG_DMAH, (uint8_t)(position >> 8U));
    write_register(REG_DMAL, (uint8_t)position);
    write_register(REG_DMDI, character);
}

static void reset_render_cache(void)
{
    render_cache_valid = false;
}

static uint32_t legacy_position(uint8_t position)
{
    static const uint8_t columns[3] = {1U, 11U, 22U};
    static const uint8_t pal_rows[3] = {1U, 7U, 14U};
    static const uint8_t ntsc_rows[3] = {1U, 6U, 11U};
    const uint8_t row = (video_pal ? pal_rows : ntsc_rows)[position / 3U];
    return (uint32_t)row * SCREEN_COLUMNS + columns[position % 3U];
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
    layout_positions[0] = legacy_position(selected_position);
    write_register(REG_DMM, DMM_CLEAR);
    HAL_Delay(1U);
    write_register(REG_VM0, video_mode | (enabled ? VM0_ENABLE : 0U));
    reset_render_cache();
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
    selected_position = position;
    layout_positions[0] = legacy_position(position);
    enabled = requested_enabled;
    reset_render_cache();
    if (!available) return true;
    if (!menu_active) {
        max7456_clear_screen();
        write_register(REG_VM0, (video_pal ? VM0_PAL : 0U) |
                                (enabled ? VM0_ENABLE : 0U));
    }
    return true;
}

bool max7456_set_layout(uint32_t enabled_mask,
                        const uint32_t positions[OSD_ELEMENT_COUNT],
                        const char *requested_pilot_name)
{
    if (positions == NULL || requested_pilot_name == NULL ||
        enabled_mask >= (1U << OSD_ELEMENT_COUNT)) return false;
    for (uint8_t i = 0U; i < OSD_ELEMENT_COUNT; ++i) {
        if (positions[i] >= SCREEN_COLUMNS * SCREEN_ROWS) return false;
    }
    size_t pilot_length = 0U;
    while (pilot_length <= OSD_TEXT_LENGTH &&
           requested_pilot_name[pilot_length] != '\0') ++pilot_length;
    if (pilot_length > OSD_TEXT_LENGTH) return false;
    layout_enabled_mask = enabled_mask;
    memcpy(layout_positions, positions, sizeof(layout_positions));
    (void)snprintf(pilot_name, sizeof(pilot_name), "%s",
                   requested_pilot_name);
    reset_render_cache();
    if (available && !menu_active) max7456_clear_screen();
    return true;
}

static void compose_element(char screen[SCREEN_COLUMNS * SCREEN_ROWS],
                            uint8_t element, const char *value)
{
    if ((layout_enabled_mask & (1U << element)) == 0U) return;
    const uint32_t position = layout_positions[element];
    const uint8_t column = (uint8_t)(position % SCREEN_COLUMNS);
    for (uint8_t i = 0U; i < OSD_TEXT_LENGTH && value[i] != '\0' &&
         column + i < SCREEN_COLUMNS; ++i) {
        screen[position + i] = value[i];
    }
}

static void render_layout(const char *total_voltage, const char *cell_text,
                          const char *timer)
{
    char screen[SCREEN_COLUMNS * SCREEN_ROWS];
    memset(screen, ' ', sizeof(screen));
    compose_element(screen, 0U, total_voltage);
    compose_element(screen, 1U, cell_text);
    compose_element(screen, 2U, timer);
    compose_element(screen, 3U, "FLIGHTCODE");
    compose_element(screen, 4U, pilot_name);
    const flight_settings_t *settings = flight_settings_get();
    static const char bands[] = "ABEFRL";
    char vtx_channel[OSD_TEXT_LENGTH + 1U];
    char vtx_power[OSD_TEXT_LENGTH + 1U];
    (void)snprintf(vtx_channel, sizeof(vtx_channel), "VTX %c%lu",
                   bands[settings->vtx_band],
                   (unsigned long)(settings->vtx_channel + 1U));
    (void)snprintf(vtx_power, sizeof(vtx_power), "VTX %luMW",
                   (unsigned long)settings->vtx_power_mw);
    for (uint8_t item = 0U; item < 2U; ++item) {
        if ((settings->vtx_osd_enabled_mask & (1U << item)) == 0U) continue;
        const uint32_t position = settings->vtx_osd_positions[item];
        const uint8_t column = (uint8_t)(position % SCREEN_COLUMNS);
        const char *text = item == 0U ? vtx_channel : vtx_power;
        for (uint8_t i = 0U; i < OSD_TEXT_LENGTH && text[i] != '\0' &&
             column + i < SCREEN_COLUMNS; ++i) screen[position + i] = text[i];
    }

    for (uint16_t position = 0U; position < sizeof(screen); ++position) {
        const bool changed = render_cache_valid
            ? screen[position] != previous_screen[position]
            : screen[position] != ' ';
        if (changed) {
            write_character(position, font_character(screen[position]));
        }
    }
    memcpy(previous_screen, screen, sizeof(previous_screen));
    render_cache_valid = true;
}

void max7456_update(float voltage, bool armed, uint32_t now_us)
{
    if (!available || !enabled || menu_active) return;
    if (armed && !previous_armed) {
        flight_started_us = now_us;
        flight_duration_us = 0U;
        timer_started = true;
    } else if (armed && timer_started) {
        flight_duration_us = now_us - flight_started_us;
    }
    previous_armed = armed;

    char total_voltage[OSD_TEXT_LENGTH + 1U] = "";
    char cell_text[OSD_TEXT_LENGTH + 1U] = "";
    bool show_voltage = true;
    if (voltage >= 1.0f && voltage < 100.0f) {
        uint8_t candidate_cells = (uint8_t)ceilf(voltage / 4.25f);
        if (candidate_cells > 8U) candidate_cells = 8U;
        if (candidate_cells > detected_battery_cells) {
            detected_battery_cells = candidate_cells;
        }
        const float per_cell_voltage =
            voltage / (float)detected_battery_cells;
        const char icon = (char)battery_glyph(per_cell_voltage);
        const bool critical =
            per_cell_voltage < BATTERY_CRITICAL_CELL_VOLTAGE;
        if (critical) {
            show_voltage = ((now_us / BATTERY_BLINK_HALF_PERIOD_US) &
                            1U) == 0U;
        }
        total_voltage[0] = icon;
        cell_text[0] = icon;
        if (show_voltage) {
            (void)snprintf(&total_voltage[1], sizeof(total_voltage) - 1U,
                           "%.2fV", voltage);
            (void)snprintf(&cell_text[1], sizeof(cell_text) - 1U,
                           "%.2fV", per_cell_voltage);
        }
    } else {
        detected_battery_cells = 0U;
    }
    const uint32_t total_seconds = flight_duration_us / 1000000U;
    char timer[OSD_TEXT_LENGTH + 1U];
    const int timer_length = snprintf(timer, sizeof(timer), "%02lu:%02lu",
                   (unsigned long)((total_seconds / 60U) % 100U),
                   (unsigned long)(total_seconds % 60U));
    if (timer_length > 0 && timer_length < (int)OSD_TEXT_LENGTH) {
        timer[timer_length] = (char)FLIGHTCODE_GLYPH_CLOCK;
        timer[timer_length + 1] = '\0';
    }
    render_layout(total_voltage, cell_text, timer);
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
    reset_render_cache();
    write_register(REG_VM0, (video_pal ? VM0_PAL : 0U) |
                            (enabled ? VM0_ENABLE : 0U));
}
