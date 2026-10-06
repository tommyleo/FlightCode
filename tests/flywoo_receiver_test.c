#include "flywoo_uart_stubs.inc"
#include "flywoo_board_under_test.inc"
#define BOARD_HAS_SELECTABLE_RECEIVER_UART 0
#define VTX_PROTOCOL_OFF 0U
#define VTX_PROTOCOL_HDZERO_MSP 3U
#define RECEIVER_PROTOCOL_SBUS 0U
#define RECEIVER_PROTOCOL_CRSF 1U
typedef struct { uint32_t vtx_uart, vtx_protocol, receiver_protocol, tuning; } flight_settings_t;
typedef enum { VTX_UART_OK, VTX_UART_UNAVAILABLE, VTX_UART_RECEIVER_CONFLICT,
               VTX_UART_INVERTED_SBUS_ONLY } vtx_uart_status_t;
#include "crsf_bind.h"
#include "flywoo_uart_under_test.inc"
#include "flywoo_vtx_under_test.inc"
#if defined(BOARD_FLYWOOF405NANO)
#define SETTINGS_MAGIC 0x12345678U
#define SETTINGS_VERSION 28U
typedef struct { uint32_t magic, version; flight_settings_t settings; uint32_t checksum; } settings_record_t;
static uint32_t checksum(const settings_record_t *r)
{
    return r->magic ^ r->version ^ r->settings.vtx_uart ^ r->settings.vtx_protocol ^
        r->settings.receiver_protocol ^ r->settings.tuning;
}
static bool migrate(const settings_record_t *stored, settings_record_t *result)
{
#include "flywoo_migration_under_test.inc"
    *result = *stored;
    return migrated_uart6;
}
#endif
int main(void)
{
    assert(board_receiver_uart_configure(false, 0));
    assert(hsbus_uart.Instance == UART5 && enabled_irq == 5);
    assert(pad_port == GPIOD && pad == GPIO_PIN_2);
    assert(board_receiver_uart_configure(true, 0));
    assert(disabled_irq == 5 && hsbus_uart.Init.BaudRate == 420000);
    assert(hsbus_uart.Init.WordLength == 8 && hsbus_uart.Init.Parity == 0);
#if defined(BOARD_FLYWOOF405NANO)
    assert(hsbus_uart.Instance == USART6 && enabled_irq == 6);
    assert(pad_port == GPIOC && pad == GPIO_PIN_7 && af == 8);
    assert(strcmp(CRSF_UART_NAME, "UART6") == 0);
    expected_tx_port = GPIOC; expected_tx_pin = GPIO_PIN_6;
    assert(BOARD_DEFAULT_RECEIVER_PROTOCOL == RECEIVER_PROTOCOL_CRSF);
    assert(BOARD_DEFAULT_VTX_UART == 4);
#else
    assert(hsbus_uart.Instance == UART4 && enabled_irq == 4);
    assert(pad_port == GPIOA && pad == GPIO_PIN_1 && af == 8);
    assert(strcmp(CRSF_UART_NAME, "UART4") == 0);
    expected_tx_port = GPIOA; expected_tx_pin = GPIO_PIN_0;
#endif
    hsbus_uart.Instance->CR1 = 4;
    transmit_status = HAL_OK;
    assert(board_receiver_uart_transmit(crsf_bind_frame, sizeof(crsf_bind_frame)));
    assert(hsbus_uart.Instance->CR1 == 4 && mode == GPIO_MODE_INPUT);
    transmit_status = 1;
    assert(!board_receiver_uart_transmit(crsf_bind_frame, sizeof(crsf_bind_frame)));
    assert(hsbus_uart.Instance->CR1 == 4);
    flight_settings_t settings = {6, VTX_PROTOCOL_HDZERO_MSP, RECEIVER_PROTOCOL_CRSF, 135};
#if defined(BOARD_FLYWOOF405NANO)
    settings_record_t old = {SETTINGS_MAGIC, SETTINGS_VERSION, settings, 0}, result;
    old.checksum = checksum(&old);
    assert(migrate(&old, &result));
    assert(result.settings.vtx_uart == 4 && result.settings.vtx_protocol == VTX_PROTOCOL_HDZERO_MSP);
    assert(result.settings.tuning == 135 && result.settings.receiver_protocol == RECEIVER_PROTOCOL_CRSF);
    assert(result.checksum == checksum(&result));
    old.checksum ^= 1;
    assert(!migrate(&old, &result));
    assert(result.settings.vtx_uart == 6); /* Corrupted records are not repaired. */
    assert(flight_settings_vtx_uart_status(&settings) == VTX_UART_RECEIVER_CONFLICT);
    settings.receiver_protocol = RECEIVER_PROTOCOL_SBUS;
    assert(flight_settings_vtx_uart_status(&settings) == VTX_UART_RECEIVER_CONFLICT);
    settings.vtx_uart = 4;
    assert(flight_settings_vtx_uart_status(&settings) == VTX_UART_OK);
#else
    assert(flight_settings_vtx_uart_status(&settings) == VTX_UART_OK);
    settings.vtx_uart = 4;
    assert(flight_settings_vtx_uart_status(&settings) == VTX_UART_RECEIVER_CONFLICT);
#endif
    puts("Flywoo UART RX, bind TX, RX preservation and VTX conflicts: PASS");
}
