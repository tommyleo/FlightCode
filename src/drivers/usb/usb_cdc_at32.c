#include "usb_cdc.h"
#include "board.h"
#include "usb_core.h"
#include "usbd_int.h"
#include "cdc_class.h"
#include "cdc_desc.h"
#include <string.h>

#define RX_SIZE 512U
#define TX_SIZE 8192U
static otg_core_type core;
static uint8_t rx_ring[RX_SIZE], tx_ring[TX_SIZE];
static uint16_t rx_head, rx_tail, tx_head, tx_tail;
static uint8_t rx_packet[64] __attribute__((aligned(4)));
static uint8_t tx_packet[64] __attribute__((aligned(4)));
static bool started, needs_zlp;

void usb_delay_ms(uint32_t ms) { HAL_Delay(ms); }
void usb_delay_us(uint32_t us)
{
    const uint32_t start = board_micros();
    while ((uint32_t)(board_micros() - start) < us) __NOP();
}
void OTGFS1_IRQHandler(void) { usbd_irq_handler(&core); }
void usb_cdc_init(void)
{
    crm_periph_clock_enable(CRM_OTGFS1_PERIPH_CLOCK, TRUE);
    at32_gpio_config(GPIOA, GPIO_PIN_11 | GPIO_PIN_12, GPIO_MODE_MUX, GPIO_PULL_NONE, GPIO_MUX_10);
    rx_head = rx_tail = tx_head = tx_tail = 0;
    needs_zlp = false;
    NVIC_SetPriority(OTGFS1_IRQn, 6U);
    NVIC_ClearPendingIRQ(OTGFS1_IRQn);
    if (usbd_init(&core, USB_FULL_SPEED_CORE_ID, 0, &cdc_class_handler, &cdc_desc_handler) != USB_OK)
        return;
    started = true;
    NVIC_EnableIRQ(OTGFS1_IRQn);
}
void usb_cdc_deinit(void)
{
    if (!started) return;
    NVIC_DisableIRQ(OTGFS1_IRQn);
    usbd_disconnect(&core.dev);
    crm_periph_clock_enable(CRM_OTGFS1_PERIPH_CLOCK, FALSE);
    started = false;
}
static void receive_packet(void)
{
    /* Vendor CDC NAKs the endpoint until the packet is consumed. Reserve room
     * for a whole packet so command lines are never silently truncated. */
    if ((RX_SIZE - 1U - ((rx_head - rx_tail) & (RX_SIZE - 1U))) < sizeof(rx_packet)) return;
    const uint16_t n = usb_vcp_get_rxdata(&core.dev, rx_packet);
    for (uint16_t i = 0; i < n; ++i) {
        rx_ring[rx_head] = rx_packet[i];
        rx_head = (rx_head + 1U) & (RX_SIZE - 1U);
    }
}
size_t usb_cdc_read(uint8_t *data, size_t capacity)
{
    if (started && core.dev.conn_state == USB_CONN_STATE_CONFIGURED) receive_packet();
    size_t n = 0;
    while (n < capacity && rx_tail != rx_head) {
        data[n++] = rx_ring[rx_tail];
        rx_tail = (rx_tail + 1U) & (RX_SIZE - 1U);
    }
    return n;
}
size_t usb_cdc_write(const uint8_t *data, size_t length)
{
    const size_t available = TX_SIZE - 1U - ((tx_head - tx_tail) & (TX_SIZE - 1U));
    if (length > available) { usb_cdc_poll(); return 0; }
    for (size_t i = 0; i < length; ++i) {
        tx_ring[tx_head] = data[i];
        tx_head = (tx_head + 1U) & (TX_SIZE - 1U);
    }
    usb_cdc_poll();
    return length;
}
void usb_cdc_poll(void)
{
    if (!started || core.dev.conn_state != USB_CONN_STATE_CONFIGURED) return;
    receive_packet();
    cdc_struct_type *cdc = (cdc_struct_type *)core.dev.class_handler->pdata;
    if (!cdc->g_tx_completed) return;
    uint16_t tail = tx_tail, n = 0;
    while (tail != tx_head && n < sizeof(tx_packet)) {
        tx_packet[n++] = tx_ring[tail];
        tail = (tail + 1U) & (TX_SIZE - 1U);
    }
    if (n == 0 && !needs_zlp) return;
    if (usb_vcp_send_data(&core.dev, tx_packet, n) == SUCCESS) {
        tx_tail = tail;
        needs_zlp = n == sizeof(tx_packet);
    }
}
