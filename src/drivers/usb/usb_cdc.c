#include "usb_cdc.h"

#include <stdbool.h>
#include <string.h>

#include "usbd_cdc.h"
#include "usbd_core.h"
#include "usbd_desc.h"

#define RX_USB_SIZE 64U
#define RX_RING_SIZE 512U
#define TX_RING_SIZE 1024U

static USBD_HandleTypeDef usb_device;
static uint8_t usb_rx[RX_USB_SIZE];
static uint8_t rx_ring[RX_RING_SIZE];
static volatile uint16_t rx_head, rx_tail;
static uint8_t tx_ring[TX_RING_SIZE];
static volatile uint16_t tx_head, tx_tail;
static uint8_t tx_packet[CDC_DATA_FS_MAX_PACKET_SIZE];
static volatile bool tx_busy;

static int8_t cdc_init(void)
{
    USBD_CDC_SetRxBuffer(&usb_device, usb_rx);
    return USBD_CDC_ReceivePacket(&usb_device);
}
static int8_t cdc_deinit(void) { return USBD_OK; }
static int8_t cdc_control(uint8_t command, uint8_t *buffer, uint16_t length)
{
    (void)command; (void)buffer; (void)length;
    return USBD_OK;
}
static int8_t cdc_receive(uint8_t *buffer, uint32_t *length)
{
    for (uint32_t i = 0U; i < *length; ++i) {
        const uint16_t next = (uint16_t)((rx_head + 1U) % RX_RING_SIZE);
        if (next != rx_tail) {
            rx_ring[rx_head] = buffer[i];
            rx_head = next;
        }
    }
    USBD_CDC_SetRxBuffer(&usb_device, usb_rx);
    USBD_CDC_ReceivePacket(&usb_device);
    return USBD_OK;
}
static int8_t cdc_transmit_complete(uint8_t *buffer, uint32_t *length, uint8_t ep)
{
    (void)buffer; (void)length; (void)ep;
    tx_busy = false;
    return USBD_OK;
}
static USBD_CDC_ItfTypeDef cdc_interface = {
    cdc_init, cdc_deinit, cdc_control, cdc_receive, cdc_transmit_complete
};

void usb_cdc_init(void)
{
    if (USBD_Init(&usb_device, &FlightCode_Desc, 0U) != USBD_OK) return;
    if (USBD_RegisterClass(&usb_device, &USBD_CDC) != USBD_OK) return;
    if (USBD_CDC_RegisterInterface(&usb_device, &cdc_interface) != USBD_OK) return;
    USBD_Start(&usb_device);
}

void usb_cdc_deinit(void)
{
    USBD_Stop(&usb_device);
    USBD_DeInit(&usb_device);
}

size_t usb_cdc_read(uint8_t *data, size_t capacity)
{
    size_t count = 0U;
    while (count < capacity && rx_tail != rx_head) {
        data[count++] = rx_ring[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1U) % RX_RING_SIZE);
    }
    return count;
}

size_t usb_cdc_write(const uint8_t *data, size_t length)
{
    const size_t used = tx_head >= tx_tail
        ? (size_t)(tx_head - tx_tail)
        : (size_t)(TX_RING_SIZE - tx_tail + tx_head);
    const size_t available = TX_RING_SIZE - 1U - used;
    /* Protocol messages are line based: never enqueue a truncated line. */
    if (length > available) {
        usb_cdc_poll();
        return 0U;
    }
    size_t count = 0U;
    while (count < length) {
        const uint16_t next = (uint16_t)((tx_head + 1U) % TX_RING_SIZE);
        if (next == tx_tail) break;
        tx_ring[tx_head] = data[count++];
        tx_head = next;
    }
    usb_cdc_poll();
    return count;
}

void usb_cdc_poll(void)
{
    if (tx_busy || tx_tail == tx_head ||
        usb_device.dev_state != USBD_STATE_CONFIGURED) {
        return;
    }
    uint16_t length = 0U;
    while (tx_tail != tx_head && length < sizeof(tx_packet)) {
        tx_packet[length++] = tx_ring[tx_tail];
        tx_tail = (uint16_t)((tx_tail + 1U) % TX_RING_SIZE);
    }
    tx_busy = true;
    USBD_CDC_SetTxBuffer(&usb_device, tx_packet, length);
    if (USBD_CDC_TransmitPacket(&usb_device) != USBD_OK) {
        tx_busy = false;
    }
}
