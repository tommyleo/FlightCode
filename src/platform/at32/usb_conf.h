#pragma once
#include <stddef.h>
#include "at32f435_437.h"
#define USE_OTG_DEVICE_MODE
#define USB_VBUS_IGNORE
#define USB_EPT_MAX_NUM 8
#define USBD_RX_SIZE 128
#define USBD_EP0_TX_SIZE 24
#define USBD_EP1_TX_SIZE 80
#define USBD_EP2_TX_SIZE 20
#define USBD_EP3_TX_SIZE 20
#define USBD_EP4_TX_SIZE 20
#define USBD_EP5_TX_SIZE 20
#define USBD_EP6_TX_SIZE 20
#define USBD_EP7_TX_SIZE 20
#define USBD2_RX_SIZE 128
#define USBD2_EP0_TX_SIZE 24
#define USBD2_EP1_TX_SIZE 80
#define USBD2_EP2_TX_SIZE 20
#define USBD2_EP3_TX_SIZE 20
#define USBD2_EP4_TX_SIZE 20
#define USBD2_EP5_TX_SIZE 20
#define USBD2_EP6_TX_SIZE 20
#define USBD2_EP7_TX_SIZE 20
void usb_delay_ms(uint32_t ms);
void usb_delay_us(uint32_t us);
