#include "usbd_desc.h"

#if defined(PLATFORM_STM32H7)
#include "stm32h7xx_hal.h"
#else
#include "stm32f4xx_hal.h"
#endif
#include "usbd_core.h"

#define USB_VID 0x0483U
#define USB_PID 0x5740U
#define USB_LANGID 0x0409U
#define SERIAL_DESC_SIZE 26U

static uint8_t device_descriptor[USB_LEN_DEV_DESC] = {
    0x12, USB_DESC_TYPE_DEVICE, 0x00, 0x02,
    0x02, 0x02, 0x00, USB_MAX_EP0_SIZE,
    LOBYTE(USB_VID), HIBYTE(USB_VID),
    LOBYTE(USB_PID), HIBYTE(USB_PID),
    0x00, 0x01,
    USBD_IDX_MFC_STR, USBD_IDX_PRODUCT_STR, USBD_IDX_SERIAL_STR, 0x01
};
static uint8_t lang_descriptor[USB_LEN_LANGID_STR_DESC] = {
    USB_LEN_LANGID_STR_DESC, USB_DESC_TYPE_STRING,
    LOBYTE(USB_LANGID), HIBYTE(USB_LANGID)
};
static uint8_t string_descriptor[USBD_MAX_STR_DESC_SIZ];
static uint8_t serial_descriptor[SERIAL_DESC_SIZE] = {
    SERIAL_DESC_SIZE, USB_DESC_TYPE_STRING
};

static void hex_unicode(uint32_t value, uint8_t *buffer, uint8_t digits)
{
    for (uint8_t i = 0U; i < digits; ++i) {
        const uint8_t nibble = (uint8_t)(value >> 28);
        uint8_t character = (uint8_t)'0' + nibble;
        if (nibble >= 10U) {
            character = (uint8_t)((uint8_t)'A' + nibble - 10U);
        }
        buffer[i * 2U] = character;
        buffer[i * 2U + 1U] = 0U;
        value <<= 4;
    }
}

static uint8_t *device(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    *length = sizeof(device_descriptor);
    return device_descriptor;
}
static uint8_t *lang(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    *length = sizeof(lang_descriptor);
    return lang_descriptor;
}
static uint8_t *manufacturer(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)"FlightCode", string_descriptor, length);
    return string_descriptor;
}
static uint8_t *product(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)"FlightCode USB Configurator", string_descriptor, length);
    return string_descriptor;
}
static uint8_t *serial(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    const uint32_t uid0 = *(const uint32_t *)UID_BASE;
    const uint32_t uid1 = *(const uint32_t *)(UID_BASE + 4U);
    const uint32_t uid2 = *(const uint32_t *)(UID_BASE + 8U);
    hex_unicode(uid0 + uid2, &serial_descriptor[2], 8U);
    hex_unicode(uid1, &serial_descriptor[18], 4U);
    *length = sizeof(serial_descriptor);
    return serial_descriptor;
}
static uint8_t *configuration(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)"FlightCode CDC", string_descriptor, length);
    return string_descriptor;
}
static uint8_t *interface(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)"FlightCode Serial", string_descriptor, length);
    return string_descriptor;
}

USBD_DescriptorsTypeDef FlightCode_Desc = {
    device, lang, manufacturer, product, serial, configuration, interface
};
