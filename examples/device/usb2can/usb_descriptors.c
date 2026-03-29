#include <string.h>

#include "bsp/board_api.h"
#include "tusb.h"

#include "usb2can.h"

enum {
  ITF_NUM_GS_USB = 0,
  ITF_NUM_TOTAL,
};

enum {
  STRID_LANGID = 0,
  STRID_MANUFACTURER,
  STRID_PRODUCT,
  STRID_SERIAL,
  STRID_INTERFACE,
};

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_VENDOR_DESC_LEN)
#define EPNUM_GSUSB_OUT  0x01
#define EPNUM_GSUSB_IN   0x82

static const tusb_desc_device_t desc_device = {
  .bLength = sizeof(tusb_desc_device_t),
  .bDescriptorType = TUSB_DESC_DEVICE,
  .bcdUSB = 0x0200,
  .bDeviceClass = TUSB_CLASS_VENDOR_SPECIFIC,
  .bDeviceSubClass = 0,
  .bDeviceProtocol = 0,
  .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
  .idVendor = USB2CAN_USB_VID,
  .idProduct = USB2CAN_USB_PID,
  .bcdDevice = USB2CAN_USB_BCD,
  .iManufacturer = STRID_MANUFACTURER,
  .iProduct = STRID_PRODUCT,
  .iSerialNumber = STRID_SERIAL,
  .bNumConfigurations = 1,
};

static const uint8_t desc_configuration[] = {
  TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x80, 100),
  TUD_VENDOR_DESCRIPTOR(ITF_NUM_GS_USB, STRID_INTERFACE, EPNUM_GSUSB_OUT, EPNUM_GSUSB_IN, 64),
};

static const char *const string_desc_arr[] = {
  (const char[]) { 0x09, 0x04 },
  "TinyUSB",
  "TM4C USB2CAN",
  NULL,
  "gs_usb",
};

static uint16_t _desc_str[33];

uint8_t const *tud_descriptor_device_cb(void) {
  return (uint8_t const *) &desc_device;
}

uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
  (void) index;
  return desc_configuration;
}

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
  size_t chr_count;

  (void) langid;

  if (index == STRID_LANGID) {
    memcpy(&_desc_str[1], string_desc_arr[0], 2);
    chr_count = 1;
  } else if (index == STRID_SERIAL) {
    chr_count = board_usb_get_serial(&_desc_str[1], 32);
  } else {
    if (index >= (sizeof(string_desc_arr) / sizeof(string_desc_arr[0])) || string_desc_arr[index] == NULL) {
      return NULL;
    }

    chr_count = strlen(string_desc_arr[index]);
    if (chr_count > 32) {
      chr_count = 32;
    }

    for (size_t i = 0; i < chr_count; i++) {
      _desc_str[1 + i] = string_desc_arr[index][i];
    }
  }

  _desc_str[0] = (uint16_t) ((TUSB_DESC_STRING << 8) | ((2 * chr_count) + 2));
  return _desc_str;
}