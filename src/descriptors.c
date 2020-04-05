/* SPDX-License-Identifier: MIT */

#include "tusb.h"

#define _PID_MAP(itf, n)  ( (CFG_TUD_##itf) << (n) )
#define USB_PID           (0x4000 | _PID_MAP(CDC, 0) | _PID_MAP(MSC, 1) | _PID_MAP(HID, 2) | \
                           _PID_MAP(MIDI, 3) | _PID_MAP(VENDOR, 4) )

static tusb_desc_device_t const desc_device = {
        .bLength            = sizeof(tusb_desc_device_t),
        .bDescriptorType    = TUSB_DESC_DEVICE,
        .bcdUSB             = 0x0200,

        // the following three are needed for a USB composite device
        .bDeviceClass       = TUSB_CLASS_MISC,
        .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
        .bDeviceProtocol    = MISC_PROTOCOL_IAD,

        .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

        .idVendor           = 0xf055,
        .idProduct          = USB_PID,
        .bcdDevice          = 0x0100,

        .iManufacturer      = 0x01,
        .iProduct           = 0x02,
        .iSerialNumber      = 0x03,

        .bNumConfigurations = 0x01
};

uint8_t const * tud_descriptor_device_cb(void) {
    return (uint8_t const *) &desc_device;
}

//////////////////////////////////////////////////////////////////////////////////////////////////

static uint8_t const desc_hid_report[] = {
        HID_USAGE_PAGE ( HID_USAGE_PAGE_DESKTOP     )        ,\
  HID_USAGE      ( HID_USAGE_DESKTOP_GAMEPAD  )        ,\
  HID_COLLECTION ( HID_COLLECTION_APPLICATION )        ,\
    /* 16 bit Button Map */ \
    HID_USAGE_PAGE   ( HID_USAGE_PAGE_BUTTON                  ) ,\
    HID_USAGE_MIN    ( 1                                      ) ,\
    HID_USAGE_MAX    ( 16                                     ) ,\
    HID_LOGICAL_MIN  ( 0                                      ) ,\
    HID_LOGICAL_MAX  ( 1                                      ) ,\
    HID_PHYSICAL_MIN (0), \
    HID_PHYSICAL_MAX (1), \
    HID_REPORT_COUNT ( 16                                     ) ,\
    HID_REPORT_SIZE  ( 1                                      ) ,\
    HID_INPUT        ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE    ) ,\
    /* X, Y, RX, RY, RZ (min -127, max 127 ) */ \
    HID_USAGE_PAGE   ( HID_USAGE_PAGE_DESKTOP                 ) ,\
    HID_LOGICAL_MIN  ( 0x81                                   ) ,\
    HID_LOGICAL_MAX  ( 0x7f                                   ) ,\
    HID_USAGE        ( HID_USAGE_DESKTOP_X                    ) ,\
    HID_USAGE        ( HID_USAGE_DESKTOP_Y                    ) ,\
    HID_USAGE        ( HID_USAGE_DESKTOP_RX                   ) ,\
    HID_USAGE        ( HID_USAGE_DESKTOP_RY                   ) ,\
    HID_USAGE        ( HID_USAGE_DESKTOP_RZ                   ) ,\
    HID_REPORT_COUNT ( 5                                      ) ,\
    HID_REPORT_SIZE  ( 8                                     ) ,\
    HID_INPUT        ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE ) ,\

        HID_USAGE_PAGE   ( HID_USAGE_PAGE_DESKTOP                 ) ,\
    HID_LOGICAL_MIN  ( 0x0                                   ) ,\
    HID_LOGICAL_MAX  ( 0x1                                   ) ,\
    HID_PHYSICAL_MIN (0), \
    HID_PHYSICAL_MAX (1), \
    HID_USAGE        ( HID_USAGE_DESKTOP_DPAD_UP                   ) ,\
    HID_USAGE        ( HID_USAGE_DESKTOP_DPAD_DOWN                    ) ,\
    HID_USAGE        ( HID_USAGE_DESKTOP_DPAD_LEFT                   ) ,\
    HID_USAGE        ( HID_USAGE_DESKTOP_DPAD_RIGHT                   ) ,\
    HID_REPORT_COUNT ( 4                                      ) ,\
    HID_REPORT_SIZE  ( 1                                     ) ,\
    HID_INPUT        ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE ) ,\
    HID_REPORT_COUNT (1), \
    HID_REPORT_SIZE  (4), \
    HID_INPUT        ( HID_CONSTANT | HID_VARIABLE | HID_ABSOLUTE), \
  HID_COLLECTION_END \
};

uint8_t const * tud_hid_descriptor_report_cb(void) {
    return desc_hid_report;
}

//////////////////////////////////////////////////////////////////////////////////////////////////

enum
{
    ITF_NUM_CDC = 0,
    ITF_NUM_CDC_DATA,
    ITF_NUM_HID,
    ITF_NUM_TOTAL
};

#define EPNUM_HID 0x03

#define CONFIG_TOTAL_LEN    (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_HID_INOUT_DESC_LEN )

static uint8_t const desc_configuration[] = {
        // Interface count, string index, total length, attribute, power in mA
        TUD_CONFIG_DESCRIPTOR(ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 500),

        // Interface number, string index, EP notification address and size, EP data address (out, in) and size.
        TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4, 0x81, 8, 0x02, 0x82, 64),

        // Interface number, string index, protocol, report descriptor len, EP In & Out address, size & polling interval
        TUD_HID_INOUT_DESCRIPTOR(ITF_NUM_HID, 0, HID_PROTOCOL_NONE, sizeof(desc_hid_report), EPNUM_HID, 0x80 | EPNUM_HID, CFG_TUD_HID_BUFSIZE, 2)
};

uint8_t const * tud_descriptor_configuration_cb(uint8_t index) {
    (void) index; // for multiple configurations
    return desc_configuration;
}

//////////////////////////////////////////////////////////////////////////////////////////////////

static char const* string_desc_arr [] = {
        (const char[]) { 0x09, 0x04 }, // 0: is supported language is English (0x0409)
        "Herbert Engineering",                     // 1: Manufacturer
        "turtleboard",              // 2: Product
        "123456",                      // 3: Serials, should use chip ID
        "turtleboard CDC",                 // 4: CDC Interface
};

static uint16_t string_descriptor_buf[32];

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {

    (void)langid; // ignore arg

    uint8_t chr_count;

    if ( index == 0)
    {
        memcpy(&string_descriptor_buf[1], string_desc_arr[0], 2);
        chr_count = 1;
    }else
    {
        // Convert ASCII string into UTF-16

        if ( !(index < sizeof(string_desc_arr)/sizeof(string_desc_arr[0])) ) return NULL;

        const char* str = string_desc_arr[index];

        // Cap at max char
        chr_count = strlen(str);
        if ( chr_count > 31 ) chr_count = 31;

        for(uint8_t i=0; i<chr_count; i++)
        {
            string_descriptor_buf[1+i] = str[i];
        }
    }

    // first byte is length (including header), second byte is string type
    string_descriptor_buf[0] = (TUSB_DESC_STRING << 8 ) | (2*chr_count + 2);

    return string_descriptor_buf;
}
