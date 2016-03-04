#include "usb_api.h"
#include "usb_descriptor_keyboard_report.h"

uint8_t UsbKeyboardReportDescriptor[USB_KEYBOARD_REPORT_DESCRIPTOR_LENGTH] = {
    HID_RI_USAGE_PAGE(8, HID_RI_USAGE_PAGE_GENERIC_DESKTOP),
    HID_RI_USAGE(8, HID_RI_USAGE_GENERIC_DESKTOP_KEYBOARD),
    HID_RI_COLLECTION(8, HID_RI_COLLECTION_APPLICATION),

        // Modifiers
        HID_RI_USAGE_PAGE(8, HID_RI_USAGE_PAGE_KEY_CODES),
        HID_RI_USAGE_MINIMUM(8, 0xE0),
        HID_RI_USAGE_MAXIMUM(8, 0xE7),
        HID_RI_LOGICAL_MINIMUM(8, 0x00),
        HID_RI_LOGICAL_MAXIMUM(8, 0x01),
        HID_RI_REPORT_SIZE(8, 0x01),
        HID_RI_REPORT_COUNT(8, 0x08),
        HID_RI_INPUT(8, HID_IOF_DATA | HID_IOF_VARIABLE | HID_IOF_ABSOLUTE),

        // Reserved for future use, always 0
        HID_RI_REPORT_COUNT(8, 0x01),
        HID_RI_REPORT_SIZE(8, 0x08),
        HID_RI_INPUT(8, HID_IOF_CONSTANT),

        // LED status
        HID_RI_USAGE_PAGE(8, HID_RI_USAGE_PAGE_LEDS),
        HID_RI_USAGE_MINIMUM(8, 0x01),
        HID_RI_USAGE_MAXIMUM(8, 0x05),
        HID_RI_REPORT_COUNT(8, 0x05),
        HID_RI_REPORT_SIZE(8, 0x01),
        HID_RI_OUTPUT(8, HID_IOF_DATA | HID_IOF_VARIABLE | HID_IOF_ABSOLUTE | HID_IOF_NON_VOLATILE),

        // LED status padding
        HID_RI_REPORT_COUNT(8, 0x01),
        HID_RI_REPORT_SIZE(8, 0x03),
        HID_RI_OUTPUT(8, HID_IOF_CONSTANT),

        // Scancodes
        HID_RI_LOGICAL_MINIMUM(8, 0x00),
        HID_RI_LOGICAL_MAXIMUM(8, 0xFF),
        HID_RI_USAGE_PAGE(8, HID_RI_USAGE_PAGE_KEY_CODES),
        HID_RI_USAGE_MINIMUM(8, 0x00),
        HID_RI_USAGE_MAXIMUM(8, 0xFF),
        HID_RI_REPORT_COUNT(8, USB_KEYBOARD_MAX_KEYS),
        HID_RI_REPORT_SIZE(8, 0x08),
        HID_RI_INPUT(8, HID_IOF_DATA | HID_IOF_ARRAY | HID_IOF_ABSOLUTE),

    HID_RI_END_COLLECTION(0),
};
