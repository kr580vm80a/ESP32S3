#pragma once

#include <stdint.h>

#define REPORT_ID_KEYBOARD 1
#define REPORT_ID_MOUSE    2
#define REPORT_ID_WIN_ABS  3
#define REPORT_ID_MEDIA    4
#define REPORT_ID_MAC_ABS  5

// --- REPORT ID 1: Standard HID Keyboard (6KRO) ---
#define HID_REPORT_DESC_KEYBOARD \
    0x05, 0x01,                    /* Usage Page (Generic Desktop Ctrls) */ \
    0x09, 0x06,                    /* Usage (Keyboard) */ \
    0xA1, 0x01,                    /* Collection (Application) */ \
    0x85, REPORT_ID_KEYBOARD,      /*   Report ID (1) */ \
    0x05, 0x07,                    /*   Usage Page (Keyboard/Keypad) */ \
    0x19, 0xE0,                    /*   Usage Minimum (Keyboard LeftControl) */ \
    0x29, 0xE7,                    /*   Usage Maximum (Keyboard Right GUI) */ \
    0x15, 0x00,                    /*   Logical Minimum (0) */ \
    0x25, 0x01,                    /*   Logical Maximum (1) */ \
    0x75, 0x01,                    /*   Report Size (1) */ \
    0x95, 0x08,                    /*   Report Count (8 bits for modifiers) */ \
    0x81, 0x02,                    /*   Input (Data,Var,Abs) - Modifiers */ \
    0x95, 0x01,                    /*   Report Count (1) */ \
    0x75, 0x08,                    /*   Report Size (8) */ \
    0x81, 0x01,                    /*   Input (Const,Array,Abs) - Reserved byte */ \
    0x95, 0x05,                    /*   Report Count (5) */ \
    0x75, 0x01,                    /*   Report Size (1) */ \
    0x05, 0x08,                    /*   Usage Page (LEDs) */ \
    0x19, 0x01,                    /*   Usage Minimum (Num Lock) */ \
    0x29, 0x05,                    /*   Usage Maximum (Kana) */ \
    0x91, 0x02,                    /*   Output (Data,Var,Abs) - LEDs */ \
    0x95, 0x01,                    /*   Report Count (1) */ \
    0x75, 0x03,                    /*   Report Size (3) */ \
    0x91, 0x01,                    /*   Output (Const,Array,Abs) - Padding */ \
    0x95, 0x06,                    /*   Report Count (6) */ \
    0x75, 0x08,                    /*   Report Size (8) */ \
    0x15, 0x00,                    /*   Logical Minimum (0) */ \
    0x25, 0x65,                    /*   Logical Maximum (101 keys) */ \
    0x05, 0x07,                    /*   Usage Page (Keyboard/Keypad) */ \
    0x19, 0x00,                    /*   Usage Minimum (Reserved) */ \
    0x29, 0x65,                    /*   Usage Maximum (Keyboard Application) */ \
    0x81, 0x00,                    /*   Input (Data,Array,Abs) - 6 keycodes */ \
    0xC0                           /* End Collection */

// --- REPORT ID 2: Native 12-bit High-Resolution Mouse (Logitech Darkfield Standard) ---
#define HID_REPORT_DESC_MOUSE \
    0x05, 0x01,                    /* Usage Page (Generic Desktop Ctrls) */ \
    0x09, 0x02,                    /* Usage (Mouse) */ \
    0xA1, 0x01,                    /* Collection (Application) */ \
    0x85, REPORT_ID_MOUSE,         /*   Report ID (2) */ \
    0x09, 0x01,                    /*   Usage (Pointer) */ \
    0xA1, 0x00,                    /*   Collection (Physical) */ \
    0x05, 0x09,                    /*     Usage Page (Button) */ \
    0x19, 0x01,                    /*     Usage Minimum (0x01) */ \
    0x29, 0x05,                    /*     Usage Maximum (0x05) */ \
    0x15, 0x00,                    /*     Logical Minimum (0) */ \
    0x25, 0x01,                    /*     Logical Maximum (1) */ \
    0x95, 0x05,                    /*     Report Count (5) */ \
    0x75, 0x01,                    /*     Report Size (1) */ \
    0x81, 0x02,                    /*     Input (Data,Var,Abs) */ \
    0x95, 0x01,                    /*     Report Count (1) */ \
    0x75, 0x03,                    /*     Report Size (3) */ \
    0x81, 0x03,                    /*     Input (Const,Var,Abs) */ \
    0x05, 0x01,                    /*     Usage Page (Generic Desktop Ctrls) */ \
    0x09, 0x30,                    /*     Usage (X) */ \
    0x09, 0x31,                    /*     Usage (Y) */ \
    0x16, 0x01, 0xF8,              /*     Logical Minimum (-2047) */ \
    0x26, 0xFF, 0x07,              /*     Logical Maximum (2047) */ \
    0x75, 0x0C,                    /*     Report Size (12) */ \
    0x95, 0x02,                    /*     Report Count (2: X, Y) */ \
    0x81, 0x06,                    /*     Input (Data,Var,Rel) */ \
    0x09, 0x38,                    /*     Usage (Wheel) */ \
    0x15, 0x81,                    /*     Logical Minimum (-127) */ \
    0x25, 0x7F,                    /*     Logical Maximum (127) */ \
    0x75, 0x08,                    /*     Report Size (8) */ \
    0x95, 0x01,                    /*     Report Count (1: Wheel) */ \
    0x81, 0x06,                    /*     Input (Data,Var,Rel) */ \
    0x05, 0x0C,                    /*     Usage Page (Consumer) */ \
    0x0A, 0x38, 0x02,              /*     Usage (AC Pan) */ \
    0x15, 0x81,                    /*     Logical Minimum (-127) */ \
    0x25, 0x7F,                    /*     Logical Maximum (127) */ \
    0x75, 0x08,                    /*     Report Size (8) */ \
    0x95, 0x01,                    /*     Report Count (1: AC Pan) */ \
    0x81, 0x06,                    /*     Input (Data,Var,Rel) */ \
    0xC0,                          /*   End Collection */ \
    0xC0                           /* End Collection */

// --- REPORT ID 3: Absolute Pointer (for Windows / Android multi-monitor transitions) ---
#define HID_REPORT_DESC_WIN_ABS \
    0x05, 0x01,                    /* Usage Page (Generic Desktop Ctrls) */ \
    0x09, 0x01,                    /* Usage (Pointer) */ \
    0xA1, 0x01,                    /* Collection (Application) */ \
    0x85, REPORT_ID_WIN_ABS,       /*   Report ID (3) */ \
    0x05, 0x01,                    /*   Usage Page (Generic Desktop Ctrls) */ \
    0x09, 0x30,                    /*   Usage (X) */ \
    0x09, 0x31,                    /*   Usage (Y) */ \
    0x16, 0x00, 0x00,              /*   Logical Minimum (0) */ \
    0x26, 0xFF, 0x7F,              /*   Logical Maximum (32767) */ \
    0x75, 0x10,                    /*   Report Size (16) */ \
    0x95, 0x02,                    /*   Report Count (2: X, Y) */ \
    0x81, 0x02,                    /*   Input (Data,Var,Abs) */ \
    0xC0                           /* End Collection */

// --- REPORT ID 5: Absolute Digitizer Pen (for macOS / iPadOS transitions) ---
#define HID_REPORT_DESC_MAC_ABS \
    0x05, 0x0D,                    /* Usage Page (Digitizers) */ \
    0x09, 0x02,                    /* Usage (Pen) */ \
    0xA1, 0x01,                    /* Collection (Application) */ \
    0x85, REPORT_ID_MAC_ABS,       /*   Report ID (5) */ \
    0x09, 0x32,                    /*   Usage (In Range) */ \
    0x15, 0x00,                    /*   Logical Minimum (0) */ \
    0x25, 0x01,                    /*   Logical Maximum (1) */ \
    0x75, 0x01,                    /*   Report Size (1) */ \
    0x95, 0x01,                    /*   Report Count (In Range) */ \
    0x81, 0x02,                    /*   Input (Data,Var,Abs) */ \
    0x75, 0x07,                    /*   Report Size (7) */ \
    0x95, 0x01,                    /*   Report Count (1: Padding) */ \
    0x81, 0x03,                    /*   Input (Const,Var,Abs) */ \
    0x05, 0x01,                    /*   Usage Page (Generic Desktop Ctrls) */ \
    0x09, 0x30,                    /*   Usage (X) */ \
    0x09, 0x31,                    /*   Usage (Y) */ \
    0x16, 0x00, 0x00,              /*   Logical Minimum (0) */ \
    0x26, 0xFF, 0x7F,              /*   Logical Maximum (32767) */ \
    0x36, 0x00, 0x00,              /*   Physical Minimum (0) */ \
    0x46, 0xFF, 0x7F,              /*   Physical Maximum (32767) */ \
    0x75, 0x10,                    /*   Report Size (16) */ \
    0x95, 0x02,                    /*   Report Count (2: X, Y) */ \
    0x81, 0x02,                    /*   Input (Data,Var,Abs) */ \
    0xC0                           /* End Collection */

// --- REPORT ID 4: Consumer Control (Media Keys) ---
#define HID_REPORT_DESC_MEDIA \
    0x05, 0x0C,                    /* Usage Page (Consumer) */ \
    0x09, 0x01,                    /* Usage (Consumer Control) */ \
    0xA1, 0x01,                    /* Collection (Application) */ \
    0x85, REPORT_ID_MEDIA,         /*   Report ID (4) */ \
    0x15, 0x00,                    /*   Logical Minimum (0) */ \
    0x26, 0xFF, 0x03,              /*   Logical Maximum (1023) */ \
    0x19, 0x00,                    /*   Usage Minimum (Unassigned) */ \
    0x2A, 0xFF, 0x03,              /*   Usage Maximum (1023) */ \
    0x75, 0x10,                    /*   Report Size (16) */ \
    0x95, 0x01,                    /*   Report Count (1) */ \
    0x81, 0x00,                    /*   Input (Data,Array,Abs) */ \
    0xC0                           /* End Collection */

// --- Full HID Report Map for NimBLE BLE Server ---
const uint8_t hidReportMap[] = {
    HID_REPORT_DESC_KEYBOARD,
    HID_REPORT_DESC_MOUSE,
    HID_REPORT_DESC_WIN_ABS,
    HID_REPORT_DESC_MAC_ABS,
    HID_REPORT_DESC_MEDIA
};

// --- Custom Composite Descriptor for USB TinyUSB Absolute Pointer (ID 3) & Digitizer (ID 5) ---
static const uint8_t usb_custom_abs_report_descriptor[] = {
    HID_REPORT_DESC_WIN_ABS,
    HID_REPORT_DESC_MAC_ABS
};
