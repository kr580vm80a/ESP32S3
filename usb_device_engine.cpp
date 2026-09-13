#include "usb_device_engine.h"

#if CONFIG_IDF_TARGET_ESP32S3
#include "USB.h"
#include "USBHIDMouse.h"
#include "USBHIDKeyboard.h"
#include "USBHIDConsumerControl.h"
#include "hid_descriptors.h"

void logPrint(const char* format, ...);

static USBHIDMouse s_usbMouse;
static USBHIDKeyboard s_usbKeyboard;
static USBHIDConsumerControl s_usbConsumer;

class USBHIDAbsolute : public USBHIDDevice {
private:
    USBHID hid;
public:
    USBHIDAbsolute() : hid() {
        static bool initialized = false;
        if (!initialized) {
            initialized = true;
            hid.addDevice(this, sizeof(usb_custom_abs_report_descriptor));
        }
    }
    void begin() {
        hid.begin();
    }
    void end() {}
    uint16_t _onGetDescriptor(uint8_t* dst) {
        memcpy(dst, usb_custom_abs_report_descriptor, sizeof(usb_custom_abs_report_descriptor));
        return sizeof(usb_custom_abs_report_descriptor);
    }
    bool sendWindowsAbs(const uint8_t* rep4) {
        return hid.SendReport(REPORT_ID_WIN_ABS, rep4, 4);
    }
    bool sendMacDigitizer(const uint8_t* rep5) {
        return hid.SendReport(REPORT_ID_MAC_ABS, rep5, 5);
    }
};

static USBHIDAbsolute s_usbAbs;
static bool s_usbDeviceStarted = false;
static usb_led_cb_t s_ledCallback = nullptr;
static uint8_t s_lastButtons = 0;

static volatile bool s_eventMounted = false;
static volatile bool s_eventUnmounted = false;
static volatile bool s_eventSuspended = false;
static volatile bool s_eventResumed = false;
static volatile uint8_t s_pendingLeds = 0;
static volatile bool s_hasPendingLeds = false;

static void onKeyboardLed(void* arg, esp_event_base_t base, int32_t id, void* data) {
    if (id == ARDUINO_USB_HID_KEYBOARD_LED_EVENT && data) {
        arduino_usb_hid_keyboard_event_data_t* ledData = (arduino_usb_hid_keyboard_event_data_t*)data;
        s_pendingLeds = ledData->leds;
        s_hasPendingLeds = true;
    }
}

void usb_device_set_led_callback(usb_led_cb_t cb) {
    s_ledCallback = cb;
}

void usb_device_init() {
    if (s_usbDeviceStarted) return;
    logPrint("[USB DEVICE] Initializing TinyUSB HID Mouse, Keyboard, Consumer & Absolute Pointer...");
    USB.productName("ESP32 KVM Combo");
    USB.manufacturerName("Espressif");
    USB.onEvent([](void* arg, esp_event_base_t base, int32_t id, void* data) {
        if (id == ARDUINO_USB_STARTED_EVENT) {
            s_eventMounted = true;
        } else if (id == ARDUINO_USB_STOPPED_EVENT) {
            s_eventUnmounted = true;
        } else if (id == ARDUINO_USB_SUSPEND_EVENT) {
            s_eventSuspended = true;
        } else if (id == ARDUINO_USB_RESUME_EVENT) {
            s_eventResumed = true;
        }
    });
    s_usbKeyboard.onEvent(onKeyboardLed);
    s_usbMouse.begin();
    s_usbKeyboard.begin();
    s_usbConsumer.begin();
    s_usbAbs.begin();
    USB.begin();
    s_usbDeviceStarted = true;
    logPrint("[USB DEVICE] TinyUSB HID started successfully!");
}

void usb_device_loop() {
    if (s_eventMounted) {
        s_eventMounted = false;
        logPrint("[USB DEVICE] Host PC MOUNTED device (Ready for 1000Hz HID)!");
    }
    if (s_eventUnmounted) {
        s_eventUnmounted = false;
        logPrint("[USB DEVICE] Host PC UNMOUNTED device!");
    }
    if (s_eventSuspended) {
        s_eventSuspended = false;
        logPrint("[USB DEVICE] USB Bus Suspended");
    }
    if (s_eventResumed) {
        s_eventResumed = false;
        logPrint("[USB DEVICE] USB Bus Resumed");
    }
    if (s_hasPendingLeds) {
        s_hasPendingLeds = false;
        uint8_t leds = s_pendingLeds;
        logPrint("[USB DEVICE] PC sent keyboard LED state: 0x%02X (Caps: %d, Num: %d, Scroll: %d)",
                 leds, (leds & 0x02) ? 1 : 0, (leds & 0x01) ? 1 : 0, (leds & 0x04) ? 1 : 0);
        if (s_ledCallback) {
            s_ledCallback(leds);
        }
    }
}

void usb_device_stop() {
    if (!s_usbDeviceStarted) return;
    logPrint("[USB DEVICE] Stopping TinyUSB HID...");
    s_usbAbs.end();
    s_usbConsumer.end();
    s_usbKeyboard.end();
    s_usbMouse.end();
    s_usbDeviceStarted = false;
}

bool usb_device_is_connected() {
    return s_usbDeviceStarted && (bool)USB;
}

void usb_device_send_mouse(uint8_t buttons, int16_t dx, int16_t dy, int8_t wheel, int8_t pan) {
    if (!s_usbDeviceStarted) return;
    do {
        int8_t curX = (int8_t)constrain(dx, -127, 127);
        int8_t curY = (int8_t)constrain(dy, -127, 127);
        
        // Update button press/release if changed
        if (buttons != s_lastButtons) {
            for (uint8_t b = 0; b < 5; b++) {
                uint8_t mask = (1 << b);
                if ((buttons & mask) && !(s_lastButtons & mask)) {
                    s_usbMouse.press(mask);
                } else if (!(buttons & mask) && (s_lastButtons & mask)) {
                    s_usbMouse.release(mask);
                }
            }
            s_lastButtons = buttons;
        }

        if (curX != 0 || curY != 0 || wheel != 0 || pan != 0) {
            s_usbMouse.move(curX, curY, wheel, pan);
        }

        dx -= curX;
        dy -= curY;
        wheel = 0;
        pan = 0;
    } while (dx != 0 || dy != 0);
}

void usb_device_send_keyboard(const uint8_t* rep8) {
    if (!s_usbDeviceStarted || !rep8) return;
    KeyReport rep;
    rep.modifiers = rep8[0];
    rep.reserved = rep8[1];
    memcpy(rep.keys, &rep8[2], 6);
    s_usbKeyboard.sendReport(&rep);
}

void usb_device_send_consumer(uint16_t usage) {
    if (!s_usbDeviceStarted) return;
    if (usage == 0) {
        s_usbConsumer.release();
    } else {
        s_usbConsumer.press(usage);
    }
}

void usb_device_send_mac_abs(const uint8_t* rep5) {
    if (!s_usbDeviceStarted || !rep5) return;
    s_usbAbs.sendMacDigitizer(rep5);
}

void usb_device_send_win_abs(const uint8_t* rep4) {
    if (!s_usbDeviceStarted || !rep4) return;
    s_usbAbs.sendWindowsAbs(rep4);
}

#endif
