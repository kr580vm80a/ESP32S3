#include "usb_device_engine.h"

#if CONFIG_IDF_TARGET_ESP32S3
#include "USB.h"
#include "tusb.h"
#include "USBHIDMouse.h"
#include "USBHIDKeyboard.h"
#include "USBHIDConsumerControl.h"
#include "hid_descriptors.h"
#include "kvm_types.h"

void logPrint(const char* format, ...);

static volatile int s_usbHostOs = -1;
static volatile bool s_seenMsftDescriptor = false;
static String s_usbBoundMac = "";
static volatile bool s_probeActive = false;
static volatile uint32_t s_probeStartTime = 0;
static volatile int s_probeTargetOs = -1;
static volatile bool s_probeSentRelease = false;

// Override TinyUSB weak callback to catch Microsoft OS String Descriptor (0xEE)
extern "C" uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    if (index == 0xEE) {
        s_seenMsftDescriptor = true;
        s_usbHostOs = OS_WINDOWS;
        return NULL;
    }
    static uint16_t _desc_str[127];
    uint8_t chr_count = 0;
    if (index == 0) {
        _desc_str[1] = 0x0409;
        chr_count = 1;
    } else {
        const char *str = NULL;
        if (index == 1) str = USB.manufacturerName();
        else if (index == 2) str = USB.productName();
        else if (index == 3) str = USB.serialNumber();
        if (!str) return NULL;
        chr_count = strlen(str);
        if (chr_count > 126) chr_count = 126;
        for (uint8_t i = 0; i < chr_count; i++) {
            _desc_str[1 + i] = str[i];
        }
    }
    _desc_str[0] = (0x03 << 8) | (2 * chr_count + 2);
    return _desc_str;
}

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

int usb_device_get_detected_os() {
    return s_usbHostOs;
}

String usb_device_get_bound_mac() {
    return s_usbBoundMac;
}

void usb_device_set_bound_mac(const String& mac) {
    s_usbBoundMac = mac;
    updateKvmPowerAndRateProfiles("", true);
}

void usb_device_clear_bound_mac() {
    s_usbBoundMac = "";
    updateKvmPowerAndRateProfiles("", true);
}

void usb_device_check_detection() {
    if (!usb_device_is_connected() || s_usbHostOs == -1) return;
    if (s_usbBoundMac.length() > 0 && !s_probeActive) return;

    // Collect all unique candidate MACs in active layout that match s_usbHostOs
    int matchingCount = 0;
    String matchingMacs[MAX_SUPPORTED_KVM_CLIENTS];
    for (int i = 0; i < monitorCount; i++) {
        if (monitors[i].os == s_usbHostOs && monitors[i].mac.length() > 0 &&
            !monitors[i].mac.equalsIgnoreCase("USB") && !monitors[i].mac.equalsIgnoreCase("USB-C")) {
            bool found = false;
            for (int m = 0; m < matchingCount; m++) {
                if (matchingMacs[m].equalsIgnoreCase(monitors[i].mac)) { found = true; break; }
            }
            if (!found && matchingCount < MAX_SUPPORTED_KVM_CLIENTS) {
                matchingMacs[matchingCount++] = monitors[i].mac;
            }
        }
    }

    if (matchingCount == 0) {
        logPrint("[USB DETECT] No layout monitor matched detected OS (%s)",
                 s_usbHostOs == OS_WINDOWS ? "Windows" : "macOS");
        return;
    }

    if (matchingCount == 1) {
        usb_device_set_bound_mac(matchingMacs[0]);
        logPrint("[USB DETECT] >>> Instant Match: Only 1 %s in layout -> USB bound to %s (1000Hz) <<<",
                 (s_usbHostOs == OS_WINDOWS ? "Windows PC" : "MacBook"), s_usbBoundMac.c_str());
        return;
    }

    // Multiple candidates (>= 2): check how many are currently connected via BLE
    int connectedBleCount = 0;
    String connectedBleMacs[MAX_SUPPORTED_KVM_CLIENTS];
    String disconnectedMacs[MAX_SUPPORTED_KVM_CLIENTS];
    int disconnectedCount = 0;

    for (int m = 0; m < matchingCount; m++) {
        bool isConn = false;
        for (int k = 0; k < MAX_SUPPORTED_KVM_CLIENTS; k++) {
            if (kvmClients[k].active && kvmClients[k].conn_id != BLE_HS_CONN_HANDLE_NONE &&
                kvmClients[k].mac.equalsIgnoreCase(matchingMacs[m])) {
                isConn = true;
                break;
            }
        }
        if (isConn) {
            connectedBleMacs[connectedBleCount++] = matchingMacs[m];
        } else {
            disconnectedMacs[disconnectedCount++] = matchingMacs[m];
        }
    }

    if (connectedBleCount == 0) {
        logPrint("[USB DETECT] Multiple %s candidates (%d in layout), but 0 connected via BLE. Waiting for BLE connect...",
                 (s_usbHostOs == OS_WINDOWS ? "Windows" : "macOS"), matchingCount);
        return;
    }

    // Trigger Echo Probe
    logPrint("[USB DETECT] Starting Echo Probe for %s (%d BLE connected candidates)...",
             (s_usbHostOs == OS_WINDOWS ? "Windows (ScrollLock)" : "macOS (CapsLock)"), connectedBleCount);
    s_probeActive = true;
    s_probeStartTime = millis();
    s_probeTargetOs = s_usbHostOs;
    s_probeSentRelease = false;

    if (s_usbHostOs == OS_WINDOWS) {
        uint8_t p[8] = {0, 0, 0x47, 0, 0, 0, 0, 0}; // ScrollLock press
        usb_device_send_keyboard(p);
        uint8_t rel[8] = {0};
        usb_device_send_keyboard(rel);
        s_probeSentRelease = true;
    } else if (s_usbHostOs == OS_MAC) {
        uint8_t p[8] = {0, 0, 0x39, 0, 0, 0, 0, 0}; // CapsLock press
        usb_device_send_keyboard(p);
        // Released after 80ms in usb_device_loop()
    }
}

void usb_device_on_ble_led_report(const String& mac, uint8_t leds) {
    if (!s_probeActive) return;
    bool match = false;
    if (s_probeTargetOs == OS_WINDOWS && (leds & 0x04)) {
        match = true;
    } else if (s_probeTargetOs == OS_MAC && (leds & 0x02)) {
        match = true;
    }
    if (match) {
        s_probeActive = false;
        usb_device_set_bound_mac(mac);
        logPrint("[USB DETECT] >>> Echo Probe Match! USB verified physically connected to %s (1000Hz) <<<", mac.c_str());
        // Revert key on USB host to restore original state
        if (s_probeTargetOs == OS_WINDOWS) {
            uint8_t p[8] = {0, 0, 0x47, 0, 0, 0, 0, 0};
            usb_device_send_keyboard(p);
            uint8_t rel[8] = {0};
            usb_device_send_keyboard(rel);
        } else if (s_probeTargetOs == OS_MAC) {
            uint8_t p[8] = {0, 0, 0x39, 0, 0, 0, 0, 0};
            usb_device_send_keyboard(p);
            vTaskDelay(pdMS_TO_TICKS(50));
            uint8_t rel[8] = {0};
            usb_device_send_keyboard(rel);
        }
    }
}

void usb_device_on_ble_connect(const String& mac, int os) {
    if (!usb_device_is_connected()) return;
    if (s_usbHostOs == -1) return;
    if (os != s_usbHostOs) return;
    // If not bound or if currently bound, verify with probe
    if (s_usbBoundMac.length() == 0 || s_usbBoundMac.equalsIgnoreCase(mac)) {
        logPrint("[USB DETECT] Candidate %s connected via BLE (%s). Checking USB binding...",
                 mac.c_str(), os == OS_MAC ? "Mac" : "Win");
        usb_device_check_detection();
    }
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
    usb_device_flush_mouse();

    if (s_eventMounted) {
        s_eventMounted = false;
        if (s_seenMsftDescriptor) {
            s_usbHostOs = OS_WINDOWS;
        } else if (s_usbHostOs != OS_WINDOWS) {
            s_usbHostOs = OS_MAC;
        }
        logPrint("[USB DEVICE] Host PC MOUNTED device (Ready for 1000Hz HID)! Detected OS: %s",
                 (s_usbHostOs == OS_WINDOWS) ? "Windows (MSFT100)" : "macOS / Apple HID");
        usb_device_check_detection();
    }
    if (s_eventUnmounted) {
        s_eventUnmounted = false;
        s_seenMsftDescriptor = false;
        s_usbHostOs = -1;
        usb_device_clear_bound_mac();
        s_probeActive = false;
        logPrint("[USB DEVICE] Host PC UNMOUNTED device! Cleared USB binding.");
    }
    if (s_eventSuspended) {
        s_eventSuspended = false;
        logPrint("[USB DEVICE] USB Bus Suspended (Cable unplugged or PC Sleep) -> KVM fallback to BLE");
        updateKvmPowerAndRateProfiles("", true);
    }
    if (s_eventResumed) {
        s_eventResumed = false;
        logPrint("[USB DEVICE] USB Bus Resumed -> Re-activating 1000Hz HID!");
        if (s_usbHostOs != -1 && s_usbBoundMac.length() == 0) {
            usb_device_check_detection();
        }
        updateKvmPowerAndRateProfiles("", true);
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

    // Active probe timeout & elimination handling
    if (s_probeActive) {
        uint32_t elapsed = millis() - s_probeStartTime;
        // Release CapsLock after 80ms for Mac
        if (s_probeTargetOs == OS_MAC && !s_probeSentRelease && elapsed >= 80) {
            uint8_t rel[8] = {0};
            usb_device_send_keyboard(rel);
            s_probeSentRelease = true;
        }
        // Timeout after 250ms
        if (elapsed >= 250) {
            s_probeActive = false;
            logPrint("[USB DETECT] Probe timeout: No connected BLE client echoed.");
            if (s_probeTargetOs == OS_MAC) {
                uint8_t p[8] = {0, 0, 0x39, 0, 0, 0, 0, 0};
                usb_device_send_keyboard(p);
                vTaskDelay(pdMS_TO_TICKS(50));
                uint8_t rel[8] = {0};
                usb_device_send_keyboard(rel);
            } else if (s_probeTargetOs == OS_WINDOWS) {
                uint8_t p[8] = {0, 0, 0x47, 0, 0, 0, 0, 0};
                usb_device_send_keyboard(p);
                uint8_t rel[8] = {0};
                usb_device_send_keyboard(rel);
            }

            // Apply Elimination Principle:
            int matchingCount = 0;
            String matchingMacs[MAX_SUPPORTED_KVM_CLIENTS];
            for (int i = 0; i < monitorCount; i++) {
                if (monitors[i].os == s_probeTargetOs && monitors[i].mac.length() > 0 &&
                    !monitors[i].mac.equalsIgnoreCase("USB") && !monitors[i].mac.equalsIgnoreCase("USB-C")) {
                    bool f = false;
                    for (int m = 0; m < matchingCount; m++) {
                        if (matchingMacs[m].equalsIgnoreCase(monitors[i].mac)) { f = true; break; }
                    }
                    if (!f && matchingCount < MAX_SUPPORTED_KVM_CLIENTS) {
                        matchingMacs[matchingCount++] = monitors[i].mac;
                    }
                }
            }
            int disconnectedCount = 0;
            String disconnectedMacs[MAX_SUPPORTED_KVM_CLIENTS];
            for (int m = 0; m < matchingCount; m++) {
                bool isConn = false;
                for (int k = 0; k < MAX_SUPPORTED_KVM_CLIENTS; k++) {
                    if (kvmClients[k].active && kvmClients[k].conn_id != BLE_HS_CONN_HANDLE_NONE &&
                        kvmClients[k].mac.equalsIgnoreCase(matchingMacs[m])) {
                        isConn = true;
                        break;
                    }
                }
                if (!isConn) {
                    disconnectedMacs[disconnectedCount++] = matchingMacs[m];
                }
            }
            if (disconnectedCount == 1) {
                usb_device_set_bound_mac(disconnectedMacs[0]);
                logPrint("[USB DETECT] >>> Elimination Match: Connected BLE client(s) did not echo -> USB bound to %s (1000Hz) <<<",
                         s_usbBoundMac.c_str());
            }
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
    return s_usbDeviceStarted && tud_ready();
}

static int16_t s_pendingDx = 0;
static int16_t s_pendingDy = 0;
static int8_t  s_pendingWheel = 0;
static int8_t  s_pendingPan = 0;
static uint8_t s_transmittedButtons = 0;

void usb_device_flush_mouse() {
    if (!s_usbDeviceStarted || !tud_ready() || !tud_hid_n_ready(0)) return;
    bool buttonsChanged = (s_lastButtons != s_transmittedButtons);
    if (!buttonsChanged && s_pendingDx == 0 && s_pendingDy == 0 && s_pendingWheel == 0 && s_pendingPan == 0) return;

    int8_t curX = (int8_t)constrain(s_pendingDx, -127, 127);
    int8_t curY = (int8_t)constrain(s_pendingDy, -127, 127);
    int8_t curWheel = (int8_t)constrain(s_pendingWheel, -127, 127);
    int8_t curPan = (int8_t)constrain(s_pendingPan, -127, 127);

    uint8_t mouseReport[5] = {
        s_lastButtons,
        (uint8_t)curX,
        (uint8_t)curY,
        (uint8_t)curWheel,
        (uint8_t)curPan
    };

    if (tud_hid_n_report(0, REPORT_ID_MOUSE, mouseReport, sizeof(mouseReport))) {
        s_pendingDx -= curX;
        s_pendingDy -= curY;
        s_pendingWheel -= curWheel;
        s_pendingPan -= curPan;
        s_transmittedButtons = s_lastButtons;
    }
}

void usb_device_send_mouse(uint8_t buttons, int16_t dx, int16_t dy, int8_t wheel, int8_t pan) {
    if (!s_usbDeviceStarted || !tud_ready()) return;

    s_pendingDx += dx;
    s_pendingDy += dy;
    s_pendingWheel += wheel;
    s_pendingPan += pan;
    s_lastButtons = buttons;

    usb_device_flush_mouse();
}

void usb_device_send_keyboard(const uint8_t* rep8) {
    if (!s_usbDeviceStarted || !tud_ready() || !rep8) return;
    tud_hid_n_report(0, REPORT_ID_KEYBOARD, rep8, 8);
}

void usb_device_send_consumer(uint16_t usage) {
    if (!s_usbDeviceStarted || !tud_ready()) return;
    tud_hid_n_report(0, REPORT_ID_MEDIA, &usage, sizeof(usage));
}

void usb_device_send_mac_abs(const uint8_t* rep5) {
    if (!s_usbDeviceStarted || !tud_ready() || !rep5) return;
    tud_hid_n_report(0, REPORT_ID_MAC_ABS, rep5, 5);
}

void usb_device_send_win_abs(const uint8_t* rep4) {
    if (!s_usbDeviceStarted || !tud_ready() || !rep4) return;
    tud_hid_n_report(0, REPORT_ID_WIN_ABS, rep4, 4);
}

#endif
