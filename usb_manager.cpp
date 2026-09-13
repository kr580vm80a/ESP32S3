#include "usb_manager.h"
#include "logi_bolt.h"
#include "usb_device_engine.h"

#if CONFIG_IDF_TARGET_ESP32S3
#include "driver/periph_ctrl.h"
#include "soc/periph_defs.h"
#include "soc/usb_wrap_reg.h"
#include "soc/soc.h"
#include "driver/gpio.h"

void logPrint(const char* format, ...);

static UsbKvmMode s_currentMode = USB_KVM_MODE_NONE;
static volatile bool s_boltDevGone = false;
static uint32_t s_lastCheckMs = 0;
static uint32_t s_deviceDisconnectTimer = 0;

static bool s_pcMountedOnce = false;

static UsbKvmMode probe_usb_lines() {
    REG_CLR_BIT(USB_WRAP_OTG_CONF_REG, USB_WRAP_USB_PAD_ENABLE);
    gpio_reset_pin(GPIO_NUM_20); // D+
    gpio_reset_pin(GPIO_NUM_19); // D-

    // 1. Test for Full-Speed USB Device (Logi Bolt, Mouse, Keyboard):
    // Full-Speed devices have hardware 1.5k pullup to 3.3V on D+.
    // Under internal pulldown (45k), D+ stays HIGH (~3.19V).
    pinMode(20, INPUT_PULLDOWN);
    pinMode(19, INPUT_PULLDOWN);
    delay(5);
    int dp_pd = digitalRead(20);

    if (dp_pd == HIGH) {
        pinMode(20, INPUT);
        pinMode(19, INPUT);
        return USB_KVM_MODE_HOST_BOLT;
    }

    // 2. Test for USB Host (PC / MacBook) vs Open Circuit (Nothing connected):
    // USB Hosts have 15k pulldowns to GND on BOTH D+ and D-.
    // Open Circuit (nothing plugged in) has NO connection to GND,
    // so internal pullup holds BOTH lines at 3.3V (HIGH).
    pinMode(20, INPUT_PULLUP);
    pinMode(19, INPUT_PULLUP);
    delay(5);
    int dp_pu = digitalRead(20);
    int dm_pu = digitalRead(19);

    pinMode(20, INPUT);
    pinMode(19, INPUT);

    // If PC Host (15k to GND on both lines) is connected:
    // Voltage divider 45k pullup vs 15k pulldown produces ~0.825V (LOW) on both D+ and D-.
    if (dp_pu == LOW && dm_pu == LOW) {
        return USB_KVM_MODE_DEVICE_PC;
    }

    // Open circuit: both lines are pulled HIGH by internal pullups
    return USB_KVM_MODE_NONE;
}

static bool s_tinyUsbEverInitialized = false;
static bool s_usbHostEverInitialized = false;

static void usb_manager_switch_to(UsbKvmMode newMode) {
    if (s_currentMode == newMode) return;

    // Check if hardware role swap requires clean ESP32 reboot:
    // ESP-IDF hardware USB PHY interrupts cannot be dynamically swapped between Host and Device
    if ((newMode == USB_KVM_MODE_HOST_BOLT && s_tinyUsbEverInitialized) ||
        (newMode == USB_KVM_MODE_DEVICE_PC && s_usbHostEverInitialized)) {
        logPrint("[USB MGR] Hardware USB role swap requested (Host <-> Device). Rebooting ESP32-S3 cleanly...");
        delay(100);
        esp_restart();
    }

    // Teardown current mode
    if (s_currentMode == USB_KVM_MODE_HOST_BOLT) {
        logPrint("[USB MGR] Stopping USB Host stack (Logi Bolt)...");
        logi_bolt_deinit();
    } else if (s_currentMode == USB_KVM_MODE_DEVICE_PC) {
        logPrint("[USB MGR] Stopping USB Device stack (PC HID)...");
        usb_device_stop();
        REG_CLR_BIT(USB_WRAP_OTG_CONF_REG, USB_WRAP_USB_PAD_ENABLE);
        gpio_reset_pin(GPIO_NUM_20);
        gpio_reset_pin(GPIO_NUM_19);
    }

    s_currentMode = USB_KVM_MODE_NONE;
    s_boltDevGone = false;
    s_deviceDisconnectTimer = 0;
    s_pcMountedOnce = false;

    // Start target mode
    if (newMode == USB_KVM_MODE_HOST_BOLT) {
        logPrint("[USB MGR] >>> Mode: USB Host (Logitech Bolt / Peripheral) <<<");
        s_usbHostEverInitialized = true;
        s_currentMode = USB_KVM_MODE_HOST_BOLT;
        logi_bolt_init();
    } else if (newMode == USB_KVM_MODE_DEVICE_PC) {
        logPrint("[USB MGR] >>> Mode: USB Device (Wired PC / MacBook HID 1000Hz) <<<");
        s_tinyUsbEverInitialized = true;
        s_currentMode = USB_KVM_MODE_DEVICE_PC;
        extern void handleUsbDeviceKeyboardLed(uint8_t leds);
        usb_device_set_led_callback(handleUsbDeviceKeyboardLed);
        usb_device_init();
    } else {
        logPrint("[USB MGR] >>> Mode: Pure BLE (Port empty / idle) <<<");
        s_currentMode = USB_KVM_MODE_NONE;
    }
}

void usb_manager_init() {
    logPrint("[USB MGR] Probing USB port electrical state...");
    UsbKvmMode initial = probe_usb_lines();
    usb_manager_switch_to(initial);
}

void usb_manager_loop() {
    if (s_currentMode == USB_KVM_MODE_HOST_BOLT) {
        logi_bolt_loop();
    } else if (s_currentMode == USB_KVM_MODE_DEVICE_PC) {
        usb_device_loop();
    }

    uint32_t now = millis();
    if (now - s_lastCheckMs < 500) return;
    s_lastCheckMs = now;

    if (s_currentMode == USB_KVM_MODE_NONE) {
        UsbKvmMode detected = probe_usb_lines();
        if (detected != USB_KVM_MODE_NONE) {
            logPrint("[USB MGR] Hot-plug detected connection: %s",
                     detected == USB_KVM_MODE_HOST_BOLT ? "Logi Bolt / Peripheral" : "PC / MacBook");
            usb_manager_switch_to(detected);
        }
    } else if (s_currentMode == USB_KVM_MODE_HOST_BOLT) {
        if (s_boltDevGone) {
            s_boltDevGone = false;
            logPrint("[USB MGR] Bolt disconnect confirmed -> switching to NONE");
            usb_manager_switch_to(USB_KVM_MODE_NONE);
        }
    } else if (s_currentMode == USB_KVM_MODE_DEVICE_PC) {
        bool connected = usb_device_is_connected();
        if (connected) {
            s_pcMountedOnce = true;
            s_deviceDisconnectTimer = 0;
        } else {
            if (s_deviceDisconnectTimer == 0) {
                s_deviceDisconnectTimer = now;
            } else {
                // If previously mounted, wait 10s before disconnect (gives time for macOS screen lock/wake).
                // If not yet mounted, wait up to 120s for user to click "Allow accessory to connect" on macOS!
                uint32_t timeout = s_pcMountedOnce ? 10000 : 120000;
                if (now - s_deviceDisconnectTimer > timeout) {
                    s_deviceDisconnectTimer = 0;
                    s_pcMountedOnce = false;
                    logPrint("[USB MGR] PC disconnected from USB-C -> switching to NONE");
                    usb_manager_switch_to(USB_KVM_MODE_NONE);
                }
            }
        }
    }
}

UsbKvmMode usb_manager_get_mode() {
    return s_currentMode;
}

bool usb_manager_is_pc_connected() {
    return (s_currentMode == USB_KVM_MODE_DEVICE_PC) && usb_device_is_connected();
}

bool usb_manager_is_bolt_connected() {
    return (s_currentMode == USB_KVM_MODE_HOST_BOLT) && (logi_bolt_is_mouse_connected() || logi_bolt_is_keyboard_connected());
}

void usb_manager_notify_host_dev_gone() {
    s_boltDevGone = true;
}

#else

void usb_manager_init() {}
void usb_manager_loop() {}
UsbKvmMode usb_manager_get_mode() { return USB_KVM_MODE_NONE; }
bool usb_manager_is_pc_connected() { return false; }
bool usb_manager_is_bolt_connected() { return false; }
void usb_manager_notify_host_dev_gone() {}

#endif
