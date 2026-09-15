#include "usb_manager.h"
#include "logi_bolt.h"
#include "usb_device_engine.h"

#if CONFIG_IDF_TARGET_ESP32S3
#include "driver/periph_ctrl.h"
#include "soc/periph_defs.h"
#include "soc/usb_wrap_reg.h"
#include "soc/usb_serial_jtag_reg.h"
#include "soc/soc.h"
#include "driver/gpio.h"
#include <Preferences.h>

void logPrint(const char* format, ...);

static UsbKvmMode s_currentMode = USB_KVM_MODE_NONE;
static volatile bool s_boltDevGone = false;

static void usb_manager_switch_to(UsbKvmMode newMode) {
    if (s_currentMode == newMode) return;
    s_currentMode = newMode;

    if (newMode == USB_KVM_MODE_HOST_BOLT) {
        logPrint("[USB MGR] >>> Mode: USB Host (Logitech Bolt / Peripheral) <<<");
        logi_bolt_init();
    } else {
        logPrint("[USB MGR] >>> Mode: USB Device (Wired PC / MacBook HID 1000Hz) <<<");
        extern void handleUsbDeviceKeyboardLed(uint8_t leds);
        usb_device_set_led_callback(handleUsbDeviceKeyboardLed);
        usb_device_init();
    }
}

// Hardware line probe that overrides ESP32-S3 internal pull-ups
static UsbKvmMode probe_usb_hardware() {
    // 1. Force-disable internal pullups/pulldowns from USB Serial/JTAG controller
    SET_PERI_REG_MASK(USB_SERIAL_JTAG_CONF0_REG, USB_SERIAL_JTAG_PAD_PULL_OVERRIDE);
    CLEAR_PERI_REG_MASK(USB_SERIAL_JTAG_CONF0_REG, USB_SERIAL_JTAG_USB_PAD_ENABLE);
    CLEAR_PERI_REG_MASK(USB_SERIAL_JTAG_CONF0_REG, USB_SERIAL_JTAG_DP_PULLUP | 
                                                   USB_SERIAL_JTAG_DM_PULLUP | 
                                                   USB_SERIAL_JTAG_DP_PULLDOWN | 
                                                   USB_SERIAL_JTAG_DM_PULLDOWN);

    // 2. Force-disable internal pullups/pulldowns from USB_WRAP PHY
    CLEAR_PERI_REG_MASK(USB_WRAP_OTG_CONF_REG, USB_WRAP_USB_PAD_ENABLE);
    SET_PERI_REG_MASK(USB_WRAP_OTG_CONF_REG, USB_WRAP_PAD_PULL_OVERRIDE);
    CLEAR_PERI_REG_MASK(USB_WRAP_OTG_CONF_REG, USB_WRAP_DP_PULLUP | 
                                               USB_WRAP_DM_PULLUP | 
                                               USB_WRAP_DP_PULLDOWN | 
                                               USB_WRAP_DM_PULLDOWN);

    // 3. Reset GPIO 19 and 20 into clean GPIO mode
    gpio_reset_pin(GPIO_NUM_20); // D+
    gpio_reset_pin(GPIO_NUM_19); // D-

    pinMode(20, INPUT_PULLDOWN);
    pinMode(19, INPUT_PULLDOWN);
    delay(10); // Allow line capacitance to settle through 45k pulldowns

    int dp_pd = digitalRead(20);
    int dm_pd = digitalRead(19);

    pinMode(20, INPUT);
    pinMode(19, INPUT);

    logPrint("[USB MGR] Hardware probe: D+=%d, D-=%d", dp_pd, dm_pd);

    if (dp_pd == HIGH && dm_pd == LOW) {
        logPrint("[USB MGR] -> Detected Full-Speed USB Device (Logitech Bolt Receiver)");
        return USB_KVM_MODE_HOST_BOLT;
    }

    logPrint("[USB MGR] -> No peripheral detected (Wired PC / MacBook HID 1000Hz mode)");
    return USB_KVM_MODE_DEVICE_PC;
}

void usb_manager_set_preferred_mode(const String& mode) {
    Preferences prefs;
    prefs.begin("kvm_usb", false);
    prefs.putString("mode", mode);
    prefs.end();
    logPrint("[USB MGR] Preferred USB mode saved: %s. Rebooting to apply...", mode.c_str());
    delay(100);
    esp_restart();
}

String usb_manager_get_preferred_mode() {
    Preferences prefs;
    prefs.begin("kvm_usb", true);
    String m = prefs.getString("mode", "auto");
    prefs.end();
    return m;
}

void usb_manager_init() {
    String pref = usb_manager_get_preferred_mode();
    if (pref.equalsIgnoreCase("bolt")) {
        logPrint("[USB MGR] NVS Override: Logitech Bolt / Host");
        usb_manager_switch_to(USB_KVM_MODE_HOST_BOLT);
    } else if (pref.equalsIgnoreCase("pc")) {
        logPrint("[USB MGR] NVS Override: Wired PC / MacBook HID");
        usb_manager_switch_to(USB_KVM_MODE_DEVICE_PC);
    } else {
        logPrint("[USB MGR] Auto-probing USB port hardware state...");
        UsbKvmMode detected = probe_usb_hardware();
        usb_manager_switch_to(detected);
    }
}

void usb_manager_loop() {
    if (s_currentMode == USB_KVM_MODE_HOST_BOLT) {
        logi_bolt_loop();
        if (s_boltDevGone) {
            s_boltDevGone = false;
            logPrint("[USB MGR] Bolt disconnected. Switching cleanly to USB Device (PC/Mac) & BLE...");
            logi_bolt_deinit();
            delay(50);
            esp_restart();
        }
    } else if (s_currentMode == USB_KVM_MODE_DEVICE_PC) {
        usb_device_loop();
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
