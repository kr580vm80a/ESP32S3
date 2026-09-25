#include "usb_manager.h"
#include "logi_bolt.h"
#include "usb_device_engine.h"
#include "driver/periph_ctrl.h"
#include "soc/periph_defs.h"
#include "soc/usb_wrap_reg.h"
#include "soc/usb_serial_jtag_reg.h"
#include "driver/gpio.h"

void logPrint(const char* format, ...);

static UsbKvmMode s_currentMode = USB_KVM_MODE_NONE;
static volatile bool s_boltDevGone = false;

void usb_manager_switch_to(UsbKvmMode newMode) {
    if (s_currentMode == newMode) return;

    // A clean reboot is only needed when swapping incompatible silicon roles (Host <-> Device)
    if (s_currentMode != USB_KVM_MODE_NONE && s_currentMode != newMode) {
        logPrint("[USB MGR] Hardware USB role swap requested (%s -> %s). Rebooting ESP32 cleanly...",
                 s_currentMode == USB_KVM_MODE_HOST_BOLT ? "Host (Bolt)" : "Device (PC)",
                 newMode == USB_KVM_MODE_HOST_BOLT ? "Host (Bolt)" : "Device (PC)");
        delay(100);
        esp_restart();
    }

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

UsbKvmMode usb_manager_get_mode() {
    return s_currentMode;
}

// Fast non-destructive hardware line probe that determines port electrical state
static UsbKvmMode probe_usb_lines_fast() {
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

    // Step A: Test with pulldowns for Full-Speed peripheral (Logitech Bolt: 1.5k pullup on D+)
    pinMode(20, INPUT_PULLDOWN);
    pinMode(19, INPUT_PULLDOWN);
    delay(5);
    int dp_pd = digitalRead(20);
    int dm_pd = digitalRead(19);

    if (dp_pd == HIGH && dm_pd == LOW) {
        pinMode(20, INPUT);
        pinMode(19, INPUT);
        SET_PERI_REG_MASK(USB_WRAP_OTG_CONF_REG, USB_WRAP_USB_PAD_ENABLE);
        CLEAR_PERI_REG_MASK(USB_WRAP_OTG_CONF_REG, USB_WRAP_PAD_PULL_OVERRIDE);
        return USB_KVM_MODE_HOST_BOLT;
    }

    // Step B: Test with pullups for PC / MacBook Host (15k pulldowns to GND on both lines)
    // vs Open Circuit (Empty port: lines float to 3.3V)
    pinMode(20, INPUT_PULLUP);
    pinMode(19, INPUT_PULLUP);
    delay(5);
    int dp_pu = digitalRead(20);
    int dm_pu = digitalRead(19);

    pinMode(20, INPUT);
    pinMode(19, INPUT);
    SET_PERI_REG_MASK(USB_WRAP_OTG_CONF_REG, USB_WRAP_USB_PAD_ENABLE);
    CLEAR_PERI_REG_MASK(USB_WRAP_OTG_CONF_REG, USB_WRAP_PAD_PULL_OVERRIDE);

    // PC Host has 15k pulldowns to GND on both D+ and D- -> lines read LOW
    if (dp_pu == LOW && dm_pu == LOW) {
        return USB_KVM_MODE_DEVICE_PC;
    }

    // Open circuit (empty port): internal 45k pullups pull both lines HIGH
    return USB_KVM_MODE_NONE;
}

void usb_manager_init() {
    logPrint("[USB MGR] Auto-probing USB port hardware state...");
    UsbKvmMode detected = probe_usb_lines_fast();
    if (detected == USB_KVM_MODE_DEVICE_PC) {
        logPrint("[USB MGR] -> Detected Wired PC / MacBook Host (1000Hz mode)");
        usb_manager_switch_to(USB_KVM_MODE_DEVICE_PC);
    } else if (detected == USB_KVM_MODE_HOST_BOLT) {
        logPrint("[USB MGR] -> Detected Full-Speed USB Device (Logitech Bolt Receiver)");
        usb_manager_switch_to(USB_KVM_MODE_HOST_BOLT);
    } else {
        logPrint("[USB MGR] -> Port empty (Pure BLE mode, USB standby)");
        s_currentMode = USB_KVM_MODE_NONE;
    }
}

void usb_manager_loop() {
    uint32_t now = millis();

    if (s_currentMode == USB_KVM_MODE_NONE) {
        static uint32_t s_lastNoneProbeMs = 0;
        if (now - s_lastNoneProbeMs >= 500) {
            s_lastNoneProbeMs = now;
            UsbKvmMode detected = probe_usb_lines_fast();
            if (detected == USB_KVM_MODE_DEVICE_PC) {
                logPrint("[USB MGR] ========================================================");
                logPrint("[USB MGR] PC / MacBook detected on USB-C port (D+=0, D-=0 under pullup)!");
                logPrint("[USB MGR] >>> Initializing USB Device mode (1000Hz HID) without reboot <<<");
                logPrint("[USB MGR] ========================================================");
                usb_manager_switch_to(USB_KVM_MODE_DEVICE_PC);
            } else if (detected == USB_KVM_MODE_HOST_BOLT) {
                logPrint("[USB MGR] ========================================================");
                logPrint("[USB MGR] Logitech Bolt receiver detected on USB port (D+=1, D-=0 under pulldown)!");
                logPrint("[USB MGR] >>> Initializing USB Host mode (Logi Bolt) without reboot <<<");
                logPrint("[USB MGR] ========================================================");
                usb_manager_switch_to(USB_KVM_MODE_HOST_BOLT);
            }
        }
    } else if (s_currentMode == USB_KVM_MODE_HOST_BOLT) {
        logi_bolt_loop();
        static uint32_t s_lastHostProbeMs = 0;
        if (s_lastHostProbeMs == 0) s_lastHostProbeMs = now;

        static uint32_t s_lastBoltGoneMs = 0;
        if (s_boltDevGone) {
            s_boltDevGone = false;
            s_lastBoltGoneMs = now;
            s_lastHostProbeMs = now;
            logPrint("[USB MGR] Bolt disconnected. USB Host standby active (mouse/keyboard on BLE).");
        }

        // When Bolt is NOT attached, check if a PC / MacBook USB-C cable was plugged in!
        // Wait at least 3000ms after boot / disconnect so host stack settles before any probe
        if (!logi_bolt_is_device_attached() && now >= 3000 && (s_lastBoltGoneMs == 0 || now - s_lastBoltGoneMs >= 3000)) {
            if (now - s_lastHostProbeMs >= 1500) {
                s_lastHostProbeMs = now;
                logi_bolt_pause_host_tasks(true);
                UsbKvmMode detected = probe_usb_lines_fast();
                logi_bolt_pause_host_tasks(false);

                if (detected == USB_KVM_MODE_DEVICE_PC) {
                    logPrint("[USB MGR] ========================================================");
                    logPrint("[USB MGR] PC / MacBook detected on USB-C port (D+=0, D-=0 under pullup)!");
                    logPrint("[USB MGR] >>> Controlled reboot in progress to swap USB role (Host -> Device 1000Hz) <<<");
                    logPrint("[USB MGR] >>> (This is an intentional restart, NOT a crash!) <<<");
                    logPrint("[USB MGR] ========================================================");
                    logi_bolt_deinit();
                    delay(100);
                    esp_restart();
                }
            }
        }
    } else if (s_currentMode == USB_KVM_MODE_DEVICE_PC) {
        usb_device_loop();

        static uint32_t s_lastDeviceProbeMs = 0;
        if (s_lastDeviceProbeMs == 0) s_lastDeviceProbeMs = now;

        if (usb_device_is_connected()) {
            s_lastDeviceProbeMs = now; // Continuously refresh timer while PC is connected!
        } else {
            // When PC is NOT connected, give 5000ms grace after boot / disconnect before probing
            // This guarantees macOS finishes USB enumeration undisturbed!
            if (now >= 5000 && (now - s_lastDeviceProbeMs >= 3000)) {
                s_lastDeviceProbeMs = now;
                UsbKvmMode detected = probe_usb_lines_fast();
                if (detected == USB_KVM_MODE_HOST_BOLT) {
                    logPrint("[USB MGR] ========================================================");
                    logPrint("[USB MGR] Logitech Bolt receiver detected on USB port (D+=1, D-=0 under pulldown)!");
                    logPrint("[USB MGR] >>> Controlled reboot in progress to swap USB role (Device -> Host) <<<");
                    logPrint("[USB MGR] >>> (This is an intentional restart, NOT a crash!) <<<");
                    logPrint("[USB MGR] ========================================================");
                    delay(100);
                    esp_restart();
                }
            }
        }
    }
}

bool usb_manager_is_pc_connected() {
    return (s_currentMode == USB_KVM_MODE_DEVICE_PC) && usb_device_is_connected();
}

void usb_manager_notify_host_dev_gone() {
    s_boltDevGone = true;
}

