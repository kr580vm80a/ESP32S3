#include "led_indicator.h"
#include "kvm_types.h"
#include "logi_bolt.h"

#ifndef RGB_BUILTIN
#define RGB_BUILTIN 48
#endif

// Comfortable brightness level for desktop use (halved to 15 / 255)
#define LED_BRIGHTNESS 15

static volatile uint32_t flashStartTime = 0;
static volatile uint32_t flashEndTime = 0;
static volatile int flashOs = -1;

static uint8_t curR = 255;
static uint8_t curG = 255;
static uint8_t curB = 255;

static inline void setRgb(uint8_t r, uint8_t g, uint8_t b) {
    if (r == curR && g == curG && b == curB) return;
    curR = r;
    curG = g;
    curB = b;
    neopixelWrite(RGB_BUILTIN, r, g, b);
}

void led_indicator_init() {
    setRgb(0, 0, 0); // Start with LED off
    logPrint("[LED] Initialized RGB LED indicator on GPIO %d (brightness: %d)", RGB_BUILTIN, LED_BRIGHTNESS);
}

static volatile uint32_t brightPulseEndTime = 0;
// Function to trigger a single flash.
void led_trigger_bright_pulse(int targetOs) {
    brightPulseEndTime = millis() + 50;
    flashOs = (targetOs >= 0) ? targetOs : getActiveClientOs();
}

void led_indicator_trigger_flash(int targetOs, uint32_t durationMs) {
    uint32_t now = millis();
    if (now >= flashEndTime) {
        flashStartTime = now;
    }
    flashEndTime = now + durationMs;
    flashOs = (targetOs >= 0) ? targetOs : getActiveClientOs();
}

void led_indicator_loop() {
    bool isMouseConn = mouseConnected || logi_bolt_is_mouse_connected();

    if (!isMouseConn) {
        // Blinking when mouse is not connected (500ms ON / 500ms OFF)
        bool blinkOn = ((millis() / 500) % 2) == 0;
        if (blinkOn) {
            setRgb(20, 5, 0);
        } else {
            setRgb(0, 0, 0);
        }
        return;
    }

    if (millis() < brightPulseEndTime) {
        if (flashOs == OS_MAC) {
            setRgb(0, 40, 0); // Flash Green
        } else {
            setRgb(0, 0, 70); // Flash Blue
        }
        return;
    }

    // Normal solid color based on active OS
    int os = getActiveClientOs();
    if (os == OS_MAC) {
        // Mode 3: Solid GREEN for MacBook
        setRgb(0, 7, 0);
    } else {
        // Mode 2: Solid BLUE for Windows (and default)
        setRgb(0, 0, 12);
    }
}
