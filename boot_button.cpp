#include "boot_button.h"
#include "ble_server.h"
#include "kvm_types.h"

#define BOOT_BUTTON_PIN 0

static uint32_t btnPressStart = 0;
static bool btnIsPressed = false;
static bool btnLongPressTriggered = false;

void boot_button_init() {
    pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
    logPrint("[BOOT BTN] Initialized BOOT button on GPIO %d", BOOT_BUTTON_PIN);
}

void boot_button_loop() {
    bool pressed = (digitalRead(BOOT_BUTTON_PIN) == LOW);
    uint32_t now = millis();

    if (pressed && !btnIsPressed) {
        btnIsPressed = true;
        btnPressStart = now;
        btnLongPressTriggered = false;
    } else if (pressed && btnIsPressed) {
        if (!btnLongPressTriggered && (now - btnPressStart >= 2000)) {
            btnLongPressTriggered = true;
            if (isWebServiceModeActive()) {
                logPrint("[BOOT BTN] 2s hold detected -> Deactivating Web Service Mode");
                deactivateWebServiceMode();
            } else {
                logPrint("[BOOT BTN] 2s hold detected -> Activating Web Service Mode!");
                activateWebServiceMode();
            }
        }
    } else if (!pressed && btnIsPressed) {
        btnIsPressed = false;
    }
}
