#include "keyboard_engine.h"
#include "logi_bolt.h"

static bool s_capsLockLedState = false;

void syncPhysicalKeyboardLedsForPc(const String& targetMac) {
    if (targetMac.length() == 0) return;
    uint8_t targetLeds = 0;
    for (int k = 0; k < MAX_SUPPORTED_KVM_CLIENTS; k++) {
        if (kvmClients[k].active && kvmClients[k].mac.equals(targetMac)) {
            targetLeds = kvmClients[k].ledState;
            break;
        }
    }
    s_capsLockLedState = (targetLeds & 0x02) ? true : false;
    if (logi_bolt_is_keyboard_connected()) {
        logi_bolt_set_keyboard_leds(targetLeds);
    }
    if (pKbLedChar && kbConnected) {
        bool useResponse = pKbLedChar->canWrite() && !pKbLedChar->canWriteNoResponse();
        bool ok = pKbLedChar->writeValue(&targetLeds, 1, useResponse);
        logPrint("[KEYBOARD LED] BLE write to Report ID 1 (resp: %d) -> res: %d | val: 0x%02X (Caps: %d)",
                 useResponse, ok, targetLeds, (targetLeds & 0x02) ? 1 : 0);
    }
    if (pKbBootLedChar && kbConnected) {
        bool useResponse = pKbBootLedChar->canWrite() && !pKbBootLedChar->canWriteNoResponse();
        bool ok = pKbBootLedChar->writeValue(&targetLeds, 1, useResponse);
        logPrint("[KEYBOARD LED] BLE write to Boot Output 0x2A32 (resp: %d) -> res: %d | val: 0x%02X (Caps: %d)",
                 useResponse, ok, targetLeds, (targetLeds & 0x02) ? 1 : 0);
    }
    logPrint("[KEYBOARD LED] Synced physical LEDs for PC %s -> 0x%02X (Caps: %d)",
             targetMac.c_str(), targetLeds, (targetLeds & 0x02) ? 1 : 0);
}

void checkAndSyncCapsLock(const uint8_t* rep8) {
    if (!rep8) return;
    static bool s_lastCapsLockPressed = false;
    bool currentCapsLockPressed = false;
    for (int k = 2; k < 8; k++) {
        if (rep8[k] == 0x39) {
            currentCapsLockPressed = true;
            break;
        }
    }
    if (currentCapsLockPressed && !s_lastCapsLockPressed) {
        String activeMac = (monitorCount > 0) ? monitors[currentMonitorIndex].mac : "";
        for (int i = 0; i < MAX_SUPPORTED_KVM_CLIENTS; i++) {
            if (kvmClients[i].active && kvmClients[i].mac.equals(activeMac)) {
                kvmClients[i].ledState ^= 0x02; // Toggle CapsLock bit for currently active PC
                break;
            }
        }
        syncPhysicalKeyboardLedsForPc(activeMac);
    }
    s_lastCapsLockPressed = currentCapsLockPressed;
}

void KeyboardOutputCallbacks::onWrite(NimBLECharacteristic* pCharacteristic, ble_gap_conn_desc* desc) {
    std::string val = pCharacteristic->getValue();
    if (val.length() > 0) {
        uint8_t leds = (uint8_t)val[0];
        uint16_t connHandle = desc ? desc->conn_handle : BLE_HS_CONN_HANDLE_NONE;
        
        String senderMac = "";
        for (int i = 0; i < MAX_SUPPORTED_KVM_CLIENTS; i++) {
            if (kvmClients[i].conn_id == connHandle) {
                kvmClients[i].ledState = leds;
                senderMac = kvmClients[i].mac;
                break;
            }
        }
        
        String activeMac = (monitorCount > 0) ? monitors[currentMonitorIndex].mac : "";
        bool isActivePc = (senderMac.length() > 0 && senderMac.equals(activeMac));
        
        logPrint("[KEYBOARD LED] PC %s (conn %d) sent LED state: 0x%02X (Caps: %d, Active: %s)",
                 senderMac.c_str(), connHandle, leds, (leds & 0x02) ? 1 : 0, isActivePc ? "YES" : "NO");
        
        if (isActivePc || senderMac.length() == 0) {
            syncPhysicalKeyboardLedsForPc(activeMac.length() > 0 ? activeMac : senderMac);
        }
    }
}

// Dynamic Modifier Remapping for Mac / Windows
// Logitech keyboards (like MX Keys) send standard PC/Windows modifier codes:
// - Key next to Spacebar sends Left Alt (0x04)
// - Key to the left of it (Start) sends Left GUI/Win (0x08)
// On macOS, users expect the key next to Spacebar (labeled 'cmd') to act as Command (0x08),
// and the 'opt' key to act as Option (0x04).
// When target OS is Mac, swap Left Alt <-> Left GUI and Right Alt <-> Right GUI.
uint8_t remapModifiersForTargetOs(uint8_t mods, int targetOs) {
    if (targetOs == OS_MAC) {
        uint8_t remapped = mods & ~(0x04 | 0x08 | 0x40 | 0x80);
        if (mods & 0x04) remapped |= 0x08; // Left Alt -> Left GUI (Command)
        if (mods & 0x08) remapped |= 0x04; // Left GUI (Win) -> Left Alt (Option)
        if (mods & 0x40) remapped |= 0x80; // Right Alt -> Right GUI (Command)
        if (mods & 0x80) remapped |= 0x40; // Right GUI -> Right Alt (Option)
        return remapped;
    }
    return mods;
}

// Dynamic Navigation Key Remapping for Mac:
// On macOS, Home (0x4A) and End (0x4D) scroll the window instead of moving the cursor.
// Remap to match standard Windows text navigation:
// - Home        -> Cmd + Left Arrow (0x50)  (beginning of line)
// - End         -> Cmd + Right Arrow (0x4F) (end of line)
// - Ctrl + Home -> Cmd + Up Arrow (0x52)    (beginning of document)
// - Ctrl + End  -> Cmd + Down Arrow (0x51)  (end of document)
// Modifiers like Shift (for selection) are preserved automatically.
void remapKeysForTargetOs(uint8_t* rep8, int targetOs) {
    if (targetOs != OS_MAC || !rep8) return;

    for (int i = 2; i < 8; i++) {
        if (rep8[i] == 0x4A) { // Home
            bool hasCtrl = (rep8[0] & (0x01 | 0x10)) != 0;
            if (hasCtrl) {
                rep8[0] &= ~(0x01 | 0x10); // Strip Ctrl
                rep8[0] |= 0x08;           // Add Left GUI (Cmd)
                rep8[i] = 0x52;            // Up Arrow
            } else {
                rep8[0] |= 0x08;           // Add Left GUI (Cmd)
                rep8[i] = 0x50;            // Left Arrow
            }
        } else if (rep8[i] == 0x4D) { // End
            bool hasCtrl = (rep8[0] & (0x01 | 0x10)) != 0;
            if (hasCtrl) {
                rep8[0] &= ~(0x01 | 0x10); // Strip Ctrl
                rep8[0] |= 0x08;           // Add Left GUI (Cmd)
                rep8[i] = 0x51;            // Down Arrow
            } else {
                rep8[0] |= 0x08;           // Add Left GUI (Cmd)
                rep8[i] = 0x4F;            // Right Arrow
            }
        } else if (rep8[i] == 0x46) { // Print Screen -> Cmd + Shift + 4 on macOS
            rep8[0] |= (0x08 | 0x02); // Add Left GUI (Cmd) + Left Shift
            rep8[i] = 0x21;           // Key '4'
        }
    }
}

// --- macOS Globe Key Pulse Generator (AC Keyboard Layout Select 0x029D) ---
void sendGlobePulseToMac(uint16_t connHandle) {
    if (connHandle == BLE_HS_CONN_HANDLE_NONE) return;
    xTaskCreate([](void* param) {
        uint16_t conn = (uint16_t)(uintptr_t)param;
        // 1. Send clean key release on standard keyboard so no modifiers interfere
        uint8_t zeroKey[8] = { 0 };
        sendHidReport(keyboardInputChar, conn, zeroKey, 8);
        vTaskDelay(pdMS_TO_TICKS(10));

        // 2. Send Globe key press (Consumer Page 0x0C, Usage 0x029D)
        uint8_t globePress[2] = { 0x9D, 0x02 };
        sendHidReport(mediaInputChar, conn, globePress, 2);
        logPrint("[GLOBE KEY] Sent Globe (0x029D) press to macOS (conn: %d)", conn);

        vTaskDelay(pdMS_TO_TICKS(40));

        // 3. Send Globe key release
        uint8_t globeRelease[2] = { 0x00, 0x00 };
        sendHidReport(mediaInputChar, conn, globeRelease, 2);
        logPrint("[GLOBE KEY] Sent Globe (0x029D) release to macOS (conn: %d)", conn);

        vTaskDelete(NULL);
    }, "globeTask", 3072, (void*)(uintptr_t)connHandle, 1, NULL);
}

// --- macOS Calculator Shortcut Generator (Control + Option + Command + C) ---
static bool s_calcTaskBusy = false;

void sendMacCalculatorShortcut(uint16_t connHandle) {
    if (connHandle == BLE_HS_CONN_HANDLE_NONE) return;
    if (s_calcTaskBusy) return;
    s_calcTaskBusy = true;
    xTaskCreate([](void* param) {
        uint16_t conn = (uint16_t)(uintptr_t)param;
        // Press Control + Option + Command + C: Modifiers: 0x01 (Ctrl) | 0x04 (Option) | 0x08 (Cmd) = 0x0D, Key: 0x06 ('c')
        uint8_t pressReport[8] = { 0x0D, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00 };
        sendHidReport(keyboardInputChar, conn, pressReport, sizeof(pressReport));
        logPrint("[MAC CALC] Sent Ctrl + Option + Cmd + C press to macOS (conn: %d)", conn);

        vTaskDelay(pdMS_TO_TICKS(50));

        // Release keys
        uint8_t releaseReport[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
        sendHidReport(keyboardInputChar, conn, releaseReport, sizeof(releaseReport));
        logPrint("[MAC CALC] Sent Ctrl + Option + Cmd + C release to macOS (conn: %d)", conn);

        s_calcTaskBusy = false;
        vTaskDelete(NULL);
    }, "calcTask", 4096, (void*)(uintptr_t)connHandle, 1, NULL);
}

// --- macOS Screen Snip Shortcut Generator (Cmd + Shift + 4) ---
static bool s_snipTaskBusy = false;

void sendMacScreenshotShortcut(uint16_t connHandle) {
    if (connHandle == BLE_HS_CONN_HANDLE_NONE) return;
    if (s_snipTaskBusy) return;
    s_snipTaskBusy = true;
    xTaskCreate([](void* param) {
        uint16_t conn = (uint16_t)(uintptr_t)param;
        uint8_t zeroReport[8] = { 0 };
        sendHidReport(keyboardInputChar, conn, zeroReport, sizeof(zeroReport));
        vTaskDelay(pdMS_TO_TICKS(10));

        // Press Cmd + Shift + 4: Modifiers: 0x08 (Cmd) | 0x02 (Shift) = 0x0A, Key: 0x21 ('4')
        uint8_t pressReport[8] = { 0x0A, 0x00, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00 };
        sendHidReport(keyboardInputChar, conn, pressReport, sizeof(pressReport));
        logPrint("[MAC SNIP] Sent Cmd + Shift + 4 press to macOS (conn: %d)", conn);

        vTaskDelay(pdMS_TO_TICKS(60));

        // Release keys
        sendHidReport(keyboardInputChar, conn, zeroReport, sizeof(zeroReport));
        logPrint("[MAC SNIP] Sent Cmd + Shift + 4 release to macOS (conn: %d)", conn);

        s_snipTaskBusy = false;
        vTaskDelete(NULL);
    }, "snipTask", 4096, (void*)(uintptr_t)connHandle, 1, NULL);
}

// --- macOS Lock Screen Shortcut Generator (Control + Command + Q) ---
static bool s_lockTaskBusy = false;

void sendMacLockShortcut(uint16_t connHandle) {
    if (connHandle == BLE_HS_CONN_HANDLE_NONE) return;
    if (s_lockTaskBusy) return;
    s_lockTaskBusy = true;
    xTaskCreate([](void* param) {
        uint16_t conn = (uint16_t)(uintptr_t)param;
        uint8_t zeroReport[8] = { 0 };
        sendHidReport(keyboardInputChar, conn, zeroReport, sizeof(zeroReport));
        vTaskDelay(pdMS_TO_TICKS(10));

        // Press Control + Command + Q: Modifiers: 0x01 (Ctrl) | 0x08 (Cmd) = 0x09, Key: 0x14 ('q')
        uint8_t pressReport[8] = { 0x09, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00 };
        sendHidReport(keyboardInputChar, conn, pressReport, sizeof(pressReport));
        logPrint("[MAC LOCK] Sent Ctrl + Cmd + Q press to macOS (conn: %d)", conn);

        vTaskDelay(pdMS_TO_TICKS(60));

        // Release keys
        sendHidReport(keyboardInputChar, conn, zeroReport, sizeof(zeroReport));
        logPrint("[MAC LOCK] Sent Ctrl + Cmd + Q release to macOS (conn: %d)", conn);

        s_lockTaskBusy = false;
        vTaskDelete(NULL);
    }, "lockTask", 4096, (void*)(uintptr_t)connHandle, 1, NULL);
}

// --- macOS Hardware Macro Interception (Win+Shift+S -> Cmd+Shift+4, Win+L -> Ctrl+Cmd+Q) ---
static bool s_snipSuppressUntilRelease = false;
static bool s_lockSuppressUntilRelease = false;

static bool checkSpecialMacCombos(uint8_t rawMods, uint8_t* rep8, int targetOs, uint16_t targetConn) {
    if (targetOs != OS_MAC) return false;

    bool hasKeys = false;
    for (int i = 2; i < 8; i++) {
        if (rep8[i] != 0) {
            hasKeys = true;
            break;
        }
    }

    if (!hasKeys) {
        if (s_snipSuppressUntilRelease) {
            if (rawMods == 0) {
                s_snipSuppressUntilRelease = false;
                return false; // Forward final clean all-zeros release
            }
            return true; // Suppress trailing modifier frames
        }
        if (s_lockSuppressUntilRelease) {
            if (rawMods == 0) {
                s_lockSuppressUntilRelease = false;
                return false; // Forward final clean all-zeros release
            }
            return true; // Suppress trailing modifier frames
        }
        return false;
    }

    // 1. Check Win + Shift + S -> Screen Snip on macOS (Cmd + Shift + 4)
    if ((rawMods & (0x08 | 0x80)) && (rawMods & (0x02 | 0x20))) { // Win + Shift
        for (int i = 2; i < 8; i++) {
            if (rep8[i] == 0x16) { // 's'
                logPrint("[MAC SNIP] Intercepted Win+Shift+S -> Triggering Cmd+Shift+4 on macOS!");
                s_snipSuppressUntilRelease = true;
                sendMacScreenshotShortcut(targetConn);
                return true;
            }
        }
    }

    // 2. Check Win + L -> Lock Screen on macOS (Control + Command + Q)
    if ((rawMods & (0x08 | 0x80)) && !(rawMods & (0x01 | 0x02 | 0x04 | 0x10 | 0x20 | 0x40))) { // Win alone
        for (int i = 2; i < 8; i++) {
            if (rep8[i] == 0x0F) { // 'l'
                logPrint("[MAC LOCK] Intercepted Win+L -> Triggering Ctrl+Cmd+Q on macOS!");
                s_lockSuppressUntilRelease = true;
                sendMacLockShortcut(targetConn);
                return true;
            }
        }
    }

    return false;
}

// --- macOS Ctrl+Shift -> Globe Key Detection ---
static bool s_ctrlShiftArmed = false;
static bool s_ctrlShiftOtherKey = false;
static uint8_t s_lastRawMods = 0;

void checkCtrlShiftGlobeTrigger(uint8_t rawMods, const uint8_t* rep8, int targetOs, uint16_t targetConn) {
    if (targetOs != OS_MAC) {
        s_ctrlShiftArmed = false;
        s_ctrlShiftOtherKey = false;
        s_lastRawMods = 0;
        return;
    }

    bool hasCtrl = (rawMods & (0x01 | 0x10)) != 0;
    bool hasShift = (rawMods & (0x02 | 0x20)) != 0;
    bool hasOtherMods = (rawMods & ~(0x01 | 0x02 | 0x10 | 0x20)) != 0; // Alt or GUI pressed
    bool prevHadOtherMods = (s_lastRawMods & ~(0x01 | 0x02 | 0x10 | 0x20)) != 0;

    bool hasKeys = false;
    if (rep8) {
        for (int k = 2; k < 8; k++) {
            if (rep8[k] != 0) {
                hasKeys = true;
                break;
            }
        }
    }

    if (hasCtrl && hasShift && !hasOtherMods) {
        if (!hasKeys) {
            // Only arm if we didn't just step down from a combo that had Alt or Win
            if (!s_ctrlShiftArmed && !prevHadOtherMods) {
                s_ctrlShiftArmed = true;
                s_ctrlShiftOtherKey = false; // Reset: Fresh clean Ctrl+Shift engagement
            }
        } else {
            s_ctrlShiftOtherKey = true; // Non-modifier key was pressed while holding Ctrl+Shift (e.g. Ctrl+Shift+T)
        }
    } else {
        if (s_ctrlShiftArmed) {
            // Only trigger Globe if clean release without adding Alt/Win or other keys
            if (!s_ctrlShiftOtherKey && !hasOtherMods && !hasKeys) {
                logPrint("[GLOBE KEY] Ctrl+Shift release detected -> Triggering Globe key on macOS!");
                sendGlobePulseToMac(targetConn);
            }
            s_ctrlShiftArmed = false;
            s_ctrlShiftOtherKey = false;
        }
    }

    s_lastRawMods = rawMods;
}

// --- Windows Ctrl+Shift Release Dwell Enforcement ---
// --- Windows Ctrl+Shift Language Switch Dwell Enforcer ---
// Prevents Windows kernel from releasing both modifiers within <15.6ms (the Windows scheduler quantum),
// ensuring ctfmon.exe in user space reliably registers the modifier transition and switches language.
static bool s_winCtrlShiftActive = false;
static bool s_winCtrlShiftInterrupted = false;
static bool s_winDwellPending = false;
static volatile bool s_winDwellActive = false;
static volatile uint32_t s_winDwellReleaseTime = 0;
static volatile uint16_t s_winDwellConn = BLE_HS_CONN_HANDLE_NONE;

void checkWindowsCtrlShiftDwell() {
    if (s_winDwellActive && (int32_t)(millis() - s_winDwellReleaseTime) >= 0) {
        s_winDwellActive = false;
        uint16_t conn = s_winDwellConn;
        if (conn != BLE_HS_CONN_HANDLE_NONE) {
            uint8_t zeroReport[8] = { 0 };
            sendHidReport(keyboardInputChar, conn, zeroReport, sizeof(zeroReport));
            logPrint("[WIN LANG] Delivered final release after 35ms dwell -> Language switch guaranteed!");
        }
    }
}

static bool handleWindowsCtrlShiftDwell(uint8_t rawMods, const uint8_t* rep8, int targetOs, uint16_t targetConn) {
    if (targetOs != OS_WINDOWS) {
        s_winCtrlShiftActive = false;
        s_winCtrlShiftInterrupted = false;
        s_winDwellPending = false;
        s_winDwellActive = false;
        return false;
    }

    bool hasCtrl = (rawMods & (0x01 | 0x10)) != 0;
    bool hasShift = (rawMods & (0x02 | 0x20)) != 0;
    bool hasOtherMods = (rawMods & ~(0x01 | 0x02 | 0x10 | 0x20)) != 0;

    bool hasKeys = false;
    if (rep8) {
        for (int k = 2; k < 8; k++) {
            if (rep8[k] != 0) {
                hasKeys = true;
                break;
            }
        }
    }

    // If a dwell is active and new key activity arrives, cancel the dwell
    if (s_winDwellActive) {
        if (hasKeys || rawMods != 0) {
            s_winDwellActive = false;
        } else {
            return true; // Already awaiting release; suppress duplicate 0x00
        }
    }

    if (hasKeys || hasOtherMods) {
        s_winCtrlShiftActive = false;
        s_winCtrlShiftInterrupted = true;
        s_winDwellPending = false;
        return false;
    }

    // Both Ctrl and Shift held cleanly without any other keys
    if (hasCtrl && hasShift) {
        s_winCtrlShiftActive = true;
        s_winCtrlShiftInterrupted = false;
        s_winDwellPending = false;
        return false;
    }

    if (s_winCtrlShiftActive && !s_winCtrlShiftInterrupted) {
        // One of the modifiers was just released (e.g. Shift released -> 0x01, or Ctrl released -> 0x02)
        if ((hasCtrl && !hasShift) || (!hasCtrl && hasShift)) {
            s_winDwellPending = true;
            return false; // Forward this first release immediately
        }

        // Both modifiers released
        if (!hasCtrl && !hasShift) {
            bool wasPending = s_winDwellPending;
            s_winCtrlShiftActive = false;
            s_winDwellPending = false;

            if (!wasPending) {
                // If both were released simultaneously in the same packet, send Shift release first
                uint8_t shiftUpReport[8] = { 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
                sendHidReport(keyboardInputChar, targetConn, shiftUpReport, sizeof(shiftUpReport));
            }

            // Schedule final all-zeros release after 35ms dwell in loop() (zero task stack overhead!)
            s_winDwellConn = targetConn;
            s_winDwellReleaseTime = millis() + 35;
            s_winDwellActive = true;

            return true; // We intercepted the premature 0x00; loop() will deliver it after 35ms dwell!
        }
    }

    return false;
}

static bool s_isAnyKeyboardKeyPressed = false;

bool isAnyKeyboardKeyPressed() {
    return s_isAnyKeyboardKeyPressed;
}

void resetKeyboardPressedState() {
    s_isAnyKeyboardKeyPressed = false;
    s_winDwellActive = false;
    s_winCtrlShiftActive = false;
    s_winDwellPending = false;
    s_ctrlShiftArmed = false;
    s_ctrlShiftOtherKey = false;
    s_lastRawMods = 0;
}

static bool checkReportHasActiveKeys(const uint8_t* pData, size_t length) {
    if (!pData || length == 0) return false;
    if (length == 7) {
        if (pData[0] != 0) return true; // Modifiers
        for (size_t i = 1; i < 7; i++) {
            if (pData[i] != 0) return true;
        }
        return false;
    } else if (length == 8) {
        if (pData[0] != 0) return true; // Modifiers
        for (size_t i = 2; i < 8; i++) {
            if (pData[i] != 0) return true;
        }
        return false;
    } else if (length == 9) {
        if (pData[1] != 0) return true; // Modifiers
        for (size_t i = 3; i < 9; i++) {
            if (pData[i] != 0) return true;
        }
        return false;
    } else if (length >= 15 && length <= 17) {
        if (pData[0] != 0) return true; // Modifiers
        for (size_t i = 1; i < length; i++) {
            if (pData[i] != 0) return true;
        }
        return false;
    } else if (length >= 3 && length <= 5 && (pData[0] == 0x03 || pData[0] == 0x04)) {
        // Consumer Control / Media report with Report ID at byte 0
        for (size_t i = 1; i < length; i++) {
            if (pData[i] != 0) return true;
        }
        return false;
    } else if (length == 2) {
        // Consumer Control report without Report ID (2 bytes)
        return (pData[0] != 0 || pData[1] != 0);
    } else if (length == 19 || pData[0] == 0xFF) {
        // Logitech HID++ vendor packet: keep existing state
        return s_isAnyKeyboardKeyPressed;
    } else {
        for (size_t i = 0; i < length; i++) {
            if (pData[i] != 0) return true;
        }
        return false;
    }
}

// Logitech MX Keys BLE Characteristic Handles
constexpr uint16_t LOGI_MX_KEYS_CONSUMER_HDL  = 0x003C; // Consumer Control (Media, Brightness, Volume, Calculator)
constexpr uint16_t LOGI_MX_KEYS_TELEPHONY_HDL = 0x0040; // Telephony Control (F7: Microphone Mute)

// Logitech MX Keys BLE Consumer Bitmask Values (received on LOGI_MX_KEYS_CONSUMER_HDL)
constexpr uint16_t LOGI_BLE_KEY_BRIGHTNESS_DOWN = 0x0001; // F1: Brightness Decrement / Down (byte 0: 0x01)
constexpr uint16_t LOGI_BLE_KEY_BRIGHTNESS_UP   = 0x0002; // F2: Brightness Increment / Up (byte 0: 0x02)
constexpr uint16_t LOGI_BLE_KEY_PREV_TRACK      = 0x0020; // F8: Previous Track (byte 0: 0x20)
constexpr uint16_t LOGI_BLE_KEY_PLAY_PAUSE      = 0x0040; // F9: Play / Pause (byte 0: 0x40)
constexpr uint16_t LOGI_BLE_KEY_NEXT_TRACK      = 0x0080; // F10: Next Track (byte 0: 0x80)
constexpr uint16_t LOGI_BLE_KEY_MUTE            = 0x0100; // F11: Mute (byte 1: 0x01)
constexpr uint16_t LOGI_BLE_KEY_VOLUME_DOWN     = 0x0200; // F12: Volume Decrement / Down (byte 1: 0x02)
constexpr uint16_t LOGI_BLE_KEY_VOLUME_UP       = 0x0400; // 13-th: Volume Increment / Up (byte 1: 0x04)
constexpr uint16_t LOGI_BLE_KEY_CALCULATOR      = 0x0800; // Calculator (byte 1: 0x08)

// Standard HID Consumer & Telephony Usages (Usage Pages 0x0C & 0x0B)
constexpr uint16_t HID_USAGE_CONSUMER_BRIGHTNESS_DOWN = 0x0070; // Display Brightness Decrement
constexpr uint16_t HID_USAGE_CONSUMER_BRIGHTNESS_UP   = 0x006F; // Display Brightness Increment
constexpr uint16_t HID_USAGE_CONSUMER_SCAN_PREV_TRACK = 0x00B6; // Scan Previous Track
constexpr uint16_t HID_USAGE_CONSUMER_PLAY_PAUSE      = 0x00CD; // Play / Pause
constexpr uint16_t HID_USAGE_CONSUMER_SCAN_NEXT_TRACK = 0x00B5; // Scan Next Track
constexpr uint16_t HID_USAGE_CONSUMER_MUTE            = 0x00E2; // Mute (Audio)
constexpr uint16_t HID_USAGE_CONSUMER_VOLUME_DOWN     = 0x00EA; // Volume Decrement
constexpr uint16_t HID_USAGE_CONSUMER_VOLUME_UP       = 0x00E9; // Volume Increment
constexpr uint16_t HID_USAGE_CONSUMER_AL_CALCULATOR   = 0x0192; // Application Launch Calculator
constexpr uint16_t HID_USAGE_CONSUMER_AL_LOCK         = 0x019E; // AL Terminal Lock / Screensaver
constexpr uint16_t HID_USAGE_SYSTEM_SLEEP             = 0x0034; // System Sleep
constexpr uint16_t HID_USAGE_TELEPHONY_PHONE_MUTE     = 0x028A; // Phone Mute / Microphone Mute

static uint16_t translateConsumerUsage(uint16_t rawUsage) {
    switch (rawUsage) {
        // Logitech MX Keys direct BLE Consumer Bitmask -> Standard HID Consumer Usage ID
        case LOGI_BLE_KEY_BRIGHTNESS_DOWN: return HID_USAGE_CONSUMER_BRIGHTNESS_DOWN; // F1: Brightness Decrement / Down
        case LOGI_BLE_KEY_BRIGHTNESS_UP:   return HID_USAGE_CONSUMER_BRIGHTNESS_UP;   // F2: Brightness Increment / Up
        // F3: Keyboard Backlight Down (handled internally by hardware via HID++ 19B)
        // F4: Keyboard Backlight Up (handled internally by hardware via HID++ 19B)
        // F5: Dictation (sends standard keyboard Win+H 7B)
        // F6: Emoji (sends standard keyboard Ctrl+Shift+Alt+Win+Space 7B)
        // F7: Mic Mute (sends on dedicated LOGI_MX_KEYS_TELEPHONY_HDL)
        case LOGI_BLE_KEY_PREV_TRACK:      return HID_USAGE_CONSUMER_SCAN_PREV_TRACK; // F8: Scan Previous Track
        case LOGI_BLE_KEY_PLAY_PAUSE:      return HID_USAGE_CONSUMER_PLAY_PAUSE;      // F9: Play / Pause
        case LOGI_BLE_KEY_NEXT_TRACK:      return HID_USAGE_CONSUMER_SCAN_NEXT_TRACK; // F10: Scan Next Track
        case LOGI_BLE_KEY_MUTE:            return HID_USAGE_CONSUMER_MUTE;            // F11: Mute (Audio)
        case LOGI_BLE_KEY_VOLUME_DOWN:     return HID_USAGE_CONSUMER_VOLUME_DOWN;     // F12: Volume Decrement / Down
        case LOGI_BLE_KEY_VOLUME_UP:       return HID_USAGE_CONSUMER_VOLUME_UP;       // 13-th: Volume Increment / Up
        case LOGI_BLE_KEY_CALCULATOR:      return HID_USAGE_CONSUMER_AL_CALCULATOR;   // Calculator
        default:                           return rawUsage;                           // Already standard 16-bit Usage ID or release (0x0000)
    }
}

// Callback when HID data is received from the keyboard (Follow-the-Mouse)
void keyboardNotifyCallback(NimBLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    if (!pData || length == 0) return;

    // Track whether any keyboard key or modifier is currently held
    if (length != 19 && pData[0] != 0xFF) {
        s_isAnyKeyboardKeyPressed = checkReportHasActiveKeys(pData, length);
    }

    // Log the raw incoming keyboard packet
    String hexDump = "";
    for (size_t i = 0; i < length; i++) {
        char buf[4];
        snprintf(buf, sizeof(buf), "%02X ", pData[i]);
        hexDump += buf;
    }
    uint16_t charHandle = pBLERemoteCharacteristic ? pBLERemoteCharacteristic->getHandle() : 0;
    logPrint("[KEYBOARD RX RAW] %s (len: %d, hdl: 0x%04X)", hexDump.c_str(), length, charHandle);

    if (monitorCount == 0) return;

    uint16_t targetConn = getTargetConnHandle(monitors[currentMonitorIndex].mac);
    if (targetConn == BLE_HS_CONN_HANDLE_NONE) {
        // Fallback to any active connected PC if current monitor target is not matched
        for (int i = 0; i < MAX_SUPPORTED_KVM_CLIENTS; i++) {
            if (kvmClients[i].active && kvmClients[i].conn_id != BLE_HS_CONN_HANDLE_NONE && isMacInActiveLayout(kvmClients[i].mac)) {
                targetConn = kvmClients[i].conn_id;
                break;
            }
        }
    }
    if (targetConn == BLE_HS_CONN_HANDLE_NONE) return;

    int targetOs = OS_WINDOWS;
    if (currentMonitorIndex < monitorCount) {
        targetOs = monitors[currentMonitorIndex].os;
    }

    if (length == 7) {
        // 7-byte report from Logitech MX Keys: [modifiers, key1, key2, key3, key4, key5, key6]
        // Standard HID 6KRO report requires 8 bytes: [modifiers, reserved(0x00), key1, key2, key3, key4, key5, key6]
        uint8_t rep8[8];
        rep8[0] = remapModifiersForTargetOs(pData[0], targetOs); // Modifiers (Shift, Ctrl, Alt, GUI)
        rep8[1] = 0x00;     // Reserved
        rep8[2] = pData[1]; // Key 1
        rep8[3] = pData[2]; // Key 2
        rep8[4] = pData[3]; // Key 3
        rep8[5] = pData[4]; // Key 4
        rep8[6] = pData[5]; // Key 5
        rep8[7] = pData[6]; // Key 6
        remapKeysForTargetOs(rep8, targetOs);
        if (checkSpecialMacCombos(pData[0], rep8, targetOs, targetConn)) return;
        if (handleWindowsCtrlShiftDwell(pData[0], rep8, targetOs, targetConn)) return;
        sendHidReport(keyboardInputChar, targetConn, rep8, 8);
        checkAndSyncCapsLock(rep8);
        checkCtrlShiftGlobeTrigger(pData[0], rep8, targetOs, targetConn);
        logPrint("[KEYBOARD FWD] 7B->8B [Mods: 0x%02X, Key1: 0x%02X] -> Conn %d (Mon #%d, OS: %s)",
                 rep8[0], rep8[2], targetConn, currentMonitorIndex + 1, targetOs == OS_MAC ? "Mac" : "Win");
    } else if (length == 8) {
        // Standard 8-byte keyboard report: [mods, res, k1, k2, k3, k4, k5, k6]
        uint8_t rep8[8];
        memcpy(rep8, pData, 8);
        rep8[0] = remapModifiersForTargetOs(pData[0], targetOs);
        remapKeysForTargetOs(rep8, targetOs);
        if (checkSpecialMacCombos(pData[0], rep8, targetOs, targetConn)) return;
        if (handleWindowsCtrlShiftDwell(pData[0], rep8, targetOs, targetConn)) return;
        sendHidReport(keyboardInputChar, targetConn, rep8, length);
        checkAndSyncCapsLock(rep8);
        checkCtrlShiftGlobeTrigger(pData[0], rep8, targetOs, targetConn);
        logPrint("[KEYBOARD FWD] 8B [Mods: 0x%02X, Key1: 0x%02X] -> Conn %d (Mon #%d, OS: %s)",
                 rep8[0], rep8[2], targetConn, currentMonitorIndex + 1, targetOs == OS_MAC ? "Mac" : "Win");
    } else if (length == 9) {
        // 9-byte report with Report ID prepended: forward payload without Report ID
        uint8_t rep8[8];
        memcpy(rep8, &pData[1], 8);
        rep8[0] = remapModifiersForTargetOs(pData[1], targetOs);
        remapKeysForTargetOs(rep8, targetOs);
        if (checkSpecialMacCombos(pData[1], rep8, targetOs, targetConn)) return;
        if (handleWindowsCtrlShiftDwell(pData[1], rep8, targetOs, targetConn)) return;
        sendHidReport(keyboardInputChar, targetConn, rep8, 8);
        checkAndSyncCapsLock(rep8);
        checkCtrlShiftGlobeTrigger(pData[1], rep8, targetOs, targetConn);
        logPrint("[KEYBOARD FWD] 9B (ID 0x%02X) [Mods: 0x%02X, Key1: 0x%02X] -> Conn %d (Mon #%d, OS: %s)",
                 pData[0], rep8[0], rep8[2], targetConn, currentMonitorIndex + 1, targetOs == OS_MAC ? "Mac" : "Win");
    } else if (length >= 15 && length <= 17) {
        // 16-byte Bitmap / NKRO Keyboard report from Logitech Bolt Receiver:
        // Byte 0: Modifiers (Ctrl, Shift, Alt, GUI)
        // Bytes 1..15: 120-bit key mask starting at HID usage 0x04 ('a')
        uint8_t rep8[8] = {0};
        rep8[0] = remapModifiersForTargetOs(pData[0], targetOs); // Modifiers
        rep8[1] = 0x00;     // Reserved
        
        int keyIndex = 2;
        for (int byteIdx = 1; byteIdx < (int)length && keyIndex < 8; byteIdx++) {
            uint8_t b = pData[byteIdx];
            if (b == 0) continue;
            for (int bitIdx = 0; bitIdx < 8 && keyIndex < 8; bitIdx++) {
                if (b & (1 << bitIdx)) {
                    uint8_t hidCode = (uint8_t)((byteIdx - 1) * 8 + bitIdx + 4);
                    rep8[keyIndex++] = hidCode;
                }
            }
        }
        remapKeysForTargetOs(rep8, targetOs);
        if (checkSpecialMacCombos(pData[0], rep8, targetOs, targetConn)) return;
        if (handleWindowsCtrlShiftDwell(pData[0], rep8, targetOs, targetConn)) return;
        sendHidReport(keyboardInputChar, targetConn, rep8, 8);
        checkAndSyncCapsLock(rep8);
        checkCtrlShiftGlobeTrigger(pData[0], rep8, targetOs, targetConn);
        logPrint("[KEYBOARD FWD] Bolt Bitmap %dB->8B [Mods: 0x%02X, Keys: %02X %02X %02X] -> Conn %d (OS: %s)",
                 (int)length, rep8[0], rep8[2], rep8[3], rep8[4], targetConn, targetOs == OS_MAC ? "Mac" : "Win");
    } else if (length == 2) {
        // Dedicated Telephony / Mic Mute characteristic (F7 on Logitech MX Keys)
        if (charHandle == LOGI_MX_KEYS_TELEPHONY_HDL) {
            uint16_t micUsage = (pData[0] != 0 || pData[1] != 0) ? HID_USAGE_TELEPHONY_PHONE_MUTE : 0x0000;
            uint8_t transRep[2] = { (uint8_t)(micUsage & 0xFF), (uint8_t)((micUsage >> 8) & 0xFF) };
            sendHidReport(mediaInputChar, targetConn, transRep, 2);
            logPrint("[MEDIA FWD] Mic Mute (0x%04X) %s -> Conn %d (Mon #%d, OS: %s)",
                     HID_USAGE_TELEPHONY_PHONE_MUTE, micUsage != 0 ? "PRESS" : "RELEASE",
                     targetConn, currentMonitorIndex + 1, targetOs == OS_MAC ? "Mac" : "Win");
            return;
        }

        // Consumer Control report (Media keys, 2 bytes)
        uint16_t rawUsage = (uint16_t)(pData[0] | (pData[1] << 8));
        uint16_t consumerUsage = translateConsumerUsage(rawUsage);
        if (consumerUsage == HID_USAGE_CONSUMER_AL_CALCULATOR && targetOs == OS_MAC) {
            sendMacCalculatorShortcut(targetConn);
        } else if ((consumerUsage == HID_USAGE_CONSUMER_AL_LOCK || consumerUsage == HID_USAGE_SYSTEM_SLEEP) && targetOs == OS_MAC) {
            sendMacLockShortcut(targetConn);
        } else {
            uint8_t transRep[2] = { (uint8_t)(consumerUsage & 0xFF), (uint8_t)((consumerUsage >> 8) & 0xFF) };
            sendHidReport(mediaInputChar, targetConn, transRep, 2);
            logPrint("[MEDIA FWD] 2B (Usage 0x%04X, Raw: 0x%04X) -> Conn %d (Mon #%d, OS: %s)",
                     consumerUsage, rawUsage, targetConn, currentMonitorIndex + 1, targetOs == OS_MAC ? "Mac" : "Win");
        }
    } else if ((length >= 3 && length <= 5 && (pData[0] == 0x03 || pData[0] == 0x04)) || length == 3) {
        // Consumer Control report with Report ID prepended
        uint16_t rawUsage = (uint16_t)(pData[1] | (pData[2] << 8));
        uint16_t consumerUsage = translateConsumerUsage(rawUsage);
        if (consumerUsage == HID_USAGE_CONSUMER_AL_CALCULATOR && targetOs == OS_MAC) {
            sendMacCalculatorShortcut(targetConn);
        } else if ((consumerUsage == HID_USAGE_CONSUMER_AL_LOCK || consumerUsage == HID_USAGE_SYSTEM_SLEEP) && targetOs == OS_MAC) {
            sendMacLockShortcut(targetConn);
        } else {
            uint8_t transRep[2] = { (uint8_t)(consumerUsage & 0xFF), (uint8_t)((consumerUsage >> 8) & 0xFF) };
            sendHidReport(mediaInputChar, targetConn, transRep, 2);
            logPrint("[MEDIA FWD] %dB (ID 0x%02X, Usage 0x%04X, Raw: 0x%04X) -> Conn %d (Mon #%d, OS: %s)",
                     (int)length, pData[0], consumerUsage, rawUsage, targetConn, currentMonitorIndex + 1, targetOs == OS_MAC ? "Mac" : "Win");
        }
    } else if (length == 19 || pData[0] == 0xFF) {
        // Logitech HID++ vendor packet: ignore
    } else {
        uint8_t repBuf[8];
        size_t copyLen = min((size_t)8, length);
        memcpy(repBuf, pData, copyLen);
        if (copyLen > 0) {
            repBuf[0] = remapModifiersForTargetOs(pData[0], targetOs);
            if (copyLen == 8) {
                remapKeysForTargetOs(repBuf, targetOs);
            }
            checkCtrlShiftGlobeTrigger(pData[0], repBuf, targetOs, targetConn);
        }
        sendHidReport(keyboardInputChar, targetConn, repBuf, copyLen);
        logPrint("[KEYBOARD FWD] %dB -> Conn %d (OS: %s)", (int)length, targetConn, targetOs == OS_MAC ? "Mac" : "Win");
    }
}
