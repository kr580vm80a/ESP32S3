#include "cursor_engine.h"
#include "keyboard_engine.h"
#include "logi_bolt.h"
#include <cmath>

static float subpixelX = 0.0f;
static float subpixelY = 0.0f;
static float effectiveSubpixelX = 0.0f;
static float effectiveSubpixelY = 0.0f;
static TaskHandle_t bootCalibTaskHandle = NULL;

void resetSubpixelAccumulators() {
    subpixelX = 0.0f;
    subpixelY = 0.0f;
    effectiveSubpixelX = 0.0f;
    effectiveSubpixelY = 0.0f;
}

MonitorConfig& primaryMonitor(const String& targetMac) {
    int primaryIndex = 0;
    for (int i = 0; i < monitorCount; i++) {
        if (monitors[i].isPrimary && monitors[i].mac.equals(targetMac)) {
            primaryIndex = i;
            break;
        }
    }
    return monitors[primaryIndex];
}

void sendAbsPosWindows(uint16_t connHandle, uint16_t absX, uint16_t absY) {
    uint8_t absReport[4] = {
        (uint8_t)(absX & 0xFF),
        (uint8_t)((absX >> 8) & 0xFF),
        (uint8_t)(absY & 0xFF),
        (uint8_t)((absY >> 8) & 0xFF)
    };
    sendHidReport(absInputChar, connHandle, absReport, sizeof(absReport));
    logPrint("Sent Window position at (%ld, %ld), Virtual: (%ld, %ld)", absX, absY, virtualX, virtualY);
}

void sendRelative12Bit(uint16_t connHandle, int32_t dx, int32_t dy, uint8_t buttons, int8_t scroll, int8_t hScroll) {
    do {
        int16_t curDx = (int16_t)constrain(dx, -2047, 2047);
        int16_t curDy = (int16_t)constrain(dy, -2047, 2047);
        uint16_t uX = (uint16_t)(curDx & 0x0FFF);
        uint16_t uY = (uint16_t)(curDy & 0x0FFF);
        uint8_t rep[6] = {
            (uint8_t)(buttons & 0x1F),
            (uint8_t)(uX & 0xFF),
            (uint8_t)(((uX >> 8) & 0x0F) | ((uY & 0x0F) << 4)),
            (uint8_t)((uY >> 4) & 0xFF),
            (uint8_t)constrain(scroll, -127, 127),
            (uint8_t)constrain(hScroll, -127, 127)
        };
        sendHidReport(inputChar, connHandle, rep, sizeof(rep));
        dx -= curDx;
        dy -= curDy;
        scroll = 0;
        hScroll = 0;
    } while (dx != 0 || dy != 0);
}

void sendAbsoluteCoordinatesWindows(uint16_t connHandle, int monIndex, long targetGlobalX, long targetGlobalY, const char* contextLabel) {
    MonitorConfig& targetMon = monitors[monIndex];
    if (targetMon.isPrimary) {
        long relX = constrain(targetGlobalX - targetMon.x, 0, targetMon.width);
        long relY = constrain(targetGlobalY - targetMon.y, 0, targetMon.height);
        uint16_t absX = (uint16_t)round(((float)relX / (float)targetMon.width) * 32767.0f);
        uint16_t absY = (uint16_t)round(((float)relY / (float)targetMon.height) * 32767.0f);
        uint8_t absReport[4] = {
            (uint8_t)(absX & 0xFF),
            (uint8_t)((absX >> 8) & 0xFF),
            (uint8_t)(absY & 0xFF),
            (uint8_t)((absY >> 8) & 0xFF)
        };
        sendHidReport(absInputChar, connHandle, absReport, sizeof(absReport));
        logPrint("[%s] Sent Windows position to PC %s at (%ld, %ld) [Rel: %ld, %ld -> Norm: %u, %u] on Mon #%d (%s)",
                contextLabel, targetMon.mac.c_str(), targetGlobalX, targetGlobalY, relX, relY, absX, absY, targetMon.id, targetMon.name.c_str());
        return;
    }
    MonitorConfig& primaryMon = primaryMonitor(targetMon.mac);
    int shift = 20;
    int jumpX = 0, jumpY = 0;
    int step1X = 0, step1Y = 0;
    int shiftX = 0, shiftY = 0;
    if (targetGlobalY < primaryMon.y) {
        jumpY = primaryMon.y;
        if (targetGlobalX <= (primaryMon.x)) {
            // use top left corner
            jumpX = primaryMon.x + shift;
            step1Y = -1;
            shiftX = -shift;
            shiftY = 1;
        } else if (targetGlobalX >= (primaryMon.x + primaryMon.width - 1)) {
            // use top right corner
            jumpX = primaryMon.x + primaryMon.width - 1 - shift;
            step1Y = -1;
            shiftX = shift;
            shiftY = 1;
        } else {
            // use top middle
            jumpX = targetGlobalX;
            jumpY = primaryMon.y;
        }
    } else if (targetGlobalY > (primaryMon.y + primaryMon.height - 1)) {
        jumpY = primaryMon.y + primaryMon.height - 1;
        if (targetGlobalX <= (primaryMon.x)) {
            // use bottom left corner
            jumpX = primaryMon.x + shift;
            step1Y = 1;
            shiftX = -shift;
            shiftY = -1;
        } else if (targetGlobalX >= (primaryMon.x + primaryMon.width - 1)) {
            // use bottom right corner
            jumpX = primaryMon.x + primaryMon.width - 1 - shift;
            step1Y = 1;
            shiftX = shift;
            shiftY = -1;
        } else {
            // use bottom middle
            jumpX = targetGlobalX;
            jumpY = primaryMon.y + primaryMon.height - 1;
        }
    } else {
        if (targetGlobalX < primaryMon.x) {
            jumpX = primaryMon.x;
            step1X = -1;
            shiftX = 1;
        } else {
            jumpX = primaryMon.x + primaryMon.width - 1;
            step1X = 1;
            shiftX = -1;
        }
        jumpY = primaryMon.y + (primaryMon.height / 2);
    }

    uint16_t absX = (uint16_t)round(((float)(jumpX - primaryMon.x) / (float)primaryMon.width) * 32767.0f);
    uint16_t absY = (uint16_t)round(((float)(jumpY - primaryMon.y) / (float)primaryMon.height) * 32767.0f);
    sendAbsPosWindows(connHandle, absX, absY);
    logPrint("Window jump position at (%ld, %ld)", jumpX, jumpY);

    if (step1X != 0 || step1Y != 0) {
        sendRelative12Bit(connHandle, step1X, step1Y);
        logPrint("Window step position at (%ld, %ld)", step1X, step1Y);
    }

    int32_t deltaX = targetGlobalX - jumpX + shiftX;
    int32_t deltaY = targetGlobalY - jumpY + shiftY;
    logPrint("Window move at (%ld, %ld)", deltaX, deltaY);
    sendRelative12Bit(connHandle, deltaX, deltaY);
}

// --- Simplified Absolute HID Positioning Function for macOS (MacBook at bottom) ---
void sendAbsoluteCoordinatesMacOs(uint16_t connHandle, int monIndex, long targetGlobalX, long targetGlobalY, const char* contextLabel) {
    uint8_t absReport1[5] = { 0x01, 0x00, 0x40, 0x00, 0x40 };
    sendHidReport(macAbsInputChar, connHandle, absReport1, sizeof(absReport1));
    absReport1[0] = 0x00; // In Range = OFF
    sendHidReport(macAbsInputChar, connHandle, absReport1, sizeof(absReport1));

    MonitorConfig& targetMon = monitors[monIndex];
    int shiftX = 0, shiftY = 0;
    if (targetMon.isPrimary) {
        shiftY = targetMon.height / 2 + 100;
    } else {
        shiftY = -targetMon.height / 2 - 100;
    }
    sendRelative12Bit(connHandle, 0, shiftY);
    long relX = constrain(targetGlobalX - targetMon.x, 0, targetMon.width);
    long relY = constrain(targetGlobalY - targetMon.y, 0, targetMon.height);
    uint16_t absX = (uint16_t)round(((float)relX / (float)targetMon.width) * 32767.0f);
    uint16_t absY = (uint16_t)round(((float)relY / (float)targetMon.height) * 32767.0f);
    uint8_t absReport[5] = {
        0x01,                               // In Range = ON
        (uint8_t)(absX & 0xFF),
        (uint8_t)((absX >> 8) & 0xFF),
        (uint8_t)(absY & 0xFF),
        (uint8_t)((absY >> 8) & 0xFF)
    };
    sendHidReport(macAbsInputChar, connHandle, absReport, sizeof(absReport));
    absReport[0] = 0x00;                    // In Range = OFF
    sendHidReport(macAbsInputChar, connHandle, absReport, sizeof(absReport));
    logPrint("[%s] Sent macOS digitizer position to PC %s at (%ld, %ld) [Rel: %ld, %ld -> Norm: %u, %u] on Mon #%d (%s)",
             contextLabel, targetMon.mac.c_str(), targetGlobalX, targetGlobalY, relX, relY, absX, absY, targetMon.id, targetMon.name.c_str());
}

// --- Simplified Absolute HID Positioning Function for macOS (MacBook at bottom) ---
void sendAbsoluteCoordinatesMacOs1(uint16_t connHandle, int monIndex, long targetGlobalX, long targetGlobalY, const char* contextLabel) {
    MonitorConfig& targetMon = monitors[monIndex];
    MonitorConfig& primaryMon = primaryMonitor(targetMon.mac);
    sendRelative12Bit(connHandle, 0, -5000);
    sendRelative12Bit(connHandle, -6000, 0);
    if (!targetMon.isPrimary) {
        if (targetMon.y > 0) sendRelative12Bit(connHandle, 0, targetMon.y + 100);
        if (targetMon.x > 0) sendRelative12Bit(connHandle, targetMon.x + 200, 0);
    } else {
        int32_t overMacX = primaryMon.x + (primaryMon.width / 2);
        long topMonitorY = 0;
        for (int i = 0; i < monitorCount; i++) {
            if (monitors[i].mac.equals(targetMon.mac) && !monitors[i].isPrimary) {
                if (overMacX >= monitors[i].x && overMacX < monitors[i].x + monitors[i].width) {
                    topMonitorY = monitors[i].y;
                    break;
                }
            }
        }
        if (topMonitorY > 0) sendRelative12Bit(connHandle, 0, topMonitorY + 100);
        if (overMacX > 0) sendRelative12Bit(connHandle, overMacX, 0);
        sendRelative12Bit(connHandle, 0, 3000);
    }
    long relX = constrain(targetGlobalX - targetMon.x, 0, targetMon.width);
    long relY = constrain(targetGlobalY - targetMon.y, 0, targetMon.height);
    uint16_t absX = (uint16_t)round(((float)relX / (float)targetMon.width) * 32767.0f);
    uint16_t absY = (uint16_t)round(((float)relY / (float)targetMon.height) * 32767.0f);
    uint8_t absReport[5] = {
        0x01,                               // In Range = ON
        (uint8_t)(absX & 0xFF),
        (uint8_t)((absX >> 8) & 0xFF),
        (uint8_t)(absY & 0xFF),
        (uint8_t)((absY >> 8) & 0xFF)
    };
    sendHidReport(macAbsInputChar, connHandle, absReport, sizeof(absReport));
    logPrint("[%s] Sent macOS1 digitizer pos to PC %s at (%ld, %ld) [Rel: %ld, %ld -> Norm: %u, %u] on Mon #%d (%s)",
             contextLabel, targetMon.mac.c_str(), targetGlobalX, targetGlobalY, relX, relY, absX, absY, targetMon.id, targetMon.name.c_str());
    sendRelative12Bit(connHandle, 0, 0);
}


// --- Smart Edge Reset & Positioning Function for Android ---
void sendAbsoluteCoordinatesAndroid(uint16_t connHandle, int monIndex, long targetGlobalX, long targetGlobalY, const char* contextLabel) {
    MonitorConfig& targetMon = monitors[monIndex];
    long relX = constrain(targetGlobalX - targetMon.x, 0, targetMon.width - 1);
    long relY = constrain(targetGlobalY - targetMon.y, 0, targetMon.height - 1);
    float scaleFactor = (targetMon.scale > 0) ? (targetMon.scale / 100.0f) : 1.0f;

    if (strcmp(contextLabel, "BOOT POSITION") == 0) {
        // Center position calibration
        sendRelative12Bit(connHandle, -6000, -6000);
        int32_t moveX = (int32_t)round(((float)targetMon.width / 2.0f) / scaleFactor);
        int32_t moveY = (int32_t)round(((float)targetMon.height / 2.0f) / scaleFactor);
        sendRelative12Bit(connHandle, moveX, moveY);
        logPrint("[%s] Android centered: rel (%ld, %ld), moved (%ld, %ld) on Mon #%d (%s)",
                 contextLabel, relX, relY, moveX, moveY, targetMon.id, targetMon.name.c_str());
        return;
    }

    // Determine entry border by finding the closest edge in the target monitor
    long distLeft   = relX;
    long distRight  = (targetMon.width - 1) - relX;
    long distTop    = relY;
    long distBottom = (targetMon.height - 1) - relY;

    long minDist = distLeft;
    const char* border = "LEFT";
    if (distRight < minDist) {
        minDist = distRight;
        border = "RIGHT";
    }
    if (distTop < minDist) {
        minDist = distTop;
        border = "TOP";
    }
    if (distBottom < minDist) {
        minDist = distBottom;
        border = "BOTTOM";
    }

    if (strcmp(border, "RIGHT") == 0) {
        // Entered from the RIGHT edge: pin X to right bezel, traverse Y from nearest corner
        if (relY <= targetMon.height / 2) {
            sendRelative12Bit(connHandle, 6000, -6000); // Top-Right corner
            int32_t moveY = (int32_t)round((float)relY / scaleFactor);
            sendRelative12Bit(connHandle, 100, moveY);
        } else {
            sendRelative12Bit(connHandle, 6000, 6000);  // Bottom-Right corner
            int32_t moveY = -(int32_t)round((float)distBottom / scaleFactor);
            sendRelative12Bit(connHandle, 100, moveY);
        }
    } else if (strcmp(border, "LEFT") == 0) {
        // Entered from the LEFT edge: pin X to left bezel, traverse Y from nearest corner
        if (relY <= targetMon.height / 2) {
            sendRelative12Bit(connHandle, -6000, -6000); // Top-Left corner
            int32_t moveY = (int32_t)round((float)relY / scaleFactor);
            sendRelative12Bit(connHandle, -100, moveY);
        } else {
            sendRelative12Bit(connHandle, -6000, 6000);  // Bottom-Left corner
            int32_t moveY = -(int32_t)round((float)distBottom / scaleFactor);
            sendRelative12Bit(connHandle, -100, moveY);
        }
    } else if (strcmp(border, "TOP") == 0) {
        // Entered from the TOP edge: pin Y to top bezel, traverse X from nearest corner
        if (relX <= targetMon.width / 2) {
            sendRelative12Bit(connHandle, -6000, -6000); // Top-Left corner
            int32_t moveX = (int32_t)round((float)relX / scaleFactor);
            sendRelative12Bit(connHandle, moveX, -100);
        } else {
            sendRelative12Bit(connHandle, 6000, -6000);  // Top-Right corner
            int32_t moveX = -(int32_t)round((float)distRight / scaleFactor);
            sendRelative12Bit(connHandle, moveX, -100);
        }
    } else { // BOTTOM
        // Entered from the BOTTOM edge: pin Y to bottom bezel, traverse X from nearest corner
        if (relX <= targetMon.width / 2) {
            sendRelative12Bit(connHandle, -6000, 6000);  // Bottom-Left corner
            int32_t moveX = (int32_t)round((float)relX / scaleFactor);
            sendRelative12Bit(connHandle, moveX, 100);
        } else {
            sendRelative12Bit(connHandle, 6000, 6000);   // Bottom-Right corner
            int32_t moveX = -(int32_t)round((float)distRight / scaleFactor);
            sendRelative12Bit(connHandle, moveX, 100);
        }
    }

    logPrint("[%s] Android edge reset via %s border: rel (%ld, %ld) on Mon #%d (%s)",
             contextLabel, border, relX, relY, targetMon.id, targetMon.name.c_str());
}

// --- Absolute HID Positioning Function ---
void sendAbsoluteCoordinates(uint16_t connHandle, int monIndex, long targetGlobalX, long targetGlobalY, const char* contextLabel) {
    if (monitors[monIndex].os == OS_MAC) {
        sendAbsoluteCoordinatesMacOs(connHandle, monIndex, targetGlobalX, targetGlobalY, contextLabel);
    // } else if (monitors[monIndex].os == OS_ANDROID) {
    //     sendAbsoluteCoordinatesAndroid(connHandle, monIndex, targetGlobalX, targetGlobalY, contextLabel);
    } else {
        sendAbsoluteCoordinatesWindows(connHandle, monIndex, targetGlobalX, targetGlobalY, contextLabel);
    }
}

// --- Boot Center Calibration Wrapper ---
void calibrateFirstConnectedPcToCenter(String targetMac) {
    uint16_t connHandle = getTargetConnHandle(targetMac);
    if (connHandle == BLE_HS_CONN_HANDLE_NONE) return;

    currentMonitorIndex = 0;
    for (int i = 0; i < monitorCount; i++) {
        if (monitors[i].isPrimary && monitors[i].mac.equals(targetMac)) {
            currentMonitorIndex = i;
            break;
        }
    }
    MonitorConfig& mon = monitors[currentMonitorIndex];
    virtualX = mon.x + (mon.width / 2);
    virtualY = mon.y + (mon.height / 2);
    sendAbsoluteCoordinates(connHandle, currentMonitorIndex, virtualX, virtualY, "BOOT POSITION");
    updateKvmPowerAndRateProfiles(mon.mac, true);
    syncPhysicalKeyboardLedsForPc(mon.mac);
}

void scheduleBootCalibration() {
    bool isMouseActive = mouseConnected || logi_bolt_is_mouse_connected();
    if (monitorCount == 0 || !isMouseActive || isCalibrated || firstConnectedPcMac.length() == 0) return;
    isCalibrated = true;
    if (bootCalibTaskHandle != NULL) {
        vTaskDelete(bootCalibTaskHandle);
        bootCalibTaskHandle = NULL;
    }
    xTaskCreate([](void* param) {
        String* pMac = (String*)param;
        vTaskDelay(pdMS_TO_TICKS(600));
        calibrateFirstConnectedPcToCenter(*pMac);
        delete pMac;
        bootCalibTaskHandle = NULL;
        vTaskDelete(NULL);
    }, "bootCalibTask", 3072, new String(firstConnectedPcMac), 1, &bootCalibTaskHandle);
}

/**
 * @brief Translates side mouse buttons (Back/Forward) to Cmd + [ and Cmd + ] for macOS.
 * @param buttons Raw mouse buttons bitmask.
 * @param connHandle Target BLE connection handle.
 * @return Filtered buttons bitmask with Back/Forward stripped for macOS.
 */
static uint8_t handleMacSideMouseButtons(uint8_t buttons, uint16_t connHandle) {
    static uint8_t s_lastMacNavButtons = 0;
    uint8_t pressed = (buttons & (0x08 | 0x10)) & ~s_lastMacNavButtons;
    uint8_t released = ~(buttons & (0x08 | 0x10)) & s_lastMacNavButtons;
    s_lastMacNavButtons = buttons & (0x08 | 0x10);

    if (keyboardInputChar && connHandle != BLE_HS_CONN_HANDLE_NONE) {
        // Button 4 (Back): 0x08 -> Cmd + [ (0x2F)
        if (pressed & 0x08) {
            uint8_t kbReport[8] = { 0x08, 0x00, 0x2F, 0x00, 0x00, 0x00, 0x00, 0x00 };
            sendHidReport(keyboardInputChar, connHandle, kbReport, sizeof(kbReport));
            logPrint("[MAC NAV] Back button -> Sent Cmd + [");
        } else if (released & 0x08) {
            uint8_t kbReport[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
            sendHidReport(keyboardInputChar, connHandle, kbReport, sizeof(kbReport));
        }

        // Button 5 (Forward): 0x10 -> Cmd + ] (0x30)
        if (pressed & 0x10) {
            uint8_t kbReport[8] = { 0x08, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00 };
            sendHidReport(keyboardInputChar, connHandle, kbReport, sizeof(kbReport));
            logPrint("[MAC NAV] Forward button -> Sent Cmd + ]");
        } else if (released & 0x10) {
            uint8_t kbReport[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
            sendHidReport(keyboardInputChar, connHandle, kbReport, sizeof(kbReport));
        }
    }

    // Mask out Back (0x08) and Forward (0x10) for macOS so it doesn't receive unhandled mouse button clicks
    return buttons & ~(0x08 | 0x10);
}

/**
 * @brief Checks if transition to another PC is permitted.
 * Blocks crossing PC boundaries when:
 * 1. Any mouse button is pressed/held (Left, Right, Middle, Back, Forward) - e.g. dragging, selecting.
 * 2. Any keyboard key or combination (modifiers like Shift/Ctrl/Alt/Cmd or regular keys) is held.
 *
 * @param mouseButtons Bitmask of currently pressed mouse buttons.
 * @return true if switching between PCs is allowed, false if locked to the current PC.
 */
bool isPcSwitchAllowed(uint8_t mouseButtons) {
    // 1. Block if any mouse button is pressed
    if (mouseButtons != 0) {
        return false;
    }
    // 2. Block if any keyboard key or key combination is held
    if (isAnyKeyboardKeyPressed()) {
        return false;
    }
    return true;
}

void updateVirtualCursorAndSend(uint8_t buttons, int16_t dx, int16_t dy, int8_t scroll, int8_t hScroll) {

    if (monitorCount == 0) return;

    MonitorConfig& currentMon = monitors[currentMonitorIndex];
    uint16_t connHandle = getTargetConnHandle(currentMon.mac);
    int16_t sendDx = dx;
    int16_t sendDy = dy;

    // --- Nested: Calibrate virtual cursor to stay within monitor bounds ---
    auto monitorEdgeCalibration = [&](int shift = 0) {
        if (virtualX < currentMon.x) { // Left edge
            virtualX = currentMon.x - shift;
            sendDx -= 127;
        }
        if (virtualY < currentMon.y) { // Top edge
            virtualY = currentMon.y - shift;
            sendDy -= 127;
        }
        if (virtualX >= currentMon.x + currentMon.width) { // Right edge
            virtualX = currentMon.x + currentMon.width - 1 + shift;
            sendDx += 127;
        }
        if (virtualY >= currentMon.y + currentMon.height) { // Bottom edge
            virtualY = currentMon.y + currentMon.height - 1 + shift;
            sendDy += 127;
        }
        static uint32_t lastCalibLog = 0;
        if (millis() - lastCalibLog > 500) {
            lastCalibLog = millis();
            logPrint("[CALIBRATION EDGE] Cursor at (%ld, %ld) on %s", virtualX, virtualY, currentMon.name.c_str());
        }
    };

    if (currentMon.scale != 100) {
        float scaleFactor = currentMon.scale / 100.0f;

        float rawEffX = ((float)sendDx * scaleFactor) + effectiveSubpixelX;
        float rawEffY = ((float)sendDy * scaleFactor) + effectiveSubpixelY;

        long effectiveDx = (long)truncf(rawEffX);
        long effectiveDy = (long)truncf(rawEffY);

        effectiveSubpixelX = rawEffX - (float)effectiveDx;
        effectiveSubpixelY = rawEffY - (float)effectiveDy;
        
        virtualX += effectiveDx;
        virtualY += effectiveDy;
    } else {
        virtualX += sendDx;
        virtualY += sendDy;
    }

    // Find which monitor we are currently in
    int newMonitorIndex = -1;
    for (int i = 0; i < monitorCount; i++) {
        if (virtualX >= monitors[i].x && virtualX < monitors[i].x + monitors[i].width &&
            virtualY >= monitors[i].y && virtualY < monitors[i].y + monitors[i].height) {
            newMonitorIndex = i;
            break;
        }
    }

    if (newMonitorIndex == -1) {
        monitorEdgeCalibration();
        resetSubpixelAccumulators();
    } else if (newMonitorIndex != currentMonitorIndex) {
        if (monitors[newMonitorIndex].mac.equals(currentMon.mac)) {
            currentMonitorIndex = newMonitorIndex;
            logPrint("[MONITOR SWITCH] Cursor at (%ld, %ld) crossed to Monitor #%d (%s)",
                virtualX, virtualY, monitors[newMonitorIndex].id, monitors[newMonitorIndex].name.c_str());
        } else {
                uint16_t targetConn = getTargetConnHandle(monitors[newMonitorIndex].mac);
                if (targetConn == BLE_HS_CONN_HANDLE_NONE || !isPcSwitchAllowed(buttons)) {
                    monitorEdgeCalibration();
                } else {
                    monitorEdgeCalibration(1);
                    // Send safe key release to old PC so no keys remain stuck
                    if (keyboardInputChar) {
                        uint8_t keyRelease[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
                        sendHidReport(keyboardInputChar, connHandle, keyRelease, sizeof(keyRelease));
                    }
                    currentMonitorIndex = newMonitorIndex;
                    logPrint("[PC SWITCH] Cursor saved at (%ld, %ld)", virtualX, virtualY);
                    updateKvmPowerAndRateProfiles(monitors[newMonitorIndex].mac, true);
                    syncPhysicalKeyboardLedsForPc(monitors[newMonitorIndex].mac);
                    sendAbsoluteCoordinates(targetConn, newMonitorIndex, virtualX, virtualY, "PC SWITCH");
                }
        }
        resetSubpixelAccumulators();
    }

    uint8_t sendButtons = buttons;
    if (currentMon.os == OS_MAC) {
        sendButtons = handleMacSideMouseButtons(buttons, connHandle);
    }

    sendRelative12Bit(connHandle, sendDx, sendDy, sendButtons, scroll, hScroll);
}




// Callback when HID data is received from the mouse
void notifyCallback(NimBLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    if (!pData || length < 6) return;
    // Logitech MX Master 3S standard HID mouse buttons (Bits 0..4: Left, Right, Middle, Back, Forward)
    uint8_t buttons = pData[0] & 0x1F;
    // 12-bit X extraction
    int16_t x = pData[2] | ((pData[3] & 0x0F) << 8);
    if (x & 0x800) x |= 0xF000; // Sign extend to 16-bit
    // 12-bit Y extraction
    int16_t y = (pData[3] >> 4) | (pData[4] << 4);
    if (y & 0x800) y |= 0xF000; // Sign extend to 16-bit
    int8_t scroll = (int8_t)pData[5];
    int8_t hScroll = (length > 6) ? (int8_t)pData[6] : 0;
    // logPrint("[DECODE] Raw: %02X %02X %02X %02X %02X %02X %02X -> Btn: 0x%02X, dX: %d, dY: %d, VS: %d, HS: %d | Pos: (%ld, %ld) Mon #%d (%s)",
    //             pData[0], pData[1], pData[2], pData[3], pData[4], pData[5], (length > 6 ? pData[6] : 0),
    //             buttons, x, y, scroll, hScroll, virtualX, virtualY,
    //             monitors[currentMonitorIndex].id, monitors[currentMonitorIndex].name.c_str());
    updateVirtualCursorAndSend(buttons, x, y, scroll, hScroll);
}
