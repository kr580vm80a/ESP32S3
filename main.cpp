#include <Arduino.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <NimBLEDevice.h>
#include <NimBLEHIDDevice.h>
#include <esp_mac.h>

#if CONFIG_IDF_TARGET_ESP32S3
#include <HWCDC.h>
#endif

#define BLE_DEVICE_NAME "ESP32 KVM Mouse"

Preferences preferences;

// NVS Flash Storage Constants
const char* NVS_NAMESPACE = "kvm_config";
const char* NVS_KEY_LAYOUT = "layout";

// Structure to store monitor configuration
struct MonitorConfig {
    int id = 1;
    String name;
    int x;
    int y;
    int width;
    int height;
    String mac;
    int scale = 100;
    bool isPrimary = false;
    int lastX = 0;
    int lastY = 0;
};

#define MAX_MONITORS 10
MonitorConfig monitors[MAX_MONITORS];
int monitorCount = 0;

// --- BLE Peripheral (Server) Variables ---
NimBLEServer* pServer = nullptr;
NimBLEHIDDevice* hidDevice = nullptr;
NimBLECharacteristic* inputChar = nullptr;
NimBLECharacteristic* absInputChar = nullptr;

// Active KVM Connections (Mac addresses of connected PCs)
struct KVMClient {
  uint16_t conn_id;
  String mac;
  String name;
  bool active;
};
#define MAX_KVM_CLIENTS 2
KVMClient kvmClients[MAX_KVM_CLIENTS];
static TaskHandle_t bootCalibTaskHandle = NULL;
uint16_t getTargetConnHandle(const String& targetMac);

// Virtual Cursor Position & State
long virtualX = 0;
long virtualY = 0;
int currentMonitorIndex = 0;

static float subpixelX = 0.0f;
static float subpixelY = 0.0f;
static float effectiveSubpixelX = 0.0f;
static float effectiveSubpixelY = 0.0f;

void resetSubpixelAccumulators() {
    subpixelX = 0.0f;
    subpixelY = 0.0f;
    effectiveSubpixelX = 0.0f;
    effectiveSubpixelY = 0.0f;
}

const uint8_t hidReportMap[] = {
    // --- REPORT ID 1: Standard Relative Mouse (for natural physical movement) ---
    0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
    0x09, 0x02,        // Usage (Mouse)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x01,        //   Report ID (1)
    0x09, 0x01,        //   Usage (Pointer)
    0xA1, 0x00,        //   Collection (Physical)
    0x05, 0x09,        //     Usage Page (Button)
    0x19, 0x01,        //     Usage Minimum (0x01)
    0x29, 0x05,        //     Usage Maximum (0x05)
    0x15, 0x00,        //     Logical Minimum (0)
    0x25, 0x01,        //     Logical Maximum (1)
    0x95, 0x05,        //     Report Count (5)
    0x75, 0x01,        //     Report Size (1)
    0x81, 0x02,        //     Input (Data,Var,Abs)
    0x95, 0x01,        //     Report Count (1)
    0x75, 0x03,        //     Report Size (3)
    0x81, 0x03,        //     Input (Const,Var,Abs)
    0x05, 0x01,        //     Usage Page (Generic Desktop Ctrls)
    0x09, 0x30,        //     Usage (X)
    0x09, 0x31,        //     Usage (Y)
    0x09, 0x38,        //     Usage (Wheel)
    0x15, 0x81,        //     Logical Minimum (-127)
    0x25, 0x7F,        //     Logical Maximum (127)
    0x75, 0x08,        //     Report Size (8)
    0x95, 0x03,        //     Report Count (3)
    0x81, 0x06,        //     Input (Data,Var,Rel)
    0x05, 0x0C,        //     Usage Page (Consumer)
    0x0A, 0x38, 0x02,  //     Usage (AC Pan)
    0x15, 0x81,        //     Logical Minimum (-127)
    0x25, 0x7F,        //     Logical Maximum (127)
    0x75, 0x08,        //     Report Size (8)
    0x95, 0x01,        //     Report Count (1)
    0x81, 0x06,        //     Input (Data,Var,Rel)
    0xC0,              //   End Collection
    0xC0,              // End Collection

    // --- REPORT ID 2: Absolute Mouse / Pointer (for instant 0ms cross-PC transitions) ---
    0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
    0x09, 0x02,        // Usage (Mouse)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x02,        //   Report ID (2)
    0x09, 0x01,        //   Usage (Pointer)
    0xA1, 0x00,        //   Collection (Physical)
    0x05, 0x09,        //     Usage Page (Button)
    0x19, 0x01,        //     Usage Minimum (0x01)
    0x29, 0x05,        //     Usage Maximum (0x05)
    0x15, 0x00,        //     Logical Minimum (0)
    0x25, 0x01,        //     Logical Maximum (1)
    0x95, 0x05,        //     Report Count (5)
    0x75, 0x01,        //     Report Size (1)
    0x81, 0x02,        //     Input (Data,Var,Abs)
    0x95, 0x01,        //     Report Count (1)
    0x75, 0x03,        //     Report Size (3)
    0x81, 0x03,        //     Input (Const,Var,Abs)
    0x05, 0x01,        //     Usage Page (Generic Desktop Ctrls)
    0x09, 0x30,        //     Usage (X)
    0x09, 0x31,        //     Usage (Y)
    0x16, 0x00, 0x00,  //     Logical Minimum (0)
    0x26, 0xFF, 0x7F,  //     Logical Maximum (32767)
    0x36, 0x00, 0x00,  //     Physical Minimum (0)
    0x46, 0xFF, 0x7F,  //     Physical Maximum (32767)
    0x75, 0x10,        //     Report Size (16 bits = 2 bytes per axis)
    0x95, 0x02,        //     Report Count (2 = X, Y)
    0x81, 0x02,        //     Input (Data,Var,Abs)
    0x05, 0x01,        //     Usage Page (Generic Desktop Ctrls)
    0x09, 0x38,        //     Usage (Wheel)
    0x15, 0x81,        //     Logical Minimum (-127)
    0x25, 0x7F,        //     Logical Maximum (127)
    0x75, 0x08,        //     Report Size (8)
    0x95, 0x01,        //     Report Count (1)
    0x81, 0x06,        //     Input (Data,Var,Rel)
    0x05, 0x0C,        //     Usage Page (Consumer)
    0x0A, 0x38, 0x02,  //     Usage (AC Pan)
    0x15, 0x81,        //     Logical Minimum (-127)
    0x25, 0x7F,        //     Logical Maximum (127)
    0x75, 0x08,        //     Report Size (8)
    0x95, 0x01,        //     Report Count (1)
    0x81, 0x06,        //     Input (Data,Var,Rel)
    0xC0,              //   End Collection
    0xC0               // End Collection
};

void logPrint(const char* format, ...) {
    unsigned long ms = millis();
    unsigned long seconds = ms / 1000;
    unsigned long millisec = ms % 1000;
    unsigned long minutes = (seconds / 60) % 60;
    unsigned long hours = (seconds / 3600) % 24;
    
    char timeStr[24];
    snprintf(timeStr, sizeof(timeStr), "[%02lu:%02lu:%02lu.%03lu] ", hours, minutes, seconds % 60, millisec);
    
    char buffer[384];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    Serial.print(timeStr);
    Serial.println(buffer);
#if CONFIG_IDF_TARGET_ESP32S3
    USBSerial.print(timeStr);
    USBSerial.println(buffer);
#endif
}

static String targetMouseMac = "";
static String targetMouseName = "";
static bool isScanningForMice = false;
static bool connected = false;
static bool isConnectingToMouse = false;
static NimBLEClient* pClient = nullptr;
static NimBLEAdvertisedDevice* advDevice = nullptr;
static bool doConnect = false;

static NimBLEUUID hidServiceUUID("1812");
static NimBLEUUID reportCharUUID("2a4d");

void calibrateFirstConnectedPcToCenter(String targetMac);

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) {
        String peerMac = NimBLEAddress(desc->peer_ota_addr).toString().c_str();
        peerMac.toLowerCase();
        peerMac.trim();
        logPrint("[BLE Server] PC Connected! MAC: %s (conn_handle: %d | itvl: %d | latency: %d | timeout: %d)",
                  peerMac.c_str(), desc->conn_handle, desc->conn_itvl, desc->conn_latency, desc->supervision_timeout);
        
        // Save connection
        bool updated = false;
        for (int i = 0; i < MAX_KVM_CLIENTS; i++) {
            if (kvmClients[i].mac.equals(peerMac)) {
                kvmClients[i].conn_id = desc->conn_handle;
                kvmClients[i].active = true;
                updated = true;
                break;
            }
        }
        if (!updated) {
            for (int i = 0; i < MAX_KVM_CLIENTS; i++) {
                if (!kvmClients[i].active) {
                    kvmClients[i].conn_id = desc->conn_handle;
                    kvmClients[i].mac = peerMac;
                    kvmClients[i].name = ""; // Always reset name to prevent leaking stale name from previous device
                    kvmClients[i].active = true;
                    break;
                }
            }
        }
        // Explicitly request security/bonding if peer is not encrypted yet
        if (!desc->sec_state.encrypted) {
            NimBLEDevice::startSecurity(desc->conn_handle);
        }

        // Count active connections
        int activeCount = 0;
        for (int i = 0; i < MAX_KVM_CLIENTS; i++) {
            if (kvmClients[i].active) activeCount++;
        }

        // If this is the FIRST connected PC, assign control & calibrate cursor to center of primary screen!
        if (activeCount == 1) {
            if (bootCalibTaskHandle != NULL) {
                vTaskDelete(bootCalibTaskHandle);
                bootCalibTaskHandle = NULL;
            }
            String firstMac = peerMac;
            xTaskCreate([](void* param) {
                String* pMac = (String*)param;
                vTaskDelay(pdMS_TO_TICKS(600));
                calibrateFirstConnectedPcToCenter(*pMac);
                delete pMac;
                bootCalibTaskHandle = NULL;
                vTaskDelete(NULL);
            }, "bootCalibTask", 3072, new String(firstMac), 1, &bootCalibTaskHandle);
        }

        // Resume advertising so 2nd PC can discover and connect
        if (activeCount < MAX_KVM_CLIENTS) {
            xTaskCreate([](void* param) {
                vTaskDelay(pdMS_TO_TICKS(1500));
                if (NimBLEDevice::getAdvertising() && !NimBLEDevice::getAdvertising()->isAdvertising()) {
                    logPrint("[BLE Server] Resuming advertising for additional PC...");
                    NimBLEDevice::getAdvertising()->start();
                }
                vTaskDelete(NULL);
            }, "bgAdvTask", 4096, NULL, 1, NULL);
        }
    }

    void onDisconnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) {
        String peerMac = NimBLEAddress(desc->peer_ota_addr).toString().c_str();
        peerMac.toLowerCase();
        peerMac.trim();
        logPrint("[BLE Server] PC Disconnected! MAC: %s (conn_handle: %d)", peerMac.c_str(), desc->conn_handle);

        if (bootCalibTaskHandle != NULL) {
            vTaskDelete(bootCalibTaskHandle);
            bootCalibTaskHandle = NULL;
        }
        
        for (int i = 0; i < MAX_KVM_CLIENTS; i++) {
            if (kvmClients[i].conn_id == desc->conn_handle || (kvmClients[i].mac.length() > 0 && kvmClients[i].mac.equals(peerMac))) {
                kvmClients[i].active = false;
                break;
            }
        }

        // If the disconnected PC was the current active PC, failover to another connected PC!
        if (monitorCount > 0 && monitors[currentMonitorIndex].mac.equals(peerMac)) {
            String fallbackMac = "";
            for (int i = 0; i < MAX_KVM_CLIENTS; i++) {
                if (kvmClients[i].active && kvmClients[i].mac.length() > 0 && !kvmClients[i].mac.equals(peerMac)) {
                    fallbackMac = kvmClients[i].mac;
                    break;
                }
            }

            if (fallbackMac.length() > 0) {
                int targetMonIdx = -1;
                for (int i = 0; i < monitorCount; i++) {
                    if (monitors[i].isPrimary && monitors[i].mac.equals(fallbackMac)) {
                        targetMonIdx = i;
                        break;
                    }
                }

                if (targetMonIdx != -1) {
                    currentMonitorIndex = targetMonIdx;
                    MonitorConfig& mon = monitors[targetMonIdx];
                    if (mon.lastX >= mon.x && mon.lastX < mon.x + mon.width &&
                        mon.lastY >= mon.y && mon.lastY < mon.y + mon.height) {
                        virtualX = mon.lastX;
                        virtualY = mon.lastY;
                    } else {
                        virtualX = mon.x + (mon.width / 2);
                        virtualY = mon.y + (mon.height / 2);
                    }
                    mon.lastX = virtualX;
                    mon.lastY = virtualY;
                    resetSubpixelAccumulators();

                    logPrint("[FAILOVER] Current PC %s disconnected! Switched control to active PC %s (Mon #%d: %s at %ld, %ld)",
                             peerMac.c_str(), fallbackMac.c_str(), mon.id, mon.name.c_str(), virtualX, virtualY);
                }
            } else {
                logPrint("[FAILOVER] Current PC %s disconnected and no other active PC available.", peerMac.c_str());
            }
        }

        // Debounce advertising restart to prevent FreeRTOS task flooding during rapid disconnect loops
        static uint32_t lastDisconnectAdvTime = 0;
        uint32_t now = millis();
        if (now - lastDisconnectAdvTime > 1500) {
            lastDisconnectAdvTime = now;
            xTaskCreate([](void* param) {
                vTaskDelay(pdMS_TO_TICKS(1000));
                if (NimBLEDevice::getAdvertising() && !NimBLEDevice::getAdvertising()->isAdvertising()) {
                    logPrint("[BLE Server] Resuming advertising after PC disconnect...");
                    NimBLEDevice::getAdvertising()->start();
                }
                vTaskDelete(NULL);
            }, "bgAdvTask", 4096, NULL, 1, NULL);
        }
    }

    void onAuthenticationComplete(ble_gap_conn_desc* desc) {
        String peerMac = NimBLEAddress(desc->peer_ota_addr).toString().c_str();
        peerMac.toLowerCase();
        peerMac.trim();
        logPrint("[BLE Server] Auth Complete for %s | Encrypted: %d | Bonded: %d | KeySize: %d",
                  peerMac.c_str(), desc->sec_state.encrypted, desc->sec_state.bonded, desc->sec_state.key_size);
        if (!desc->sec_state.bonded) {
            logPrint("[BLE Server] Bonding incomplete (Bonded: 0) for %s! Clearing stale bond key to allow fresh pairing...", peerMac.c_str());
            NimBLEDevice::deleteBond(desc->peer_ota_addr);
        }
    }

    uint32_t onPassKeyRequest() {
        logPrint("[BLE Server] PassKey requested by client! Enter PIN: 654321");
        return 654321;
    }

    bool onConfirmPIN(uint32_t pin) {
        logPrint("[BLE Server] PIN confirmation requested: %06d", pin);
        return pin == 654321;
    }
};

String getMonDisplayName(int idx) {
    return monitors[idx].name;
}

uint16_t getTargetConnHandle(const String& targetMac) {
    // 1. Direct MAC address match
    for (int i = 0; i < MAX_KVM_CLIENTS; i++) {
        if (kvmClients[i].active && kvmClients[i].mac.equals(targetMac)) {
            return kvmClients[i].conn_id;
        }
    }
    return BLE_HS_CONN_HANDLE_NONE;
}

// Send HID report to target connection handle or broadcast notify
void sendHidReport(NimBLECharacteristic* pChar, uint16_t connHandle, const uint8_t* report, size_t length = 5) {
    if (!pChar || !report || length == 0) return;
    if (connHandle != BLE_HS_CONN_HANDLE_NONE) {
        os_mbuf *om = ble_hs_mbuf_from_flat(report, length);
        if (om != NULL) {
            int rc = ble_gatts_notify_custom(connHandle, pChar->getHandle(), om);
            if (rc != 0) os_mbuf_free_chain(om);
        }
    } else {
        pChar->setValue(report, length);
        pChar->notify();
    }
}

// --- Absolute HID Positioning Function ---
void sendAbsoluteCoordinates(uint16_t connHandle, int monIndex, long targetGlobalX, long targetGlobalY, const char* contextLabel = "ABS POINTER") {
    if (monitorCount == 0) return;

    MonitorConfig& targetMon = monitors[monIndex];
    String targetMac = targetMon.mac;

    long monW = (targetMon.width > 0) ? targetMon.width : 1920;
    long monH = (targetMon.height > 0) ? targetMon.height : 1080;

    long relX = constrain(targetGlobalX - targetMon.x, 0, monW);
    long relY = constrain(targetGlobalY - targetMon.y, 0, monH);

    uint16_t absX = (uint16_t)round(((float)relX / (float)monW) * 32767.0f);
    uint16_t absY = (uint16_t)round(((float)relY / (float)monH) * 32767.0f);

    uint8_t absReport[7] = {
        0x00,                               // Buttons
        (uint8_t)(absX & 0xFF),             // X Low
        (uint8_t)((absX >> 8) & 0xFF),      // X High
        (uint8_t)(absY & 0xFF),             // Y Low
        (uint8_t)((absY >> 8) & 0xFF),      // Y High
        0x00,                               // Wheel
        0x00                                // AC Pan
    };

    sendHidReport(absInputChar, connHandle, absReport, sizeof(absReport));
    logPrint("[%s] Instantly positioned PC %s at (%ld, %ld) [Rel: %ld, %ld -> Norm: %u, %u] on Mon #%d (%s)",
             contextLabel, targetMac.c_str(), virtualX, virtualY, relX, relY, absX, absY, currentMonitorIndex + 1, targetMon.name.c_str());
}

// --- Boot Center Calibration Wrapper ---
void calibrateFirstConnectedPcToCenter(String targetMac) {
    if (monitorCount == 0) return;
    uint16_t connHandle = getTargetConnHandle(targetMac);
    if (connHandle == BLE_HS_CONN_HANDLE_NONE) return;

    int currentMonitorIndex = -1;
    for (int i = 0; i < monitorCount; i++) {
        if (monitors[i].mac.equals(targetMac) && monitors[i].isPrimary) {
            currentMonitorIndex = i;
            break;
        }
    }
    if (currentMonitorIndex == -1) currentMonitorIndex = 0;
    MonitorConfig& mon = monitors[currentMonitorIndex];
    virtualX = mon.x + (mon.width / 2);
    virtualY = mon.y + (mon.height / 2);
    sendAbsoluteCoordinates(connHandle, currentMonitorIndex, virtualX, virtualY, "BOOT POSITION");
}

// --- BLE Host (Central) Functions ---

void updateVirtualCursorAndSend(uint8_t buttons, int16_t dx, int16_t dy, int8_t scroll, int8_t hScroll) {
    if (monitorCount == 0) return;

    MonitorConfig& currentMon = monitors[currentMonitorIndex];
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
            virtualX = currentMon.x + currentMon.width -1 + shift;
            sendDx += 127;
        }
        if (virtualY >= currentMon.y + currentMon.height) { // Bottom edge
            virtualY = currentMon.y + currentMon.height -1 + shift;
            sendDy += 127;
        }
        logPrint("[CALIBRATION EDGE] Cursor at (%ld, %ld) on %s", virtualX, virtualY, currentMon.name.c_str());
    };

    if (currentMon.scale != 100) {

        long effectiveDx = dx;
        long effectiveDy = dy;

        float scaleFactor = currentMon.scale / 100.0f;

        float rawSendX = (dx / scaleFactor) + subpixelX;
        float rawSendY = (dy / scaleFactor) + subpixelY;

        sendDx = (int16_t)truncf(rawSendX);
        sendDy = (int16_t)truncf(rawSendY);

        subpixelX = rawSendX - (float)sendDx;
        subpixelY = rawSendY - (float)sendDy;

        float rawEffX = ((float)sendDx * scaleFactor) + effectiveSubpixelX;
        float rawEffY = ((float)sendDy * scaleFactor) + effectiveSubpixelY;

        effectiveDx = (long)truncf(rawEffX);
        effectiveDy = (long)truncf(rawEffY);

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
            if (targetConn == BLE_HS_CONN_HANDLE_NONE) {
                monitorEdgeCalibration();
            } else {
                monitorEdgeCalibration(1);
                monitors[currentMonitorIndex].lastX = virtualX;
                monitors[currentMonitorIndex].lastY = virtualY;
                currentMonitorIndex = newMonitorIndex;
                logPrint("[PC SWITCH] Cursor saved at (%ld, %ld)", virtualX, virtualY);
                sendAbsoluteCoordinates(targetConn, newMonitorIndex, virtualX, virtualY, "KVM SWITCH");
            }
        }
        resetSubpixelAccumulators();
    }

    // Send Standard HID Report (5 bytes: Buttons, dX, dY, VScroll, HScroll)
    uint8_t report[5] = { 
        buttons, 
        (uint8_t)constrain(sendDx, -127, 127),
        (uint8_t)constrain(sendDy, -127, 127),
        (uint8_t)constrain(scroll, -127, 127),
        (uint8_t)constrain(hScroll, -127, 127)
    };

    uint16_t connHandle = getTargetConnHandle(currentMon.mac);
    sendHidReport(inputChar, connHandle, report, sizeof(report));
}

// Callback when HID data is received from the mouse
void notifyCallback(NimBLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    if (!pData || length < 6) return;
    // Logitech MX Master 3S HID report button mask can be in byte 0 or byte 1
    uint8_t buttons = pData[0] | pData[1];
    // 12-bit X extraction
    int16_t x = pData[2] | ((pData[3] & 0x0F) << 8);
    if (x & 0x800) x |= 0xF000; // Sign extend to 16-bit
    // 12-bit Y extraction
    int16_t y = (pData[3] >> 4) | (pData[4] << 4);
    if (y & 0x800) y |= 0xF000; // Sign extend to 16-bit
    int8_t scroll = (int8_t)pData[5];
    int8_t hScroll = (length > 6) ? (int8_t)pData[6] : 0;
    logPrint("[DECODE] Raw: %02X %02X %02X %02X %02X %02X %02X -> Btn: 0x%02X, dX: %d, dY: %d, VS: %d, HS: %d | Pos: (%ld, %ld) Mon #%d (%s)",
                pData[0], pData[1], pData[2], pData[3], pData[4], pData[5], (length > 6 ? pData[6] : 0),
                buttons, x, y, scroll, hScroll, virtualX, virtualY,
                monitors[currentMonitorIndex].id, monitors[currentMonitorIndex].name.c_str());
    updateVirtualCursorAndSend(buttons, x, y, scroll, hScroll);
}

bool connectToServer();
void sendConfigResponse(const String& response);
void saveMouseToNvsLayout(String mac, String name);
String loadLayoutJsonFromNVS();
static JsonDocument scannedMiceDoc;
static TaskHandle_t reconnTaskHandle = NULL;

class ScanCallbacks : public NimBLEAdvertisedDeviceCallbacks {
    void onResult(NimBLEAdvertisedDevice* advertisedDevice) {
        String devMac = advertisedDevice->getAddress().toString().c_str();
        devMac.toLowerCase();
        devMac.trim();

        String devName = advertisedDevice->getName().c_str();
        int rssi = advertisedDevice->getRSSI();

        if (isScanningForMice) {
            JsonArray arr = scannedMiceDoc.as<JsonArray>();
            bool exists = false;
            for (JsonObject m : arr) {
                if (m["mac"].as<String>() == devMac) {
                    m["rssi"] = rssi;
                    exists = true;
                    break;
                }
            }
            if (!exists) {
                JsonObject obj = arr.add<JsonObject>();
                obj["mac"] = devMac;
                obj["name"] = devName.length() > 0 ? devName : "Bluetooth Device";
                obj["rssi"] = rssi;
            }
            return;
        }

        String macPrefix = targetMouseMac.length() >= 14 ? targetMouseMac.substring(0, 14) : "";

        if (targetMouseMac.length() > 0 && (devMac == targetMouseMac || (macPrefix.length() > 0 && devMac.startsWith(macPrefix)) || devName.equalsIgnoreCase("MX Master 3S") || devName.indexOf("MX Master") != -1)) {
            logPrint("[BLE Scan] TARGET LOCK MATCH! Connecting to %s (%s)", devName.c_str(), devMac.c_str());
            NimBLEDevice::getScan()->stop();
            advDevice = new NimBLEAdvertisedDevice(*advertisedDevice);
            doConnect = true;
        }
    }
};

void startMouseReconnectTask() {
    if (targetMouseMac.length() == 0 || isScanningForMice || connected) return;
    if (NimBLEDevice::getScan()->isScanning()) return;

    logPrint("[BLE Host] Starting continuous background scan for mouse (%s)...", targetMouseMac.c_str());
    NimBLEScan* pScan = NimBLEDevice::getScan();
    if (pScan) {
        pScan->setAdvertisedDeviceCallbacks(new ScanCallbacks(), false);
        pScan->setActiveScan(true);
        pScan->setInterval(160);
        pScan->setWindow(160);
        pScan->setDuplicateFilter(false);
        pScan->start(0, nullptr, false); // Non-blocking continuous background scan
    }
}

// Callback for BLE Connection Status
class ClientCallbacks : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient* pClient) {
        logPrint("[BLE Host] Connected to mouse!");
        connected = true;
    }
    void onDisconnect(NimBLEClient* pClient) {
        logPrint("[BLE Host] Disconnected from mouse!");
        connected = false;
        xTaskCreate([](void* param) {
            vTaskDelay(pdMS_TO_TICKS(500));
            startMouseReconnectTask();
            vTaskDelete(NULL);
        }, "reconnTask", 3072, NULL, 1, NULL);
    }
};

bool connectToServer() {
    if (isScanningForMice) return false;
    if (targetMouseMac.length() == 0 && !advDevice) return false;

    if (!pClient) {
        pClient = NimBLEDevice::createClient();
        pClient->setClientCallbacks(new ClientCallbacks());
    }

    if (pClient->isConnected()) {
        connected = true;
        return true;
    }

    isConnectingToMouse = true;

    bool wasAdvertising = false;
    if (NimBLEDevice::getAdvertising() && NimBLEDevice::getAdvertising()->isAdvertising()) {
        wasAdvertising = true;
        NimBLEDevice::getAdvertising()->stop();
    }

    if (!advDevice && targetMouseMac.length() > 0) {
        logPrint("[BLE Host] Performing targeted fast probe scan for mouse (%s)...", targetMouseMac.c_str());
        NimBLEScan* pScan = NimBLEDevice::getScan();
        if (pScan) {
            pScan->setActiveScan(true);
            pScan->setInterval(160);
            pScan->setWindow(160);
            NimBLEScanResults results = pScan->start(2, false);
            String macPrefix = targetMouseMac.length() >= 14 ? targetMouseMac.substring(0, 14) : "";
            macPrefix.toLowerCase();
            for (int i = 0; i < results.getCount(); i++) {
                NimBLEAdvertisedDevice dev = results.getDevice(i);
                String devMac = dev.getAddress().toString().c_str();
                devMac.toLowerCase();
                String devName = dev.getName().c_str();
                if (devMac == targetMouseMac || (macPrefix.length() > 0 && devMac.startsWith(macPrefix)) || devName.equalsIgnoreCase("MX Master 3S") || devName.indexOf("MX Master") != -1) {
                    if (devMac != targetMouseMac) {
                        logPrint("[BLE Host] AUTO-RESOLVED rotated mouse MAC: %s (was %s)!", devMac.c_str(), targetMouseMac.c_str());
                        saveMouseToNvsLayout(devMac, devName.length() > 0 ? devName : targetMouseName);
                        sendConfigResponse("OK_BIND_MOUSE " + targetMouseMac);
                    }
                    advDevice = new NimBLEAdvertisedDevice(dev);
                    logPrint("[BLE Host] Fast probe scan found mouse: %s (name: %s)!", devMac.c_str(), devName.c_str());
                    break;
                }
            }
            pScan->clearResults();
        }
    }

    bool connRes = false;
    if (advDevice) {
        logPrint("[BLE Host] Connecting directly to advertised device: %s...", advDevice->getAddress().toString().c_str());
        connRes = pClient->connect(advDevice);
    } else {
        logPrint("[BLE Host] Mouse not advertising yet (retrying in background loop)...");
        connRes = false;
    }

    isConnectingToMouse = false;

    int activeCount = 0;
    for (int i = 0; i < MAX_KVM_CLIENTS; i++) {
        if (kvmClients[i].active) activeCount++;
    }
    if (activeCount < MAX_KVM_CLIENTS && NimBLEDevice::getAdvertising() && !NimBLEDevice::getAdvertising()->isAdvertising()) {
        logPrint("[BLE Server] Resuming advertising for additional PC...");
        NimBLEDevice::getAdvertising()->start();
    }

    if (!connRes) {
        logPrint("[BLE Host] Connection attempt failed (mouse not advertising or out of range).");
        return false;
    }

    logPrint("[BLE Host] Connected! Securing connection (Pairing)...");
    if (!pClient->secureConnection()) {
        logPrint("[BLE Host] Initial secureConnection failed. Retrying in 100ms...");
        delay(100);
        if (!pClient->secureConnection()) {
            logPrint("[BLE Host] Secure connection retry failed. Proceeding with service discovery...");
        } else {
            logPrint("[BLE Host] Connection secured on retry!");
        }
    } else {
        logPrint("[BLE Host] Connection secured!");
    }

    NimBLERemoteService* pService = pClient->getService(hidServiceUUID);
    if (pService != nullptr) {
        std::vector<NimBLERemoteCharacteristic*>* pChars = pService->getCharacteristics(true);
        for (auto &pChar : *pChars) {
            if (pChar->getUUID() == reportCharUUID) {
                if(pChar->canNotify()) {
                    pChar->subscribe(true, notifyCallback);
                    logPrint("[BLE Host] Subscribed to HID report!");
                }
            }
        }
    } else {
        pClient->disconnect();
        isConnectingToMouse = false;
        return false;
    }
    connected = true;
    isConnectingToMouse = false;
    return true;
}

#define CONFIG_SERVICE_UUID "12345678-1234-1234-1234-1234567890ab"
#define CONFIG_RX_UUID      "12345678-1234-1234-1234-1234567890ac"
#define CONFIG_TX_UUID      "12345678-1234-1234-1234-1234567890ad"

NimBLECharacteristic* configTxChar = nullptr;
NimBLECharacteristic* configRxChar = nullptr;

void sendConfigResponse(const String& response) {
  String fullResp = response;
  if (!fullResp.endsWith("\n")) fullResp += "\n";
  Serial.print(fullResp);
#if CONFIG_IDF_TARGET_ESP32S3
  USBSerial.print(fullResp);
#endif
  if (configTxChar) {
    size_t len = fullResp.length();
    uint16_t mtu = NimBLEDevice::getMTU();
    size_t chunkSize = (mtu > 28) ? (mtu - 5) : 240;
    if (chunkSize > 480) chunkSize = 480;

    logPrint("[BLE TX] Sending %d bytes in %d-byte MTU chunks...", (int)len, (int)chunkSize);

    for (size_t i = 0; i < len; i += chunkSize) {
      String chunk = fullResp.substring(i, min(i + chunkSize, len));
      configTxChar->setValue((const uint8_t*)chunk.c_str(), chunk.length());
      configTxChar->notify();
      delay(30);
    }
  }
}


String loadLayoutJsonFromNVS() {
  preferences.begin(NVS_NAMESPACE, true);
  String json = "{}";
  size_t len = preferences.getBytesLength(NVS_KEY_LAYOUT);
  if (len > 0) {
    char* buf = (char*)malloc(len + 1);
    if (buf) {
      preferences.getBytes(NVS_KEY_LAYOUT, buf, len);
      buf[len] = '\0';
      json = String(buf);
      free(buf);
    }
  }
  preferences.end();

  targetMouseMac = "";
  targetMouseName = "";
  if (json.length() > 2) {
    JsonDocument doc;
    if (!deserializeJson(doc, json) && doc.is<JsonObject>()) {
      targetMouseMac = doc["mouseMac"] | "";
      targetMouseName = doc["mouseName"] | "";
    }
  }
  return json;
}

void initDefaultConfigDoc(JsonDocument& doc) {
  doc.clear();
  doc["activeLayoutId"] = 1;
  doc["totalLayouts"] = 1;
  JsonArray layoutsArr = doc["layouts"].to<JsonArray>();
  JsonObject layout1 = layoutsArr.add<JsonObject>();
  layout1["id"] = 1;
  layout1["name"] = "Default Layout";
  layout1["totalScreens"] = 0;
  layout1["screens"].to<JsonArray>();
  doc["clients"].to<JsonArray>();
}

void saveMouseToNvsLayout(String mac, String name) {
  mac.toLowerCase();
  mac.trim();
  name.trim();
  targetMouseMac = mac;
  targetMouseName = name;

  String json = loadLayoutJsonFromNVS();
  JsonDocument doc;
  if (deserializeJson(doc, json) || !doc.is<JsonObject>()) {
    initDefaultConfigDoc(doc);
  }

  doc["mouseMac"] = targetMouseMac;
  doc["mouseName"] = targetMouseName;

  String unifiedJson;
  serializeJson(doc, unifiedJson);

  preferences.begin(NVS_NAMESPACE, false);
  preferences.remove(NVS_KEY_LAYOUT);
  size_t bytesWritten = preferences.putBytes(NVS_KEY_LAYOUT, unifiedJson.c_str(), unifiedJson.length() + 1);
  preferences.end();

  if (bytesWritten > 0) {
    logPrint("[NVS] Persisted mouse (%s, '%s') in unified JSON layout (%u bytes)!", targetMouseMac.c_str(), targetMouseName.c_str(), bytesWritten);
  } else {
    logPrint("[NVS ERROR] Failed to save mouse to layout (putBytes returned 0)!");
  }
}

void loadConfiguration() {
  String json = loadLayoutJsonFromNVS();

  if (json.length() > 2) {
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, json);
    if (!err && doc.is<JsonObject>()) {
      JsonArray arr;
      if (doc["layouts"].is<JsonArray>() && doc["layouts"].size() > 0) {
        JsonObject activeLayout = doc["layouts"][0].as<JsonObject>();
        int targetId = doc["activeLayoutId"] | 1;
        for (JsonObject l : doc["layouts"].as<JsonArray>()) {
          int lId = l["id"] | 0;
          if (lId == targetId || (l["id"].as<String>() == String(targetId))) {
            activeLayout = l;
            break;
          }
        }
        arr = activeLayout["screens"].as<JsonArray>();
      }

      monitorCount = 0;
      if (arr) {
        for (JsonObject repo : arr) {
          int defId = monitorCount + 1;
          String defName = "Monitor #" + String(defId);
          monitors[monitorCount].id = repo["id"] | defId;
          monitors[monitorCount].name = repo["name"] | defName;
          monitors[monitorCount].x = repo["x"] | 0;
          monitors[monitorCount].y = repo["y"] | 0;
          monitors[monitorCount].width = repo["width"] | 1920;
          monitors[monitorCount].height = repo["height"] | 1080;
          monitors[monitorCount].mac = repo["mac"] | "";
          monitors[monitorCount].scale = repo["scale"] | 100;
          monitors[monitorCount].isPrimary = repo["isPrimary"] | false;
          monitorCount++;
        }
      }
      logPrint("Loaded %d monitors from NVS. Target mouse: %s (%s)", monitorCount, targetMouseMac.c_str(), targetMouseName.c_str());

      if (doc["clients"].is<JsonArray>()) {
        int clientCount = 0;
        for (JsonObject client : doc["clients"].as<JsonArray>()) {
          if (clientCount >= MAX_KVM_CLIENTS) break;
          String mac = client["mac"] | "";
          if (mac.length() > 0) {
            kvmClients[clientCount].mac = mac;
            kvmClients[clientCount].name = client["name"] | "Unknown PC";
            kvmClients[clientCount].active = false;
            clientCount++;
          }
        }
      }
    }
  }
}

void saveConfiguration(const String& jsonString) {
  JsonDocument doc;
  String finalJson = jsonString;

  if (!deserializeJson(doc, jsonString)) {
    String mac = doc["mouseMac"] | "";
    if (mac.length() > 0) {
      mac.toLowerCase();
      mac.trim();
      targetMouseMac = mac;
    } else if (targetMouseMac.length() > 0) {
      doc["mouseMac"] = targetMouseMac;
    }

    String name = doc["mouseName"] | "";
    if (name.length() > 0) {
      name.trim();
      targetMouseName = name;
    } else if (targetMouseName.length() > 0) {
      doc["mouseName"] = targetMouseName;
    }

    serializeJson(doc, finalJson);
    logPrint("[NVS] Persisted targetMouseMac '%s' and targetMouseName '%s' in unified JSON layout", targetMouseMac.c_str(), targetMouseName.c_str());
  }

  preferences.begin(NVS_NAMESPACE, false);
  preferences.remove(NVS_KEY_LAYOUT);
  size_t bytesWritten = preferences.putBytes(NVS_KEY_LAYOUT, finalJson.c_str(), finalJson.length() + 1);
  preferences.end();
  if (bytesWritten > 0) {
    logPrint("[NVS] Configuration saved to NVS successfully (%u bytes)!", bytesWritten);
  } else {
    logPrint("[NVS ERROR] Failed to save configuration (putBytes returned 0)!");
  }
}

static String pendingSaveJson = "";
static bool doSaveConfig = false;

void executePendingSave() {
  if (doSaveConfig && pendingSaveJson.length() > 0) {
    doSaveConfig = false;
    JsonDocument doc;
    if (!deserializeJson(doc, pendingSaveJson)) {
      saveConfiguration(pendingSaveJson);
      loadConfiguration();
      Serial.println("OK_SAVE");
#if CONFIG_IDF_TARGET_ESP32S3
      USBSerial.println("OK_SAVE");
#endif
      if (configTxChar) {
        String resp = "OK_SAVE\n";
        configTxChar->setValue((const uint8_t*)resp.c_str(), resp.length());
        configTxChar->notify();
      }
    }
    pendingSaveJson = "";
  }
}

void processCommand(String input) {
  input.trim();
  if (input.startsWith("SAVE_CONFIG ")) {
    String payload = input.substring(12);
    payload.trim();
    
    int expectedLen = -1;
    String jsonStr = payload;

    int spaceIdx = payload.indexOf(' ');
    if (spaceIdx > 0 && !payload.startsWith("{") && !payload.startsWith("[")) {
      String lenHeader = payload.substring(0, spaceIdx);
      expectedLen = lenHeader.toInt();
      jsonStr = payload.substring(spaceIdx + 1);
      jsonStr.trim();
    }

    if (expectedLen > 0 && (int)jsonStr.length() != expectedLen) {
      logPrint("[SAVE CONFIG ERROR] Content-Length mismatch: received %d, expected %d", (int)jsonStr.length(), expectedLen);
      sendConfigResponse("ERROR_SAVE Content-Length mismatch");
      return;
    }

    if (jsonStr.startsWith("{") || jsonStr.startsWith("[")) {
      pendingSaveJson = jsonStr;
      doSaveConfig = true;
    }
  } else if (input == "GET_CONFIG") {
    String json = loadLayoutJsonFromNVS();

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, json);

    if (err || !doc.is<JsonObject>()) {
      initDefaultConfigDoc(doc);
    }

    // Include bound mouse MAC & Name in the unified JSON payload for backup & restore
    doc["mouseMac"] = targetMouseMac;
    doc["mouseName"] = targetMouseName.length() > 0 ? targetMouseName : (targetMouseMac.length() > 0 ? "BLE Mouse" : "");

    // Preserve stored clients and reset connected status prior to active connection sync
    JsonArray clientsArr;
    if (doc["clients"].is<JsonArray>()) {
      clientsArr = doc["clients"].as<JsonArray>();
      for (JsonObject c : clientsArr) {
        c["connected"] = false;
      }
    } else {
      clientsArr = doc["clients"].to<JsonArray>();
    }

    // Dynamic merge of active connected BLE clients into unified JSON
    for (int i = 0; i < MAX_KVM_CLIENTS; i++) {
      if (kvmClients[i].active && kvmClients[i].mac.length() > 0) {
        String activeMac = kvmClients[i].mac;
        activeMac.toLowerCase();
        activeMac.trim();

        bool exists = false;
        for (JsonObject c : clientsArr) {
          String cMac = c["mac"] | "";
          cMac.toLowerCase();
          cMac.trim();
          if (cMac == activeMac) {
            c["connected"] = true;
            exists = true;
            break;
          }
        }
        if (!exists) {
          JsonObject clientObj = clientsArr.add<JsonObject>();
          clientObj["mac"] = activeMac;
          clientObj["name"] = kvmClients[i].name.length() > 0 ? kvmClients[i].name : "Detected Device";
          clientObj["connected"] = true;
        }
      }
    }

    String unifiedJson;
    serializeJson(doc, unifiedJson);

    sendConfigResponse("CONFIG " + String(unifiedJson.length()) + " " + unifiedJson);
  } else if (input == "SCAN_MICE") {
    scannedMiceDoc.clear();
    scannedMiceDoc.to<JsonArray>();

    isScanningForMice = true;

    // Cancel any background reconnect task and disconnect mouse to free radio
    if (reconnTaskHandle != NULL) {
      logPrint("[BLE Scan] Cancelling background mouseReconnectTask for discovery scan...");
      vTaskDelete(reconnTaskHandle);
      reconnTaskHandle = NULL;
    }
    if (pClient && pClient->isConnected()) {
      logPrint("[BLE Scan] Disconnecting from mouse before discovery scan...");
      pClient->disconnect();
    }
    delay(200);

    NimBLEScan* pScan = NimBLEDevice::getScan();
    if (pScan) {
      if (pScan->isScanning()) {
        pScan->stop();
        delay(100);
      }
      pScan->setAdvertisedDeviceCallbacks(new ScanCallbacks(), true);
      pScan->setActiveScan(true);
      pScan->setInterval(100);
      pScan->setWindow(99);

      logPrint("[BLE Scan] Starting 5-second active discovery scan for mice...");
      pScan->start(5, false);
      pScan->clearResults();
    }
    isScanningForMice = false;
    logPrint("[BLE Scan] Discovery scan complete! Discovered %d BLE devices.", (int)scannedMiceDoc.as<JsonArray>().size());

    String jsonStr;
    serializeJson(scannedMiceDoc, jsonStr);
    sendConfigResponse("MICE " + jsonStr);
  } else if (input.startsWith("BIND_MOUSE ")) {
    String param = input.substring(11);
    param.trim();
    String mac = param;
    String name = "BLE Mouse";
    int spaceIdx = param.indexOf(' ');
    if (spaceIdx != -1) {
      mac = param.substring(0, spaceIdx);
      name = param.substring(spaceIdx + 1);
      name.trim();
    }
    saveMouseToNvsLayout(mac, name);
    sendConfigResponse("OK_BIND_MOUSE " + targetMouseMac);

    if (reconnTaskHandle != NULL) {
      vTaskDelete(reconnTaskHandle);
      reconnTaskHandle = NULL;
    }
    if (pClient && pClient->isConnected()) {
      pClient->disconnect();
    }
    if (targetMouseMac.length() > 0) {
      doConnect = true;
    }
  } else if (input == "UNBIND_MOUSE") {
    saveMouseToNvsLayout("", "");
    if (reconnTaskHandle != NULL) {
      vTaskDelete(reconnTaskHandle);
      reconnTaskHandle = NULL;
    }
    if (pClient && pClient->isConnected()) {
      pClient->disconnect();
    }
    sendConfigResponse("OK_UNBIND_MOUSE");
  } else if (input == "GET_TARGET_MOUSE") {
    sendConfigResponse("TARGET_MOUSE " + targetMouseMac);
  } else if (input == "DUMP_FLASH") {
    String json = loadLayoutJsonFromNVS();
    logPrint("--- [NVS FLASH DUMP] ---");
    logPrint("Flash layout string length: %d bytes", json.length());
    logPrint("Flash target mouse MAC: %s", targetMouseMac.c_str());
    logPrint("%s", json.c_str());
    logPrint("--- [END NVS FLASH DUMP] ---");
  } else if (input == "CLEAR_BONDS" || input == "CLEAR_BLE_BONDS") {
    int count = NimBLEDevice::getNumBonds();
    NimBLEDevice::deleteAllBonds();
    logPrint("[BLE] Deleted %d bonded devices from NVS. Fresh pairing required for all PCs.", count);
    sendConfigResponse("OK_CLEAR_BONDS " + String(count));
  }
}

static String bleRxBuffer = "";
static std::vector<String> bleCmdQueue;

class ConfigRxCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pCharacteristic) {
        std::string rxValue = pCharacteristic->getValue();
        logPrint("[BLE RX DEBUG]: Received %d bytes: '%s'", (int)rxValue.length(), rxValue.c_str());
        if (rxValue.length() > 0) {
            bleRxBuffer += String(rxValue.c_str());
            while (bleRxBuffer.indexOf('\n') != -1) {
              int lineEnd = bleRxBuffer.indexOf('\n');
              String cmd = bleRxBuffer.substring(0, lineEnd);
              bleRxBuffer = bleRxBuffer.substring(lineEnd + 1);
              cmd.trim();
              if (cmd.length() > 0) {
                bleCmdQueue.push_back(cmd);
              }
            }
        }
    }
};

void setup() {
  Serial.setRxBufferSize(16384);
  Serial.begin(115200);
#if CONFIG_IDF_TARGET_ESP32S3
  USBSerial.begin(115200);
#endif
  delay(2000);
  
  logPrint("--- ESP32 KVM Switcher Started ---");
  loadConfiguration();
  
  logPrint("[BLE] Initializing NimBLE...");
  uint8_t customMac[6];
  esp_read_mac(customMac, ESP_MAC_BT);
  customMac[5] += 1; // Increment last byte by 1 to present clean device identity to PCs
  esp_base_mac_addr_set(customMac);
  logPrint("[BLE] Custom Base MAC: %02X:%02X:%02X:%02X:%02X:%02X",
           customMac[0], customMac[1], customMac[2], customMac[3], customMac[4], customMac[5]);

  NimBLEDevice::init(BLE_DEVICE_NAME);
  NimBLEDevice::setMTU(512);
  NimBLEDevice::setSecurityAuth(true, false, true); // Compatible Just Works pairing for macOS/Windows (bonding=true, mitm=false, sc=true)
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT); // Standard HID Mouse IO Capability
  NimBLEDevice::setSecurityInitKey(BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);
  NimBLEDevice::setSecurityRespKey(BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);
  
  int numBonds = NimBLEDevice::getNumBonds();
  logPrint("[BLE NVS BONDS] Saved bonded devices count: %d", numBonds);
  for (int i = 0; i < numBonds; i++) {
      NimBLEAddress bondAddr = NimBLEDevice::getBondedAddress(i);
      logPrint("  -> Bonded Device #%d: MAC %s", i + 1, bondAddr.toString().c_str());
  }
  
  // Setup BLE Server (Peripheral)
  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());
  hidDevice = new NimBLEHIDDevice(pServer);
  inputChar = hidDevice->inputReport(1);    // Report ID 1: Relative Mouse
  absInputChar = hidDevice->inputReport(2); // Report ID 2: Absolute Mouse / Pointer
  
  hidDevice->manufacturer()->setValue("Antigravity Labs");
  hidDevice->pnp(0x02, 0x046d, 0x0000, 0x0110);
  hidDevice->hidInfo(0x00, 0x01);
  
  hidDevice->reportMap((uint8_t*)hidReportMap, sizeof(hidReportMap));
  hidDevice->startServices();
  
  // Setup Custom Config Service ("ESP32 KVM Server")
  NimBLEService* pConfigService = pServer->createService(CONFIG_SERVICE_UUID);
  configTxChar = pConfigService->createCharacteristic(
                    CONFIG_TX_UUID,
                    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
                 );
  configRxChar = pConfigService->createCharacteristic(
                    CONFIG_RX_UUID,
                    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
                 );
  configRxChar->setCallbacks(new ConfigRxCallbacks());
  pConfigService->start();

  NimBLEAdvertising* pAdvertising = pServer->getAdvertising();
  pAdvertising->setAppearance(0x03C2); // HID Mouse Appearance
  pAdvertising->addServiceUUID(hidDevice->hidService()->getUUID());
  pAdvertising->addServiceUUID(CONFIG_SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->start();
  logPrint("[BLE Server] Advertising HID Mouse & ESP32 KVM Server Config Service...");

  // If targetMouseMac is bound, start persistent background reconnect task
  if (targetMouseMac.length() > 0 && !connected) {
    startMouseReconnectTask();
  }
}

void loop() {
  if (doSaveConfig) {
    executePendingSave();
  }

  if (!bleCmdQueue.empty()) {
    String cmd = bleCmdQueue.front();
    bleCmdQueue.erase(bleCmdQueue.begin());
    logPrint("[BLE RX CMD]: %s", cmd.c_str());
    processCommand(cmd);
  }

  if (doConnect) {
    doConnect = false;
    xTaskCreate([](void* param) {
      connectToServer();
      vTaskDelete(NULL);
    }, "mouseConnTask", 4096, NULL, 1, NULL);
  }

  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() > 0) {
      logPrint("[UART RX CMD]: %s", input.c_str());
      processCommand(input);
    }
  }

#if CONFIG_IDF_TARGET_ESP32S3
  if (USBSerial.available()) {
    String input = USBSerial.readStringUntil('\n');
    input.trim();
    if (input.length() > 0) {
      logPrint("[USB CDC RX CMD]: %s", input.c_str());
      processCommand(input);
    }
  }
#endif
}
