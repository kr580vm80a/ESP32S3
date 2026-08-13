#include <Arduino.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <NimBLEDevice.h>
#include <NimBLEHIDDevice.h>

#if CONFIG_IDF_TARGET_ESP32S3
#include <HWCDC.h>
#endif

Preferences preferences;

// NVS Flash Storage Constants
const char* NVS_NAMESPACE = "kvm_config";
const char* NVS_KEY_LAYOUT = "layout";
const char* NVS_KEY_MOUSE_MAC = "mouse_mac";
const char* NVS_KEY_MOUSE_NAME = "mouse_name";

// Structure to store monitor configuration
struct MonitorConfig {
  String id;
  String name;
  int x;
  int y;
  int width;
  int height;
  String mac;
  int scale = 100;
};

#define MAX_MONITORS 10
MonitorConfig monitors[MAX_MONITORS];
int monitorCount = 0;

// Virtual Cursor Position
long virtualX = 0;
long virtualY = 0;
int currentMonitorIndex = 0;

static float subpixelX = 0.0f;
static float subpixelY = 0.0f;

void resetSubpixelAccumulators() {
    subpixelX = 0.0f;
    subpixelY = 0.0f;
}

// --- BLE Peripheral (Server) Variables ---
NimBLEServer* pServer = nullptr;
NimBLEHIDDevice* hidDevice = nullptr;
NimBLECharacteristic* inputChar = nullptr;

// Active KVM Connections (Mac addresses of connected PCs)
struct KVMClient {
  uint16_t conn_id;
  String mac;
  String name;
  bool active;
};
#define MAX_KVM_CLIENTS 2
KVMClient kvmClients[MAX_KVM_CLIENTS];

const uint8_t hidReportMap[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
    0x09, 0x02,        // Usage (Mouse)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x01,        //   Report ID (1)
    0x09, 0x01,        //   Usage (Pointer)
    0xA1, 0x00,        //   Collection (Physical)
    0x05, 0x09,        //     Usage Page (Button)
    0x19, 0x01,        //     Usage Minimum (0x01)
    0x29, 0x03,        //     Usage Maximum (0x03)
    0x15, 0x00,        //     Logical Minimum (0)
    0x25, 0x01,        //     Logical Maximum (1)
    0x95, 0x03,        //     Report Count (3)
    0x75, 0x01,        //     Report Size (1)
    0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x95, 0x01,        //     Report Count (1)
    0x75, 0x05,        //     Report Size (5)
    0x81, 0x03,        //     Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x05, 0x01,        //     Usage Page (Generic Desktop Ctrls)
    0x09, 0x30,        //     Usage (X)
    0x09, 0x31,        //     Usage (Y)
    0x09, 0x38,        //     Usage (Wheel)
    0x15, 0x81,        //     Logical Minimum (-127)
    0x25, 0x7F,        //     Logical Maximum (127)
    0x75, 0x08,        //     Report Size (8)
    0x95, 0x03,        //     Report Count (3)
    0x81, 0x06,        //     Input (Data,Var,Rel,No Wrap,Linear,Preferred State,No Null Position)
    0x05, 0x0C,        //     Usage Page (Consumer)
    0x0A, 0x38, 0x02,  //     Usage (AC Pan)
    0x15, 0x81,        //     Logical Minimum (-127)
    0x25, 0x7F,        //     Logical Maximum (127)
    0x75, 0x08,        //     Report Size (8)
    0x95, 0x01,        //     Report Count (1)
    0x81, 0x06,        //     Input (Data,Var,Rel,No Wrap,Linear,Preferred State,No Null Position)
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
    Serial.print(buffer);
#if CONFIG_IDF_TARGET_ESP32S3
    USBSerial.print(timeStr);
    USBSerial.print(buffer);
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
        logPrint("[BLE Server] PC Connected! MAC: %s (conn_handle: %d | itvl: %d | latency: %d | timeout: %d)\n",
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
        // Count active connections
        int activeCount = 0;
        for (int i = 0; i < MAX_KVM_CLIENTS; i++) {
            if (kvmClients[i].active) activeCount++;
        }

        // If this is the FIRST connected PC, assign control & calibrate cursor to center of primary screen!
        if (activeCount == 1) {
            String firstMac = peerMac;
            xTaskCreate([](void* param) {
                String* pMac = (String*)param;
                vTaskDelay(pdMS_TO_TICKS(600));
                calibrateFirstConnectedPcToCenter(*pMac);
                delete pMac;
                vTaskDelete(NULL);
            }, "bootCalibTask", 3072, new String(firstMac), 1, NULL);
        }

        // Resume advertising so 2nd PC can discover and connect
        if (activeCount < MAX_KVM_CLIENTS) {
            xTaskCreate([](void* param) {
                vTaskDelay(pdMS_TO_TICKS(1500));
                if (NimBLEDevice::getAdvertising() && !NimBLEDevice::getAdvertising()->isAdvertising()) {
                    logPrint("[BLE Server] Resuming advertising for additional PC...\n");
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
        Serial.printf("[BLE Server] PC Disconnected! MAC: %s (conn_handle: %d)\n",
                      peerMac.c_str(), desc->conn_handle);
        
        for (int i = 0; i < MAX_KVM_CLIENTS; i++) {
            if (kvmClients[i].conn_id == desc->conn_handle || (kvmClients[i].mac.length() > 0 && kvmClients[i].mac.equalsIgnoreCase(peerMac))) {
                kvmClients[i].active = false;
                break;
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
                    Serial.println("[BLE Server] Resuming advertising after PC disconnect...");
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
        logPrint("[BLE Server] Auth Complete for %s | Encrypted: %d | Bonded: %d | KeySize: %d\n",
                  peerMac.c_str(), desc->sec_state.encrypted, desc->sec_state.bonded, desc->sec_state.key_size);
        if (!desc->sec_state.bonded) {
            logPrint("[BLE Server] Bonding incomplete (Bonded: 0) for %s! Clearing stale bond key to allow fresh pairing...\n", peerMac.c_str());
            NimBLEDevice::deleteBond(desc->peer_ota_addr);
        }
    }

    uint32_t onPassKeyRequest() {
        logPrint("[BLE Server] PassKey requested by client\n");
        return 0;
    }

    bool onConfirmPIN(uint32_t pin) {
        logPrint("[BLE Server] PIN confirmation requested: %06d\n", pin);
        return true;
    }
};

String getMonDisplayName(int idx) {
    return monitors[idx].name;
}

uint16_t getTargetConnHandle(const String& targetMac) {
    String cleanTargetMac = targetMac;
    cleanTargetMac.toLowerCase();
    cleanTargetMac.trim();

    // 1. Direct MAC address match
    for (int i = 0; i < MAX_KVM_CLIENTS; i++) {
        if (kvmClients[i].active && kvmClients[i].mac.equalsIgnoreCase(cleanTargetMac)) {
            return kvmClients[i].conn_id;
        }
    }

    // 2. Fallback: Group rank matching by distinct MAC index in layout
    String distinctMacs[MAX_KVM_CLIENTS];
    int distinctCount = 0;
    for (int i = 0; i < monitorCount; i++) {
        String mMac = monitors[i].mac;
        mMac.toLowerCase();
        mMac.trim();
        if (mMac.length() == 0) continue;
        bool exists = false;
        for (int d = 0; d < distinctCount; d++) {
            if (distinctMacs[d].equalsIgnoreCase(mMac)) {
                exists = true;
                break;
            }
        }
        if (!exists && distinctCount < MAX_KVM_CLIENTS) {
            distinctMacs[distinctCount++] = mMac;
        }
    }

    int targetGroupIdx = -1;
    for (int d = 0; d < distinctCount; d++) {
        if (distinctMacs[d].equalsIgnoreCase(cleanTargetMac)) {
            targetGroupIdx = d;
            break;
        }
    }

    if (targetGroupIdx >= 0) {
        int activeIdx = 0;
        for (int i = 0; i < MAX_KVM_CLIENTS; i++) {
            if (kvmClients[i].active) {
                if (activeIdx == targetGroupIdx) {
                    return kvmClients[i].conn_id;
                }
                activeIdx++;
            }
        }
    }

    // 3. Final Fallback: Return first active connection handle
    for (int i = 0; i < MAX_KVM_CLIENTS; i++) {
        if (kvmClients[i].active) {
            return kvmClients[i].conn_id;
        }
    }

    return BLE_HS_CONN_HANDLE_NONE;
}

// --- Unified PC Cursor Calibration & Edge Positioning Function ---
void alignPcCursorToCoordinates(int monIndex, long targetGlobalX, long targetGlobalY, const char* contextLabel) {
    if (monitorCount == 0 || monIndex < 0 || monIndex >= monitorCount) return;

    MonitorConfig& targetMon = monitors[monIndex];
    String targetMac = targetMon.mac;
    targetMac.toLowerCase();
    targetMac.trim();

    // 1. Calculate target PC's dead-end top-left origin (minPcX, minPcY) across all its displays
    int minPcX = 99999;
    int minPcY = 99999;
    for (int i = 0; i < monitorCount; i++) {
        if (monitors[i].mac.equalsIgnoreCase(targetMac)) {
            if (monitors[i].x < minPcX) minPcX = monitors[i].x;
            if (monitors[i].y < minPcY) minPcY = monitors[i].y;
        }
    }
    if (minPcX == 99999) {
        minPcX = targetMon.x;
        minPcY = targetMon.y;
    }

    uint16_t targetConnHandle = getTargetConnHandle(targetMac);

    int16_t relX = (int16_t)(targetGlobalX - minPcX);
    int16_t relY = (int16_t)(targetGlobalY - minPcY);
    if (relX < 0) relX = 0;
    if (relY < 0) relY = 0;

    // Segment-by-segment HID delta integration across displays of target PC (O(N) segment math)
    float sumScaledX = 0.0f;
    for (int i = 0; i < monitorCount; i++) {
        if (monitors[i].mac.equalsIgnoreCase(targetMac)) {
            long segStart = max((long)minPcX, (long)monitors[i].x);
            long segEnd = min(targetGlobalX, (long)(monitors[i].x + monitors[i].width));
            if (segEnd > segStart) {
                float sf = (monitors[i].scale > 0) ? (monitors[i].scale / 100.0f) : 1.0f;
                sumScaledX += (float)(segEnd - segStart) / sf;
            }
        }
    }

    float sumScaledY = 0.0f;
    for (int i = 0; i < monitorCount; i++) {
        if (monitors[i].mac.equalsIgnoreCase(targetMac)) {
            long segStart = max((long)minPcY, (long)monitors[i].y);
            long segEnd = min(targetGlobalY, (long)(monitors[i].y + monitors[i].height));
            if (segEnd > segStart) {
                float sf = (monitors[i].scale > 0) ? (monitors[i].scale / 100.0f) : 1.0f;
                sumScaledY += (float)(segEnd - segStart) / sf;
            }
        }
    }

    int16_t scaledRelX = (int16_t)round(sumScaledX);
    int16_t scaledRelY = (int16_t)round(sumScaledY);

    String monDisplayName = getMonDisplayName(monIndex);

    logPrint("[%s] Aligning PC %s (Mon #%d %s Scale:%d%%) to (%ld, %ld) [Top-Left Origin: %d, %d | Rel: %d, %d | Multi-Mon Scaled HID Rel: %d, %d]...\n",
             contextLabel, targetMac.c_str(), monIndex + 1, monDisplayName.c_str(), targetMon.scale,
             targetGlobalX, targetGlobalY, minPcX, minPcY, relX, relY, scaledRelX, scaledRelY);

    // Step A: Send HID packets to slam OS cursor all the way to target PC's Top-Left origin (minPcX, minPcY)
    // 35 pulses of (-127, -127) = -4445 px, guaranteeing OS cursor is at top-left-most pixel of target PC's virtual desktop
    uint8_t topLeftReport[5] = { 0, (uint8_t)(-127), (uint8_t)(-127), 0, 0 };
    for (int p = 0; p < 35; p++) {
        if (inputChar && targetConnHandle != BLE_HS_CONN_HANDLE_NONE) {
            os_mbuf *om = ble_hs_mbuf_from_flat(topLeftReport, sizeof(topLeftReport));
            if (om != NULL) {
                int rc = ble_gatts_notify_custom(targetConnHandle, inputChar->getHandle(), om);
                if (rc != 0) os_mbuf_free_chain(om);
            }
        } else if (inputChar) {
            inputChar->setValue(topLeftReport, sizeof(topLeftReport));
            inputChar->notify();
        }
        delay(6);
    }

    // Step B: Send HID packets to move from Top-Left origin to target coordinates (scaledRelX, scaledRelY)
    int16_t remainingX = scaledRelX;
    int16_t remainingY = scaledRelY;

    while (remainingX > 0 || remainingY > 0) {
        int8_t stepX = constrain(remainingX, 0, 127);
        int8_t stepY = constrain(remainingY, 0, 127);

        uint8_t stepReport[5] = { 0, (uint8_t)stepX, (uint8_t)stepY, 0, 0 };
        if (inputChar && targetConnHandle != BLE_HS_CONN_HANDLE_NONE) {
            os_mbuf *om = ble_hs_mbuf_from_flat(stepReport, sizeof(stepReport));
            if (om != NULL) {
                int rc = ble_gatts_notify_custom(targetConnHandle, inputChar->getHandle(), om);
                if (rc != 0) os_mbuf_free_chain(om);
            }
        } else if (inputChar) {
            inputChar->setValue(stepReport, sizeof(stepReport));
            inputChar->notify();
        }

        remainingX -= stepX;
        remainingY -= stepY;
        delay(6);
    }

    virtualX = targetGlobalX;
    virtualY = targetGlobalY;
    currentMonitorIndex = monIndex;
    resetSubpixelAccumulators();

    logPrint("[%s] SUCCESS! Positioned & calibrated PC %s at (%ld, %ld) on Monitor #%d (%s)\n",
             contextLabel, targetMac.c_str(), virtualX, virtualY, currentMonitorIndex + 1, monDisplayName.c_str());
}

// --- Boot Center Calibration Wrapper ---
void calibrateFirstConnectedPcToCenter(String targetMac) {
    if (monitorCount == 0) return;
    targetMac.toLowerCase();
    targetMac.trim();

    int targetMonIdx = -1;
    for (int i = 0; i < monitorCount; i++) {
        if (monitors[i].mac.equalsIgnoreCase(targetMac)) {
            targetMonIdx = i;
            break;
        }
    }
    if (targetMonIdx == -1) targetMonIdx = 0;

    MonitorConfig& mon = monitors[targetMonIdx];
    long centerX = mon.x + (mon.width / 2);
    long centerY = mon.y + (mon.height / 2);

    //alignPcCursorToCoordinates(targetMonIdx, centerX, centerY, "BOOT CALIBRATION");
}

// --- BLE Host (Central) Functions ---

void updateVirtualCursorAndSend(uint8_t buttons, int16_t dx, int16_t dy, int8_t scroll, int8_t hScroll) {
    if (monitorCount == 0) return;

    MonitorConfig& currentMon = monitors[currentMonitorIndex];
    int16_t sendDx = dx;
    int16_t sendDy = dy;

    long effectiveDx = dx;
    long effectiveDy = dy;

    if (currentMon.scale != 100) {
        float scaleFactor = currentMon.scale / 100.0f;

        float rawSendX = (dx / scaleFactor) + subpixelX;
        float rawSendY = (dy / scaleFactor) + subpixelY;

        sendDx = (int16_t)truncf(rawSendX);
        sendDy = (int16_t)truncf(rawSendY);

        subpixelX = rawSendX - (float)sendDx;
        subpixelY = rawSendY - (float)sendDy;

        effectiveDx = (long)round(sendDx * scaleFactor);
        effectiveDy = (long)round(sendDy * scaleFactor);
    } else {
        subpixelX = 0.0f;
        subpixelY = 0.0f;
    }

    virtualX += effectiveDx;
    virtualY += effectiveDy;
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
        newMonitorIndex = currentMonitorIndex;
        if (virtualX < currentMon.x) {
            virtualX = currentMon.x;
            logPrint("[CALIBRATION] Calibrated LEFT edge -> virtualX = %ld (%s)\n", virtualX, currentMon.name.c_str());
        } else if (virtualY < currentMon.y) {
            virtualY = currentMon.y;
            logPrint("[CALIBRATION] Calibrated TOP edge -> virtualY = %ld (%s)\n", virtualY, currentMon.name.c_str());
        } else if (virtualX >= currentMon.x + currentMon.width) {
            virtualX = currentMon.x + currentMon.width - 1;
            logPrint("[CALIBRATION] Calibrated RIGHT edge -> virtualX = %ld (%s)\n", virtualX, currentMon.name.c_str());
        } else if (virtualY >= currentMon.y + currentMon.height) {
            virtualY = currentMon.y + currentMon.height - 1;
            logPrint("[CALIBRATION] Calibrated BOTTOM edge -> virtualY = %ld (%s)\n", virtualY, currentMon.name.c_str());
        }
    } else if (newMonitorIndex != currentMonitorIndex) {
        if (monitors[newMonitorIndex].mac.equals(currentMon.mac)) {
            logPrint("[MONITOR SWITCH] Cursor at (%ld, %ld) crossed to Monitor #%s (%s)\n",
                virtualX, virtualY, monitors[newMonitorIndex].id, monitors[newMonitorIndex].name.c_str());
        } else {
            if (virtualX < currentMon.x) {
                virtualX = currentMon.x;
                sendDx -= 50;
                logPrint("[CALIBRATION1] Calibrated LEFT edge -> virtualX = %ld (%s)\n", virtualX, currentMon.name.c_str());
            } else if (virtualY < currentMon.y) {
                virtualY = currentMon.y;
                sendDy -= 50;
                logPrint("[CALIBRATION1] Calibrated TOP edge -> virtualY = %ld (%s)\n", virtualY, currentMon.name.c_str());
            } else if (virtualX >= currentMon.x + currentMon.width) {
                virtualX = currentMon.x + currentMon.width;
                sendDx += 50;
                logPrint("[CALIBRATION1] Calibrated RIGHT edge -> virtualX = %ld (%s)\n", virtualX, currentMon.name.c_str());
            } else if (virtualY >= currentMon.y + currentMon.height) {
                virtualY = currentMon.y + currentMon.height;
                sendDy += 50;
                logPrint("[CALIBRATION1] Calibrated BOTTOM edge -> virtualY = %ld (%s)\n", virtualY, currentMon.name.c_str());
            }
            // Position target PC's OS cursor at exact entering edge coordinates ONLY when switching to a different PC!
            //alignPcCursorToCoordinates(newMonitorIndex, virtualX, virtualY, "KVM SYNC EDGE");
        }
        currentMonitorIndex = newMonitorIndex;
    }

    // Send Standard HID Report (5 bytes: Buttons, dX, dY, VScroll, HScroll)
    uint8_t report[5] = { 
        buttons, 
        (uint8_t)constrain(sendDx, -127, 127), 
        (uint8_t)constrain(sendDy, -127, 127), 
        (uint8_t)constrain(scroll, -127, 127),
        (uint8_t)constrain(hScroll, -127, 127) 
    };

    if (inputChar) {
        uint16_t targetConnHandle = getTargetConnHandle(currentMon.mac);
        if (targetConnHandle != BLE_HS_CONN_HANDLE_NONE) {
            os_mbuf *om = ble_hs_mbuf_from_flat(report, sizeof(report));
            if (om != NULL) {
                int rc = ble_gatts_notify_custom(targetConnHandle, inputChar->getHandle(), om);
                if (rc != 0) os_mbuf_free_chain(om);
            }
        } else {
            inputChar->setValue(report, sizeof(report));
            inputChar->notify();
        }
    }
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
    logPrint("[DECODE] Raw: %02X %02X %02X %02X %02X %02X %02X -> Btn: 0x%02X, dX: %d, dY: %d, VS: %d, HS: %d | Pos: (%ld, %ld) Mon #%s (%s)\n",
                pData[0], pData[1], pData[2], pData[3], pData[4], pData[5], (length > 6 ? pData[6] : 0),
                buttons, x, y, scroll, hScroll, virtualX, virtualY,
                monitors[currentMonitorIndex].id, monitors[currentMonitorIndex].name.c_str());
    updateVirtualCursorAndSend(buttons, x, y, scroll, hScroll);
}

bool connectToServer();
void sendConfigResponse(const String& response);
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
            logPrint("[BLE Scan] TARGET LOCK MATCH! Connecting to %s (%s)\n", devName.c_str(), devMac.c_str());
            NimBLEDevice::getScan()->stop();
            advDevice = new NimBLEAdvertisedDevice(*advertisedDevice);
            doConnect = true;
        }
    }
};

void startMouseReconnectTask() {
    if (targetMouseMac.length() == 0 || isScanningForMice || connected) return;
    if (NimBLEDevice::getScan()->isScanning()) return;

    logPrint("[BLE Host] Starting continuous background scan for mouse (%s)...\n", targetMouseMac.c_str());
    NimBLEScan* pScan = NimBLEDevice::getScan();
    if (pScan) {
        pScan->setAdvertisedDeviceCallbacks(new ScanCallbacks(), false);
        pScan->setActiveScan(true);
        pScan->setInterval(160);
        pScan->setWindow(160);
        pScan->setDuplicateFilter(false);
        pScan->start(0, false); // 0 = continuous background scanning without blind spots
    }
}

// Callback for BLE Connection Status
class ClientCallbacks : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient* pClient) {
        logPrint("[BLE Host] Connected to mouse!\n");
        connected = true;
    }
    void onDisconnect(NimBLEClient* pClient) {
        logPrint("[BLE Host] Disconnected from mouse!\n");
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
        logPrint("[BLE Host] Performing targeted fast probe scan for mouse (%s)...\n", targetMouseMac.c_str());
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
                        logPrint("[BLE Host] AUTO-RESOLVED rotated mouse MAC: %s (was %s)!\n", devMac.c_str(), targetMouseMac.c_str());
                        targetMouseMac = devMac;
                        preferences.begin(NVS_NAMESPACE, false);
                        preferences.putString(NVS_KEY_MOUSE_MAC, targetMouseMac);
                        preferences.end();
                        sendConfigResponse("OK_BIND_MOUSE " + targetMouseMac);
                    }
                    advDevice = new NimBLEAdvertisedDevice(dev);
                    logPrint("[BLE Host] Fast probe scan found mouse: %s (name: %s)!\n", devMac.c_str(), devName.c_str());
                    break;
                }
            }
            pScan->clearResults();
        }
    }

    bool connRes = false;
    if (advDevice) {
        logPrint("[BLE Host] Connecting directly to advertised device: %s...\n", advDevice->getAddress().toString().c_str());
        connRes = pClient->connect(advDevice);
    } else {
        logPrint("[BLE Host] Mouse not advertising yet (retrying in background loop)...\n");
        connRes = false;
    }

    isConnectingToMouse = false;

    int activeCount = 0;
    for (int i = 0; i < MAX_KVM_CLIENTS; i++) {
        if (kvmClients[i].active) activeCount++;
    }
    if (activeCount < MAX_KVM_CLIENTS && NimBLEDevice::getAdvertising() && !NimBLEDevice::getAdvertising()->isAdvertising()) {
        logPrint("[BLE Server] Resuming advertising for additional PC...\n");
        NimBLEDevice::getAdvertising()->start();
    }

    if (!connRes) {
        logPrint("[BLE Host] Connection attempt failed (mouse not advertising or out of range).\n");
        return false;
    }

    logPrint("[BLE Host] Connected! Securing connection (Pairing)...\n");
    if (!pClient->secureConnection()) {
        logPrint("[BLE Host] Initial secureConnection failed. Retrying in 100ms...\n");
        delay(100);
        if (!pClient->secureConnection()) {
            logPrint("[BLE Host] Secure connection retry failed. Proceeding with service discovery...\n");
        } else {
            logPrint("[BLE Host] Connection secured on retry!\n");
        }
    } else {
        logPrint("[BLE Host] Connection secured!\n");
    }

    NimBLERemoteService* pService = pClient->getService(hidServiceUUID);
    if (pService != nullptr) {
        std::vector<NimBLERemoteCharacteristic*>* pChars = pService->getCharacteristics(true);
        for (auto &pChar : *pChars) {
            if (pChar->getUUID() == reportCharUUID) {
                if(pChar->canNotify()) {
                    pChar->subscribe(true, notifyCallback);
                    logPrint("[BLE Host] Subscribed to HID report!\n");
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

    Serial.printf("[BLE TX] Sending %d bytes in %d-byte MTU chunks...\n", (int)len, (int)chunkSize);

    for (size_t i = 0; i < len; i += chunkSize) {
      String chunk = fullResp.substring(i, min(i + chunkSize, len));
      configTxChar->setValue((const uint8_t*)chunk.c_str(), chunk.length());
      configTxChar->notify();
      delay(30);
    }
  }
}


void loadConfiguration() {
  preferences.begin(NVS_NAMESPACE, true);
  String json = preferences.getString(NVS_KEY_LAYOUT, "{}");
  targetMouseMac = preferences.getString(NVS_KEY_MOUSE_MAC, "");
  targetMouseName = preferences.getString(NVS_KEY_MOUSE_NAME, "");
  preferences.end();

  targetMouseMac.toLowerCase();
  targetMouseMac.trim();
  targetMouseName.trim();

  if (json.length() > 2 && json != "[]") {
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, json);
    if (!err) {
      // Restore mouseMac & mouseName if present in saved JSON
      if (doc["mouseMac"].is<String>() && doc["mouseMac"].as<String>().length() > 0) {
        targetMouseMac = doc["mouseMac"].as<String>();
        targetMouseMac.toLowerCase();
        targetMouseMac.trim();
      }
      if (doc["mouseName"].is<String>() && doc["mouseName"].as<String>().length() > 0) {
        targetMouseName = doc["mouseName"].as<String>();
        targetMouseName.trim();
      }

      JsonArray arr;
      if (doc["layouts"].is<JsonArray>() && doc["layouts"].size() > 0) {
        String activeId = doc["activeLayoutId"].as<String>();
        JsonObject activeLayout = doc["layouts"][0].as<JsonObject>();
        for (JsonObject l : doc["layouts"].as<JsonArray>()) {
          if (l["id"].as<String>() == activeId) {
            activeLayout = l;
            break;
          }
        }
        arr = activeLayout["screens"].as<JsonArray>();
      } else if (doc.is<JsonArray>()) {
        arr = doc.as<JsonArray>();
      }

      monitorCount = 0;
      if (arr) {
        for (JsonObject repo : arr) {
          monitors[monitorCount].id = repo["id"].is<String>() ? repo["id"].as<String>() : String(monitorCount + 1);
          monitors[monitorCount].name = repo["name"].is<String>() ? repo["name"].as<String>() : ("Monitor #" + String(monitorCount + 1));
          monitors[monitorCount].x = repo["x"];
          monitors[monitorCount].y = repo["y"];
          monitors[monitorCount].width = repo["width"];
          monitors[monitorCount].height = repo["height"];
          monitors[monitorCount].mac = repo["mac"].as<String>();
          monitors[monitorCount].scale = repo["scale"].is<int>() ? repo["scale"].as<int>() : 100;
          monitorCount++;
        }
      }
      Serial.printf("Loaded %d monitors from NVS.\n", monitorCount);

      if (doc["clients"].is<JsonArray>()) {
        int clientCount = 0;
        for (JsonObject c : doc["clients"].as<JsonArray>()) {
          if (clientCount >= MAX_KVM_CLIENTS) break;
          String mac = c["mac"].as<String>();
          if (mac.length() > 0) {
            kvmClients[clientCount].mac = mac;
            kvmClients[clientCount].name = c["name"].as<String>();
            kvmClients[clientCount].active = false; // Live BLE connection state starts as false at boot
            clientCount++;
          }
        }
      }
    }
  }
}

void saveConfiguration(const String& jsonString) {
  preferences.begin(NVS_NAMESPACE, false);
  preferences.putString(NVS_KEY_LAYOUT, jsonString);

  // Extract and persist mouseMac & mouseName from save payload if present
  JsonDocument doc;
  if (!deserializeJson(doc, jsonString)) {
    if (doc["mouseMac"].is<String>()) {
      String mac = doc["mouseMac"].as<String>();
      mac.toLowerCase();
      mac.trim();
      preferences.putString(NVS_KEY_MOUSE_MAC, mac);
      targetMouseMac = mac;
      Serial.printf("[NVS] Updated targetMouseMac from save payload: %s\n", targetMouseMac.c_str());
    }
    if (doc["mouseName"].is<String>()) {
      String name = doc["mouseName"].as<String>();
      name.trim();
      preferences.putString(NVS_KEY_MOUSE_NAME, name);
      targetMouseName = name;
      Serial.printf("[NVS] Updated targetMouseName from save payload: %s\n", targetMouseName.c_str());
    }
  }
  preferences.end();
  Serial.println("Configuration saved to NVS!");
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
      Serial.printf("[SAVE CONFIG ERROR] Content-Length mismatch: received %d, expected %d\n", (int)jsonStr.length(), expectedLen);
      sendConfigResponse("ERROR_SAVE Content-Length mismatch");
      return;
    }

    if (jsonStr.startsWith("{") || jsonStr.startsWith("[")) {
      pendingSaveJson = jsonStr;
      doSaveConfig = true;
    }
  } else if (input == "GET_CONFIG") {
    preferences.begin(NVS_NAMESPACE, true);
    String json = preferences.getString(NVS_KEY_LAYOUT, "{}");
    preferences.end();

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, json);

    if (err || !doc.is<JsonObject>()) {
      doc.clear();
      doc["device"] = "ESP32-KVM-Switch";
      doc["activeLayoutId"] = "layout_1";
      doc["totalLayouts"] = 1;
      JsonArray layoutsArr = doc["layouts"].to<JsonArray>();
      JsonObject layout1 = layoutsArr.add<JsonObject>();
      layout1["id"] = "layout_1";
      layout1["name"] = "Default Layout";
      layout1["totalScreens"] = 0;
      layout1["screens"].to<JsonArray>();
      doc["clients"].to<JsonArray>();
    }

    // Include bound mouse MAC & Name in the unified JSON payload for backup & restore
    doc["mouseMac"] = targetMouseMac;
    doc["mouseName"] = targetMouseName.length() > 0 ? targetMouseName : (targetMouseMac.length() > 0 ? "BLE Mouse" : "");

    // Reset connected status for stored clients prior to active connection sync
    if (doc["clients"].is<JsonArray>()) {
      for (JsonObject c : doc["clients"].as<JsonArray>()) {
        c["connected"] = false;
      }
    }

    // Dynamic merge of active connected BLE clients into unified JSON
    JsonArray clientsArr = doc["clients"].is<JsonArray>() ? doc["clients"].as<JsonArray>() : doc["clients"].to<JsonArray>();
    for (int i = 0; i < MAX_KVM_CLIENTS; i++) {
      if (kvmClients[i].active && kvmClients[i].mac.length() > 0) {
        String activeMac = kvmClients[i].mac;
        activeMac.toLowerCase();
        activeMac.trim();

        bool exists = false;
        for (JsonObject c : clientsArr) {
          String cMac = c["mac"].as<String>();
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
      Serial.println("[BLE Scan] Cancelling background mouseReconnectTask for discovery scan...");
      vTaskDelete(reconnTaskHandle);
      reconnTaskHandle = NULL;
    }
    if (pClient && pClient->isConnected()) {
      Serial.println("[BLE Scan] Disconnecting from mouse before discovery scan...");
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

      Serial.println("[BLE Scan] Starting 5-second active discovery scan for mice...");
      pScan->start(5, false);
      pScan->clearResults();
    }
    isScanningForMice = false;
    Serial.printf("[BLE Scan] Discovery scan complete! Discovered %d BLE devices.\n", (int)scannedMiceDoc.as<JsonArray>().size());

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
    mac.toLowerCase();
    mac.trim();
    preferences.begin(NVS_NAMESPACE, false);
    preferences.putString(NVS_KEY_MOUSE_MAC, mac);
    preferences.putString(NVS_KEY_MOUSE_NAME, name);
    preferences.end();
    targetMouseMac = mac;
    targetMouseName = name;
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
    preferences.begin(NVS_NAMESPACE, false);
    preferences.remove(NVS_KEY_MOUSE_MAC);
    preferences.remove(NVS_KEY_MOUSE_NAME);
    preferences.end();
    targetMouseMac = "";
    targetMouseName = "";
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
    preferences.begin(NVS_NAMESPACE, true);
    String json = preferences.getString(NVS_KEY_LAYOUT, "{}");
    String mouseMac = preferences.getString(NVS_KEY_MOUSE_MAC, "");
    preferences.end();
    Serial.println("\n--- [NVS FLASH DUMP] ---");
    Serial.printf("Flash layout string length: %d bytes\n", json.length());
    Serial.printf("Flash target mouse MAC: %s\n", mouseMac.c_str());
    Serial.println(json);
    Serial.println("--- [END NVS FLASH DUMP] ---\n");
  }
}

static String bleRxBuffer = "";
static std::vector<String> bleCmdQueue;

class ConfigRxCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pCharacteristic) {
        std::string rxValue = pCharacteristic->getValue();
        Serial.printf("[BLE RX DEBUG]: Received %d bytes: '%s'\n", (int)rxValue.length(), rxValue.c_str());
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
  Serial.setRxBufferSize(2048);
  Serial.begin(115200);
#if CONFIG_IDF_TARGET_ESP32S3
  USBSerial.begin(115200);
#endif
  delay(2000);
  
  logPrint("\n--- ESP32 KVM Switcher Started ---\n");
  loadConfiguration();
  
  logPrint("[BLE] Initializing NimBLE...\n");
  NimBLEDevice::init("ESP32 KVM Mouse");
  NimBLEDevice::setMTU(512);
  NimBLEDevice::setSecurityAuth(true, false, false); // Compatible Just Works pairing (bonding=true, mitm=false, sc=false for legacy compatibility)
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
  
  int numBonds = NimBLEDevice::getNumBonds();
  logPrint("[BLE NVS BONDS] Saved bonded devices count: %d\n", numBonds);
  for (int i = 0; i < numBonds; i++) {
      NimBLEAddress bondAddr = NimBLEDevice::getBondedAddress(i);
      logPrint("  -> Bonded Device #%d: MAC %s\n", i + 1, bondAddr.toString().c_str());
  }
  
  // Setup BLE Server (Peripheral)
  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());
  hidDevice = new NimBLEHIDDevice(pServer);
  inputChar = hidDevice->inputReport(1); // Report ID 1
  
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
  logPrint("[BLE Server] Advertising HID Mouse & ESP32 KVM Server Config Service...\n");

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
    Serial.print("[BLE RX CMD]: ");
    Serial.println(cmd);
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
      Serial.printf("[UART RX CMD]: %s\n", input.c_str());
      processCommand(input);
    }
  }

#if CONFIG_IDF_TARGET_ESP32S3
  if (USBSerial.available()) {
    String input = USBSerial.readStringUntil('\n');
    input.trim();
    if (input.length() > 0) {
      Serial.printf("[USB CDC RX CMD]: %s\n", input.c_str());
      processCommand(input);
    }
  }
#endif
}
