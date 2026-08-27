#include <Arduino.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <NimBLEDevice.h>
#include <NimBLEHIDDevice.h>
#include "nimble/nimble/host/include/host/ble_gap.h"
#include <esp_mac.h>
#include <mbedtls/sha256.h>

#if CONFIG_IDF_TARGET_ESP32S3
#include <HWCDC.h>
#endif

#define BLE_DEVICE_NAME "ESP32 KVM Mouse"

Preferences preferences;

// NVS Flash Storage Constants
const char* NVS_NAMESPACE = "kvm_config";
const char* NVS_KEY_LAYOUT = "layout";

enum os {
    OS_WINDOWS = 0,
    OS_MAC = 1
};
// Structure to store monitor configuration
struct MonitorConfig {
    int id = 1;
    String name;
    int x;
    int y;
    int width;
    int height;
    String mac;
    int os = OS_WINDOWS;
    int scale = 100;
    bool isPrimary = false;
};

#define MAX_MONITORS 10
MonitorConfig monitors[MAX_MONITORS];
int monitorCount = 0;

// --- BLE Peripheral (Server) Variables ---
NimBLEServer* pServer = nullptr;
NimBLEHIDDevice* hidDevice = nullptr;
NimBLECharacteristic* inputChar = nullptr;
NimBLECharacteristic* absInputChar = nullptr;
NimBLECharacteristic* macAbsInputChar = nullptr;
NimBLECharacteristic* keyboardInputChar = nullptr;
NimBLECharacteristic* keyboardOutputChar = nullptr;
NimBLECharacteristic* mediaInputChar = nullptr;

// --- Security & Authentication Constants ---
#define BLE_PAIRING_PIN             123456           // Static 6-digit PIN code for PC Bluetooth pairing
#define WEB_BLE_AUTH_PASSPHRASE     "esp32s3"        // Passphrase required for Web Bluetooth browser access

static bool isWebBleAuthenticated = false;
static String currentAuthNonce = "";

String calculateSha256(const String& input) {
    uint8_t hash[32];
    mbedtls_sha256((const unsigned char*)input.c_str(), input.length(), hash, 0);
    char hexStr[65];
    for (int i = 0; i < 32; i++) {
        sprintf(&hexStr[i * 2], "%02x", hash[i]);
    }
    hexStr[64] = '\0';
    return String(hexStr);
}

// Active KVM Connections (Mac addresses of connected PCs)
struct KVMClient {
  uint16_t conn_id;
  String mac;
  String name;
  bool active;
};
#define MAX_SUPPORTED_KVM_CLIENTS 6 // 6 PCs + 1 Mouse + 1 Keyboard + 1 Web = 9 max NimBLE connections
KVMClient kvmClients[MAX_SUPPORTED_KVM_CLIENTS];
int maxKvmClients = 3; // Dynamic variable based on number of PCs in current configuration
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
    // --- REPORT ID 1: Standard HID Keyboard (6KRO) ---
    0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
    0x09, 0x06,        // Usage (Keyboard)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x01,        //   Report ID (1)
    0x05, 0x07,        //   Usage Page (Keyboard/Keypad)
    0x19, 0xE0,        //   Usage Minimum (Keyboard LeftControl)
    0x29, 0xE7,        //   Usage Maximum (Keyboard Right GUI)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x08,        //   Report Count (8 bits for modifiers)
    0x81, 0x02,        //   Input (Data,Var,Abs) - Modifiers
    0x95, 0x01,        //   Report Count (1)
    0x75, 0x08,        //   Report Size (8)
    0x81, 0x01,        //   Input (Const,Array,Abs) - Reserved byte
    0x95, 0x05,        //   Report Count (5)
    0x75, 0x01,        //   Report Size (1)
    0x05, 0x08,        //   Usage Page (LEDs)
    0x19, 0x01,        //   Usage Minimum (Num Lock)
    0x29, 0x05,        //   Usage Maximum (Kana)
    0x91, 0x02,        //   Output (Data,Var,Abs) - LEDs
    0x95, 0x01,        //   Report Count (1)
    0x75, 0x03,        //   Report Size (3)
    0x91, 0x01,        //   Output (Const,Array,Abs) - Padding
    0x95, 0x06,        //   Report Count (6)
    0x75, 0x08,        //   Report Size (8)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x65,        //   Logical Maximum (101 keys)
    0x05, 0x07,        //   Usage Page (Keyboard/Keypad)
    0x19, 0x00,        //   Usage Minimum (Reserved)
    0x29, 0x65,        //   Usage Maximum (Keyboard Application)
    0x81, 0x00,        //   Input (Data,Array,Abs) - 6 keycodes
    0xC0,              // End Collection

    // --- REPORT ID 2: Standard Relative Mouse (for natural physical movement) ---
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
    0x16, 0x01, 0x80,  //     Logical Minimum (-32767)
    0x26, 0xFF, 0x7F,  //     Logical Maximum (32767)
    0x75, 0x10,        //     Report Size (16)
    0x95, 0x02,        //     Report Count (2: X, Y)
    0x81, 0x06,        //     Input (Data,Var,Rel)
    0x09, 0x38,        //     Usage (Wheel)
    0x15, 0x81,        //     Logical Minimum (-127)
    0x25, 0x7F,        //     Logical Maximum (127)
    0x75, 0x08,        //     Report Size (8)
    0x95, 0x01,        //     Report Count (1: Wheel)
    0x81, 0x06,        //     Input (Data,Var,Rel)
    0x05, 0x0C,        //     Usage Page (Consumer)
    0x0A, 0x38, 0x02,  //     Usage (AC Pan)
    0x15, 0x81,        //     Logical Minimum (-127)
    0x25, 0x7F,        //     Logical Maximum (127)
    0x75, 0x08,        //     Report Size (8)
    0x95, 0x01,        //     Report Count (1: AC Pan)
    0x81, 0x06,        //     Input (Data,Var,Rel)
    0xC0,              //   End Collection
    0xC0,              // End Collection

    // --- REPORT ID 3: Absolute Pointer (for Windows / Android multi-monitor transitions) ---
    0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
    0x09, 0x01,        // Usage (Pointer)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x03,        //   Report ID (3)
    0x05, 0x01,        //   Usage Page (Generic Desktop Ctrls)
    0x09, 0x30,        //   Usage (X)
    0x09, 0x31,        //   Usage (Y)
    0x16, 0x00, 0x00,  //   Logical Minimum (0)
    0x26, 0xFF, 0x7F,  //   Logical Maximum (32767)
    0x75, 0x10,        //   Report Size (16)
    0x95, 0x02,        //   Report Count (2: X, Y)
    0x81, 0x02,        //   Input (Data,Var,Abs)
    0xC0,              // End Collection

    // --- REPORT ID 5: Absolute Digitizer Pen (for macOS / iPadOS transitions) ---
    0x05, 0x0D,        // Usage Page (Digitizers)
    0x09, 0x02,        // Usage (Pen)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x05,        //   Report ID (5)
    0x09, 0x32,        //   Usage (In Range)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x01,        //   Report Count (In Range)
    0x81, 0x02,        //   Input (Data,Var,Abs)
    0x75, 0x07,        //   Report Size (7)
    0x95, 0x01,        //   Report Count (1: Padding)
    0x81, 0x03,        //   Input (Const,Var,Abs)
    0x05, 0x01,        //   Usage Page (Generic Desktop Ctrls)
    0x09, 0x30,        //   Usage (X)
    0x09, 0x31,        //   Usage (Y)
    0x16, 0x00, 0x00,  //   Logical Minimum (0)
    0x26, 0xFF, 0x7F,  //   Logical Maximum (32767)
    0x36, 0x00, 0x00,  //   Physical Minimum (0)
    0x46, 0xFF, 0x7F,  //   Physical Maximum (32767)
    0x75, 0x10,        //   Report Size (16)
    0x95, 0x02,        //   Report Count (2: X, Y)
    0x81, 0x02,        //   Input (Data,Var,Abs)
    0xC0,              // End Collection

    // --- REPORT ID 4: Consumer Control (Media Keys) ---
    0x05, 0x0C,        // Usage Page (Consumer)
    0x09, 0x01,        // Usage (Consumer Control)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x04,        //   Report ID (4)
    0x15, 0x00,        //   Logical Minimum (0)
    0x26, 0xFF, 0x03,  //   Logical Maximum (1023)
    0x19, 0x00,        //   Usage Minimum (Unassigned)
    0x2A, 0xFF, 0x03,  //   Usage Maximum (1023)
    0x75, 0x10,        //   Report Size (16)
    0x95, 0x01,        //   Report Count (1)
    0x81, 0x00,        //   Input (Data,Array,Abs)
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
static bool mouseConnected = false;
static bool isConnectingToMouse = false;
static NimBLEClient* pClient = nullptr;
static NimBLEAdvertisedDevice* advDevice = nullptr;
static bool doConnectMouse = false;

// Keyboard Central Variables
static String targetKeyboardMac = "";
static String targetKeyboardName = "";
static bool kbConnected = false;
static bool isConnectingToKeyboard = false;
static NimBLEClient* pKbClient = nullptr;
static NimBLEAdvertisedDevice* advKbDevice = nullptr;
static bool doConnectKeyboard = false;

static NimBLEUUID hidServiceUUID("1812");
static NimBLEUUID reportCharUUID("2a4d");

static uint32_t lastConfigActivityTime = 0;

bool isMacInActiveLayout(const String& mac) {
    if (mac.length() == 0) return false;
    for (int i = 0; i < monitorCount; i++) {
        if (monitors[i].mac.equals(mac)) return true;
    }
    return false;
}

int getActiveLayoutPcCount() {
    String distinctMacs[MAX_SUPPORTED_KVM_CLIENTS];
    int count = 0;
    for (int i = 0; i < monitorCount; i++) {
        if (monitors[i].mac.length() == 0) continue;
        bool found = false;
        for (int k = 0; k < count; k++) {
            if (distinctMacs[k].equals(monitors[i].mac)) {
                found = true;
                break;
            }
        }
        if (!found && count < MAX_SUPPORTED_KVM_CLIENTS) {
            distinctMacs[count++] = monitors[i].mac;
        }
    }
    return count;
}

bool isConfigModeActive() {
    return isWebBleAuthenticated || (lastConfigActivityTime > 0 && (millis() - lastConfigActivityTime < 60000));
}

void checkAndResumeAdvertising() {
    int activeCount = 0;
    for (int i = 0; i < MAX_SUPPORTED_KVM_CLIENTS; i++) {
        if (kvmClients[i].active) activeCount++;
    }

    // Hardware ACL budget: 8 max simultaneous connections in ESP32-S3 controller (10 activities).
    int maxAllowedPcConnections = max(2, 8 - (targetMouseMac.length() > 0 ? 1 : 0) - (targetKeyboardMac.length() > 0 ? 1 : 0));

    // When in config mode or when PCs are not full: keep advertising active so Web Bluetooth and PCs can connect!
    int layoutPcs = getActiveLayoutPcCount();
    int targetPcLimit = isConfigModeActive() ? maxAllowedPcConnections : (layoutPcs > 0 ? max(layoutPcs, 2) : 2);
    if (targetPcLimit > maxAllowedPcConnections) targetPcLimit = maxAllowedPcConnections;

    if (activeCount < maxAllowedPcConnections) {
        if (NimBLEDevice::getAdvertising() && !NimBLEDevice::getAdvertising()->isAdvertising()) {
            logPrint("[BLE Server] Advertising active (PCs: %d/%d [Layout: %d, ConfigMode: %s] | Mouse: %d | KB: %d)", 
                     activeCount, targetPcLimit, layoutPcs, isConfigModeActive() ? "YES" : "NO",
                     mouseConnected ? 1 : 0, kbConnected ? 1 : 0);
            NimBLEDevice::getAdvertising()->start();
        }
    } else {
        if (NimBLEDevice::getAdvertising() && NimBLEDevice::getAdvertising()->isAdvertising()) {
            logPrint("[BLE Server] Advertising paused (PCs connected: %d/%d [Layout: %d, ConfigMode: %s])", 
                     activeCount, targetPcLimit, layoutPcs, isConfigModeActive() ? "YES" : "NO");
            NimBLEDevice::getAdvertising()->stop();
        }
    }
}

class SecurityCallbacks : public NimBLESecurityCallbacks {
    uint32_t onPassKeyRequest() {
        logPrint("[BLE Security] =========================================");
        logPrint("[BLE Security] >>> onPassKeyRequest: RETURNING %06lu <<<", (unsigned long)BLE_PAIRING_PIN);
        logPrint("[BLE Security] =========================================");
        return BLE_PAIRING_PIN;
    }
    void onPassKeyNotify(uint32_t pass_key) {
        logPrint("[BLE Security] =========================================");
        logPrint("[BLE Security] >>> TYPE THIS PASSKEY ON KEYBOARD: %06lu <<<", (unsigned long)pass_key);
        logPrint("[BLE Security] >>> AND PRESS ENTER ON MX KEYS S <<<");
        logPrint("[BLE Security] =========================================");
    }
    bool onConfirmPIN(uint32_t pass_key) {
        logPrint("[BLE Security] =========================================");
        logPrint("[BLE Security] >>> onConfirmPIN: %06lu (auto-confirmed) <<<", (unsigned long)pass_key);
        logPrint("[BLE Security] =========================================");
        return true;
    }
    bool onSecurityRequest() {
        logPrint("[BLE Security] onSecurityRequest -> Accepted");
        return true;
    }
    void onAuthenticationComplete(ble_gap_conn_desc* desc) {
        logPrint("[BLE Security] onAuthenticationComplete: conn=%d, enc=%d, auth=%d, bonded=%d",
                 desc->conn_handle, desc->sec_state.encrypted, desc->sec_state.authenticated, desc->sec_state.bonded);
        checkAndResumeAdvertising();
    }
};

void calibrateFirstConnectedPcToCenter(String targetMac);

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) {
        String peerMac = NimBLEAddress(desc->peer_ota_addr).toString().c_str();
        peerMac.toLowerCase();
        peerMac.trim();
        logPrint("[BLE Server] PC Connected! MAC: %s (conn_handle: %d | itvl: %d | latency: %d | timeout: %d)",
                  peerMac.c_str(), desc->conn_handle, desc->conn_itvl, desc->conn_latency, desc->supervision_timeout);
        
        // If this PC is NOT in the active layout and Config Mode is NOT active:
        // Reject it to keep the radio channel free for active layout PCs!
        if (getActiveLayoutPcCount() > 0 && !isMacInActiveLayout(peerMac) && !isConfigModeActive()) {
            logPrint("[BLE Server] Rejected unconfigured PC %s (Not in active layout and Config Mode is inactive)", peerMac.c_str());
            pServer->disconnect(desc->conn_handle);
            return;
        }

        // Save connection
        bool updated = false;
        for (int i = 0; i < MAX_SUPPORTED_KVM_CLIENTS; i++) {
            if (kvmClients[i].mac.equals(peerMac)) {
                kvmClients[i].conn_id = desc->conn_handle;
                kvmClients[i].active = true;
                updated = true;
                break;
            }
        }
        if (!updated) {
            for (int i = 0; i < MAX_SUPPORTED_KVM_CLIENTS; i++) {
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
        for (int i = 0; i < MAX_SUPPORTED_KVM_CLIENTS; i++) {
            if (kvmClients[i].active) activeCount++;
        }

        // If this is the FIRST connected PC that is in active layout, calibrate cursor to center
        if (activeCount == 1 && isMacInActiveLayout(peerMac)) {
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

        checkAndResumeAdvertising();
    }

    void onDisconnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) {
        String peerMac = NimBLEAddress(desc->peer_ota_addr).toString().c_str();
        peerMac.toLowerCase();
        peerMac.trim();
        logPrint("[BLE Server] PC Disconnected! MAC: %s (conn_handle: %d)", peerMac.c_str(), desc->conn_handle);

        isWebBleAuthenticated = false; // Reset Web Bluetooth authorization on client disconnect
        currentAuthNonce = "";

        if (bootCalibTaskHandle != NULL) {
            vTaskDelete(bootCalibTaskHandle);
            bootCalibTaskHandle = NULL;
        }
        
        for (int i = 0; i < MAX_SUPPORTED_KVM_CLIENTS; i++) {
            if (kvmClients[i].conn_id == desc->conn_handle || kvmClients[i].mac.equals(peerMac)) {
                kvmClients[i].active = false;
                break;
            }
        }

        // If the disconnected PC was the current active PC, failover to another connected PC!
        if (monitorCount > 0 && monitors[currentMonitorIndex].mac.equals(peerMac)) {
            String fallbackMac = "";
            for (int i = 0; i < MAX_SUPPORTED_KVM_CLIENTS; i++) {
                if (kvmClients[i].active && kvmClients[i].mac.length() > 0 && !kvmClients[i].mac.equals(peerMac)) {
                    fallbackMac = kvmClients[i].mac;
                    break;
                }
            }

            if (fallbackMac.length() > 0) {
                calibrateFirstConnectedPcToCenter(fallbackMac);
                logPrint("[FAILOVER] Current PC %s disconnected! Switched control to active PC %s (Cursor at %ld, %ld)", peerMac.c_str(), fallbackMac.c_str(), virtualX, virtualY);
            } else {
                logPrint("[FAILOVER] Current PC %s disconnected and no other active PC available.", peerMac.c_str());
            }
        }

        checkAndResumeAdvertising();
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
};

String getMonDisplayName(int idx) {
    return monitors[idx].name;
}

uint16_t getTargetConnHandle(const String& targetMac) {
    for (int i = 0; i < MAX_SUPPORTED_KVM_CLIENTS; i++) {
        if (kvmClients[i].active && kvmClients[i].mac.equals(targetMac)) {
            return kvmClients[i].conn_id;
        }
    }
    return BLE_HS_CONN_HANDLE_NONE;
}

// Send HID report to target connection handle or broadcast notify
void sendHidReport(NimBLECharacteristic* pChar, uint16_t connHandle, const uint8_t* report, size_t length = 7) {
    if (!pChar || !report || length == 0) return;
    if (connHandle != BLE_HS_CONN_HANDLE_NONE) {
        os_mbuf *om = ble_hs_mbuf_from_flat(report, length);
        if (om != NULL) {
            int rc = ble_gatts_notify_custom(connHandle, pChar->getHandle(), om);
            if (rc != 0) {
                logPrint("[BLE HID NOTIFY ERR] conn: %d handle: 0x%04X rc: %d", connHandle, pChar->getHandle(), rc);
                os_mbuf_free_chain(om);
            }
        }
    } else {
        pChar->setValue(report, length);
        pChar->notify();
    }
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
    if (targetGlobalY < primaryMon.y - 1 - shift) {
        jumpY = primaryMon.y;
        if (targetGlobalX < (primaryMon.x + shift)) {
            // use top left corner
            jumpX = primaryMon.x + shift;
            step1Y = -1;
            shiftX = -shift;
            shiftY = 1;
        } else if (targetGlobalX > (primaryMon.x + primaryMon.width - 1 - shift)) {
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
    } else if (targetGlobalY > (primaryMon.y + primaryMon.height + shift)) {
        jumpY = primaryMon.y + primaryMon.height - 1;
        if (targetGlobalX < (primaryMon.x + shift)) {
            // use bottom left corner
            jumpX = primaryMon.x + shift;
            step1Y = 1;
            shiftX = -shift;
            shiftY = -1;
        } else if (targetGlobalX > (primaryMon.x + primaryMon.width - 1 - shift)) {
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
        uint8_t report[7] = {
            0,
            (uint8_t)(step1X & 0xFF),
            (uint8_t)((step1X >> 8) & 0xFF),
            (uint8_t)(step1Y & 0xFF),
            (uint8_t)((step1Y >> 8) & 0xFF),
            0,
            0
        };
        sendHidReport(inputChar, connHandle, report, sizeof(report));
        logPrint("Window step position at (%ld, %ld)", step1X, step1Y);
    }

    int32_t deltaX = targetGlobalX - jumpX + shiftX;
    int32_t deltaY = targetGlobalY - jumpY + shiftY;
    logPrint("Window move at (%ld, %ld)", deltaX, deltaY);
    uint8_t report[7] = {
        0,
        (uint8_t)(deltaX & 0xFF),
        (uint8_t)((deltaX >> 8) & 0xFF),
        (uint8_t)(deltaY & 0xFF),
        (uint8_t)((deltaY >> 8) & 0xFF),
        0,
        0
    };
    sendHidReport(inputChar, connHandle, report, sizeof(report));
}

// --- Absolute HID Positioning Function ---
void sendAbsoluteCoordinatesMacOs(uint16_t connHandle, int monIndex, long targetGlobalX, long targetGlobalY, const char* contextLabel) {
    MonitorConfig& targetMon = monitors[monIndex];
    long relX = constrain(targetGlobalX - targetMon.x, 0, targetMon.width);
    long relY = constrain(targetGlobalY - targetMon.y, 0, targetMon.height);
    uint16_t absX = (uint16_t)round(((float)relX / (float)targetMon.width) * 32767.0f);
    uint16_t absY = (uint16_t)round(((float)relY / (float)targetMon.height) * 32767.0f);
    long maxOtherY = targetMon.y;
    long minOtherY = targetMon.y;
    for (int i = 0; i < monitorCount; i++) {
        if (monitors[i].mac.equals(targetMon.mac)) {
            if (monitors[i].y > maxOtherY) maxOtherY = monitors[i].y;
            if (monitors[i].y < minOtherY) minOtherY = monitors[i].y;
        }
    }

    uint8_t anchorReport[7] = { 0, 0, 0, 0, 0, 0, 0 };
    if (targetMon.y >= maxOtherY && maxOtherY > minOtherY) {
        // Target is bottom monitor (Retina) -> send rapid burst DOWN to force focus to bottom screen
        int16_t dY = 500;
        anchorReport[3] = (uint8_t)(dY & 0xFF);
        anchorReport[4] = (uint8_t)((dY >> 8) & 0xFF);
        for (int k = 0; k < 3; k++) {
            sendHidReport(inputChar, connHandle, anchorReport, sizeof(anchorReport));
        }
    } else if (targetMon.y <= minOtherY && maxOtherY > minOtherY) {
        // Target is top monitor (DELL) -> send rapid burst UP to force focus to top screen
        int16_t dY = -500;
        anchorReport[3] = (uint8_t)(dY & 0xFF);
        anchorReport[4] = (uint8_t)((dY >> 8) & 0xFF);
        for (int k = 0; k < 3; k++) {
            sendHidReport(inputChar, connHandle, anchorReport, sizeof(anchorReport));
        }
    }

    uint8_t absReport[5] = {
        0x01,                               // In Range = ON
        (uint8_t)(absX & 0xFF),
        (uint8_t)((absX >> 8) & 0xFF),
        (uint8_t)(absY & 0xFF),
        (uint8_t)((absY >> 8) & 0xFF)
    };
    sendHidReport(macAbsInputChar, connHandle, absReport, sizeof(absReport));
    absReport[0] = 0x00;                    // In Range = OFF (release hover)
    sendHidReport(macAbsInputChar, connHandle, absReport, sizeof(absReport));
    logPrint("[%s] Sent macOS digitizer position to PC %s at (%ld, %ld) [Rel: %ld, %ld -> Norm: %u, %u] on Mon #%d (%s)",
                contextLabel, targetMon.mac.c_str(), targetGlobalX, targetGlobalY, relX, relY, absX, absY, targetMon.id, targetMon.name.c_str());
}

// --- Absolute HID Positioning Function ---
void sendAbsoluteCoordinates(uint16_t connHandle, int monIndex, long targetGlobalX, long targetGlobalY, const char* contextLabel) {
    if (monitorCount == 0) return;
    if (monitors[monIndex].os == OS_MAC) {
        sendAbsoluteCoordinatesMacOs(connHandle, monIndex, targetGlobalX, targetGlobalY, contextLabel);
    } else {
        sendAbsoluteCoordinatesWindows(connHandle, monIndex, targetGlobalX, targetGlobalY, contextLabel);
    }
}

// --- Boot Center Calibration Wrapper ---
void calibrateFirstConnectedPcToCenter(String targetMac) {
    if (monitorCount == 0) return;
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
}

// --- BLE Host (Central) Functions ---

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
                // Send safe key release to old PC so no keys remain stuck
                if (keyboardInputChar) {
                    uint8_t keyRelease[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
                    sendHidReport(keyboardInputChar, connHandle, keyRelease, sizeof(keyRelease));
                }
                currentMonitorIndex = newMonitorIndex;
                logPrint("[PC SWITCH] Cursor saved at (%ld, %ld)", virtualX, virtualY);
                sendAbsoluteCoordinates(targetConn, newMonitorIndex, virtualX, virtualY, "PC SWITCH");
            }
        }
        resetSubpixelAccumulators();
    }

    // Send 16-bit Relative HID Mouse Report (7 bytes: Buttons, dX_low, dX_high, dY_low, dY_high, VScroll, HScroll)
    uint8_t report[7] = { 
        buttons,
        (uint8_t)(sendDx & 0xFF),
        (uint8_t)((sendDx >> 8) & 0xFF),
        (uint8_t)(sendDy & 0xFF),
        (uint8_t)((sendDy >> 8) & 0xFF),
        (uint8_t)constrain(scroll, -127, 127), 
        (uint8_t)constrain(hScroll, -127, 127) 
    };
    sendHidReport(inputChar, connHandle, report, sizeof(report));
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

// Callback when HID data is received from the keyboard (Follow-the-Mouse)
void keyboardNotifyCallback(NimBLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    if (!pData || length == 0) return;

    // Log the raw incoming keyboard packet
    String hexDump = "";
    for (size_t i = 0; i < length; i++) {
        char buf[4];
        snprintf(buf, sizeof(buf), "%02X ", pData[i]);
        hexDump += buf;
    }
    logPrint("[KEYBOARD RX RAW] %s (len: %d)", hexDump.c_str(), length);

    if (monitorCount == 0) return;

    uint16_t targetConn = getTargetConnHandle(monitors[currentMonitorIndex].mac);
    if (targetConn == BLE_HS_CONN_HANDLE_NONE) {
        // Fallback to any active connected PC if current monitor target is not matched
        for (int i = 0; i < maxKvmClients; i++) {
            if (kvmClients[i].active && kvmClients[i].conn_id != BLE_HS_CONN_HANDLE_NONE) {
                targetConn = kvmClients[i].conn_id;
                break;
            }
        }
    }
    if (targetConn == BLE_HS_CONN_HANDLE_NONE) return;

    if (length == 7) {
        // 7-byte report from Logitech MX Keys: [modifiers, key1, key2, key3, key4, key5, key6]
        // Standard HID 6KRO report requires 8 bytes: [modifiers, reserved(0x00), key1, key2, key3, key4, key5, key6]
        uint8_t rep8[8];
        rep8[0] = pData[0]; // Modifiers (Shift, Ctrl, Alt, GUI)
        rep8[1] = 0x00;     // Reserved
        rep8[2] = pData[1]; // Key 1
        rep8[3] = pData[2]; // Key 2
        rep8[4] = pData[3]; // Key 3
        rep8[5] = pData[4]; // Key 4
        rep8[6] = pData[5]; // Key 5
        rep8[7] = pData[6]; // Key 6
        sendHidReport(keyboardInputChar, targetConn, rep8, 8);
        logPrint("[KEYBOARD FWD] 7B->8B [Mods: 0x%02X, Key1: 0x%02X] -> Conn %d (Mon #%d)", rep8[0], rep8[2], targetConn, currentMonitorIndex + 1);
    } else if (length == 8) {
        // Standard 8-byte keyboard report: [mods, res, k1, k2, k3, k4, k5, k6]
        sendHidReport(keyboardInputChar, targetConn, pData, length);
        logPrint("[KEYBOARD FWD] 8B -> Conn %d (Mon #%d)", targetConn, currentMonitorIndex + 1);
    } else if (length == 9) {
        // 9-byte report with Report ID prepended: forward payload without Report ID
        sendHidReport(keyboardInputChar, targetConn, &pData[1], 8);
        logPrint("[KEYBOARD FWD] 9B (ID 0x%02X) -> Conn %d (Mon #%d)", pData[0], targetConn, currentMonitorIndex + 1);
    } else if (length == 2) {
        // Consumer Control report (Media keys)
        sendHidReport(mediaInputChar, targetConn, pData, length);
        logPrint("[MEDIA FWD] 2B -> Conn %d (Mon #%d)", targetConn, currentMonitorIndex + 1);
    } else if (length == 3) {
        // Consumer Control report with Report ID prepended
        sendHidReport(mediaInputChar, targetConn, &pData[1], 2);
        logPrint("[MEDIA FWD] 3B (ID 0x%02X) -> Conn %d (Mon #%d)", pData[0], targetConn, currentMonitorIndex + 1);
    } else if (length == 19 || pData[0] == 0xFF) {
        // Logitech HID++ vendor packet: ignore
    } else {
        sendHidReport(keyboardInputChar, targetConn, pData, min((size_t)8, length));
        logPrint("[KEYBOARD FWD] %dB -> Conn %d", (int)length, targetConn);
    }
}

bool connectToServer();
bool connectToKeyboard();
void sendConfigResponse(const String& response);
void saveMouseToNvsLayout(String mac, String name);
void saveKeyboardToNvsLayout(String mac, String name);
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
            String nameLower = devName;
            nameLower.toLowerCase();

            bool hasHidService = advertisedDevice->haveServiceUUID() && advertisedDevice->isAdvertisingService(NimBLEUUID((uint16_t)0x1812));
            uint16_t appearance = advertisedDevice->haveAppearance() ? advertisedDevice->getAppearance() : 0;
            bool hasHidAppearance = (appearance == 0x03C1 || appearance == 0x03C2 || appearance == 0x03C0 || appearance == 0x03C3 || appearance == 0x03C4);
            bool hasHidName = (nameLower.indexOf("mouse") != -1 || nameLower.indexOf("keyboard") != -1 || 
                               nameLower.indexOf("keys") != -1 || nameLower.indexOf("master") != -1 || 
                               nameLower.indexOf("trackpad") != -1 || nameLower.indexOf("magic") != -1 ||
                               nameLower.indexOf("keychron") != -1 || nameLower.indexOf("naga") != -1 || 
                               nameLower.indexOf("basilisk") != -1);
            bool isTargetDevice = (targetMouseMac.length() > 0 && devMac == targetMouseMac) || 
                                 (targetKeyboardMac.length() > 0 && devMac == targetKeyboardMac);

            // Strict Filter: Only include genuine HID input peripherals (Mice, Keyboards, Trackpads)
            if (!hasHidService && !hasHidAppearance && !hasHidName && !isTargetDevice) {
                return; // Ignore smartphones, TVs, smart meters, beacons, etc.
            }

            String devType = "unknown";
            if (appearance == 0x03C2 || nameLower.indexOf("mouse") != -1 || nameLower.indexOf("master") != -1 || 
                nameLower.indexOf("naga") != -1 || nameLower.indexOf("basilisk") != -1 || devMac == targetMouseMac) {
                devType = "mouse";
            } else if (appearance == 0x03C1 || nameLower.indexOf("keyboard") != -1 || nameLower.indexOf("keys") != -1 || 
                       nameLower.indexOf("keychron") != -1 || devMac == targetKeyboardMac) {
                devType = "keyboard";
            } else {
                devType = "unknown";
            }

            JsonArray arr = scannedMiceDoc.as<JsonArray>();
            bool exists = false;
            for (JsonObject m : arr) {
                if (m["mac"].as<String>() == devMac) {
                    m["rssi"] = rssi;
                    m["type"] = devType;
                    // If real name arrives in subsequent Scan Response (SCAN_RSP), update it!
                    if (devName.length() > 0 && devName != "BLE Mouse" && devName != "BLE Keyboard" && devName != "HID Device") {
                        m["name"] = devName;
                    }
                    exists = true;
                    break;
                }
            }
            if (!exists) {
                JsonObject obj = arr.add<JsonObject>();
                obj["mac"] = devMac;
                String finalName = devName;
                if (finalName.length() == 0 || finalName == "Bluetooth Device") {
                    if (devMac == targetMouseMac && targetMouseName.length() > 0) {
                        finalName = targetMouseName;
                    } else if (devMac == targetKeyboardMac && targetKeyboardName.length() > 0) {
                        finalName = targetKeyboardName;
                    } else {
                        finalName = (devType == "mouse" ? "BLE Mouse" : (devType == "keyboard" ? "BLE Keyboard" : "HID Device"));
                    }
                }
                obj["name"] = finalName;
                obj["rssi"] = rssi;
                obj["type"] = devType;
            }
            return;
        }

        bool isLogitechMfg = false;
        if (advertisedDevice->haveManufacturerData()) {
            std::string mfg = advertisedDevice->getManufacturerData();
            if (mfg.length() >= 2) {
                uint8_t b0 = (uint8_t)mfg[0];
                uint8_t b1 = (uint8_t)mfg[1];
                if ((b0 == 0x6D && b1 == 0x04) || (b0 == 0x04 && b1 == 0x6D)) {
                    isLogitechMfg = true;
                }
            }
        }



        // 1. Mouse Check (Strict exact MAC match only)
        bool isKbMac = (targetKeyboardMac.length() > 0 && devMac == targetKeyboardMac);
        bool mouseMatch = false;

        if (!isKbMac && targetMouseMac.length() > 0 && devMac == targetMouseMac) {
            mouseMatch = true;
        }

        if (!mouseConnected && !isConnectingToMouse && mouseMatch) {
            isConnectingToMouse = true; // Set flag immediately to throttle multiple advertising packets
            logPrint("[BLE Scan] TARGET MOUSE MATCH! Connecting to %s (%s)", devName.c_str(), devMac.c_str());
            advDevice = new NimBLEAdvertisedDevice(*advertisedDevice);
            doConnectMouse = true;
        }

        // 2. Keyboard Check (Strict exact MAC match only)
        bool kbMatch = false;
        if (!mouseMatch && targetKeyboardMac.length() > 0 && devMac == targetKeyboardMac) {
            kbMatch = true;
        }

        if (!kbConnected && !isConnectingToKeyboard && kbMatch) {
            isConnectingToKeyboard = true; // Set flag immediately to throttle multiple advertising packets
            logPrint("[BLE Scan] TARGET KEYBOARD MATCH! Connecting to %s (%s)...", devName.c_str(), devMac.c_str());
            advKbDevice = new NimBLEAdvertisedDevice(*advertisedDevice);
            doConnectKeyboard = true;
        }
    }
};

// =========================================================================================
// ULTRA-FAST HOST RECONNECTION & LINK-LAYER SUBSYSTEM (OS-Level Speed Architecture)
// =========================================================================================
TaskHandle_t hostScanTaskHandle = NULL;

/**
 * @brief Background daemon maintaining active reconnection with bonded HID peripherals.
 * Implements 25ms high-duty-cycle channel hopping (Channels 37, 38, 39) to catch short
 * advertisement bursts (<20ms) from Logitech Easy-Switch keyboards and mice.
 * Uses FreeRTOS Task Notifications for 0ms instant wakeups upon peripheral disconnects.
 */
void startHostReconnectTask() {
    if (hostScanTaskHandle != NULL) return; // Daemon already active
    xTaskCreate([](void* param) {
        logPrint("[BLE Host] Host Reconnect Daemon started (Instant Wakeup Mode).");
        while (true) {
            bool needMouse = !mouseConnected;
            bool needKb = !kbConnected;

            if (!needMouse && !needKb) {
                // Both peripherals connected: Stop radio scanner to reserve 100% bandwidth for HID traffic.
                // Sleeps efficiently until woken immediately (0ms) by onDisconnect() task notification.
                NimBLEScan* pScan = NimBLEDevice::getScan();
                if (pScan && pScan->isScanning()) {
                    pScan->stop();
                }
                ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
                continue;
            }

            if (isConnectingToMouse || isConnectingToKeyboard || doConnectMouse || doConnectKeyboard) {
                NimBLEScan* pScan = NimBLEDevice::getScan();
                if (pScan && pScan->isScanning()) {
                    pScan->stop();
                }
                vTaskDelay(pdMS_TO_TICKS(50));
                continue;
            }

            if (!isScanningForMice) {
                NimBLEScan* pScan = NimBLEDevice::getScan();
                if (pScan && !pScan->isScanning()) {
                    pScan->setAdvertisedDeviceCallbacks(new ScanCallbacks(), false);
                    pScan->setActiveScan(true);
                    pScan->setInterval(40);  // 25ms interval
                    pScan->setWindow(40);    // 25ms window (100% continuous listening with rapid channel hops)
                    pScan->setDuplicateFilter(false);
                    pScan->start(0, false);  // Continuous non-blocking asynchronous scan
                }
            }

            // Quick 20ms task yield before next evaluation cycle
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        hostScanTaskHandle = NULL;
        vTaskDelete(NULL);
    }, "hostScanDaemon", 4096, NULL, 1, &hostScanTaskHandle);
}

// Callback for BLE Mouse Connection Status
class ClientCallbacks : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient* pClient) {
        logPrint("[BLE Host] Connected to mouse!");
        mouseConnected = true;
    }
    void onDisconnect(NimBLEClient* pClient) {
        logPrint("[BLE Host] Disconnected from mouse!");
        mouseConnected = false;
        isConnectingToMouse = false;
        // Instantly wake up the reconnect daemon without waiting for periodic timer tick
        if (hostScanTaskHandle != NULL) {
            xTaskNotifyGive(hostScanTaskHandle);
        }
    }
};

// Callback for BLE Keyboard Connection Status
class KeyboardClientCallbacks : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient* pClient) {
        logPrint("[BLE Host] Connected to Keyboard!");
        kbConnected = true;
    }
    void onDisconnect(NimBLEClient* pClient) {
        logPrint("[BLE Host] Disconnected from Keyboard!");
        kbConnected = false;
        isConnectingToKeyboard = false;
        // Instantly wake up the reconnect daemon without waiting for periodic timer tick
        if (hostScanTaskHandle != NULL) {
            xTaskNotifyGive(hostScanTaskHandle);
        }
    }
};

static bool kbGattInitialized = false;
static bool mouseGattInitialized = false;

/**
 * @brief Establishes Direct Link-Layer connection to the target bonded keyboard.
 * Replicates OS-level (Windows/macOS) connection speed by:
 * 1. Initializing connection parameters directly at 7.5ms (Connection Interval = 6).
 * 2. Avoiding manual scan stops that delay HCI packets.
 * 3. Preserving RAM GATT cache (deleteAttributes = false).
 * 4. Using asynchronous Write-Without-Response (response = false) for CCCD subscriptions.
 */
bool connectToKeyboard() {
    if (isScanningForMice) return false;
    if (targetKeyboardMac.length() == 0 && !advKbDevice) return false;

    if (!pKbClient) {
        pKbClient = NimBLEDevice::createClient();
        pKbClient->setClientCallbacks(new KeyboardClientCallbacks());
        pKbClient->setConnectTimeout(5);
    }

    if (pKbClient->isConnected()) {
        kbConnected = true;
        return true;
    }

    isConnectingToKeyboard = true;

    bool wasAdv = false;
    NimBLEScan* pScan = NimBLEDevice::getScan();
    if (pScan && pScan->isScanning()) {
        pScan->stop();
        int waitCount = 0;
        while (pScan->isScanning() && waitCount++ < 50) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    if (ble_gap_disc_active()) {
        ble_gap_disc_cancel();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (NimBLEDevice::getAdvertising() && NimBLEDevice::getAdvertising()->isAdvertising()) {
        wasAdv = true;
        NimBLEDevice::getAdvertising()->stop();
    }
    if (ble_gap_adv_active()) {
        wasAdv = true;
        ble_gap_adv_stop();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (ble_gap_conn_active()) {
        logPrint("[BLE Host] Lingering connection attempt detected, cancelling...");
        ble_gap_conn_cancel();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    logPrint("[BLE Host GAP Status] conn_active=%d, disc_active=%d, adv_active=%d",
             ble_gap_conn_active(), ble_gap_disc_active(), ble_gap_adv_active());

    // Direct Link-Layer Connection (Connects on first radio burst in <50ms)
    bool connRes = false;
    if (advKbDevice) {
        logPrint("[BLE Host] Direct Link-Layer Connecting to Keyboard: %s (Type: %d)...", 
                 advKbDevice->getAddress().toString().c_str(), advKbDevice->getAddressType());
        connRes = pKbClient->connect(advKbDevice, false);
        delete advKbDevice;
        advKbDevice = nullptr;
    } else if (targetKeyboardMac.length() > 0) {
        logPrint("[BLE Host] Direct Link-Layer Connecting to MAC: %s...", targetKeyboardMac.c_str());
        connRes = pKbClient->connect(NimBLEAddress(targetKeyboardMac.c_str(), BLE_ADDR_RANDOM), false);
        if (!connRes) {
            connRes = pKbClient->connect(NimBLEAddress(targetKeyboardMac.c_str(), BLE_ADDR_PUBLIC), false);
        }
    }

    if (!connRes) {
        int errCode = pKbClient ? pKbClient->getLastError() : -1;
        logPrint("[BLE Host] Keyboard connection attempt failed: rc=%d (%s)", 
                 errCode, NimBLEUtils::returnCodeToString(errCode));
        if (pKbClient) {
            NimBLEDevice::deleteClient(pKbClient);
            pKbClient = nullptr;
        }
        vTaskDelay(pdMS_TO_TICKS(1500));
        isConnectingToKeyboard = false;
        return false;
    }

    logPrint("[BLE Host] Keyboard connected! Securing link...");
    pKbClient->setConnectionParams(6, 12, 0, 500); // Enforce 7.5ms BLE stream latency
    if (!pKbClient->secureConnection()) {
        logPrint("[BLE Host] Initial secureConnection failed. Retrying in 100ms...");
        delay(100);
        if (!pKbClient->secureConnection()) {
            logPrint("[BLE Host] Secure connection retry failed. Proceeding with service discovery...");
        } else {
            logPrint("[BLE Host] Keyboard connection secured on retry!");
        }
    } else {
        logPrint("[BLE Host] Keyboard connection secured!");
    }

    // GATT Service Discovery & subscription
    NimBLERemoteService* pService = pKbClient->getService(hidServiceUUID);
    if (pService != nullptr) {
        std::vector<NimBLERemoteCharacteristic*>* pChars = pService->getCharacteristics(false);
        if (pChars == nullptr || pChars->empty()) {
            pChars = pService->getCharacteristics(true);
        }
        int subCount = 0;
        if (pChars != nullptr) {
            for (auto &pChar : *pChars) {
                if (pChar->canNotify()) {
                    // Async subscribe (response=false) completes in 0ms without blocking FreeRTOS queue
                    pChar->subscribe(true, keyboardNotifyCallback, false);
                    subCount++;
                }
            }
        }

        kbConnected = true;
        isConnectingToKeyboard = false;
        logPrint("[BLE Host] Keyboard FULLY CONNECTED & READY (%d active chars)!", subCount);
        checkAndResumeAdvertising();
        return true;
    } else {
        logPrint("[BLE Host] HID Service 0x1812 not found on Keyboard.");
        pKbClient->disconnect();
        isConnectingToKeyboard = false;
        checkAndResumeAdvertising();
        return false;
    }
}

bool connectToServer() {
    if (isScanningForMice) return false;
    if (targetMouseMac.length() == 0 && !advDevice) return false;

    if (!pClient) {
        pClient = NimBLEDevice::createClient();
        pClient->setClientCallbacks(new ClientCallbacks());
        pClient->setConnectTimeout(5);
    }

    if (pClient->isConnected()) {
        mouseConnected = true;
        return true;
    }

    isConnectingToMouse = true;

    if (!advDevice && targetMouseMac.length() > 0) {
        logPrint("[BLE Host] Performing targeted fast probe scan for mouse (%s)...", targetMouseMac.c_str());
        NimBLEScan* pScan = NimBLEDevice::getScan();
        if (pScan) {
            pScan->setActiveScan(true);
            pScan->setInterval(160);
            pScan->setWindow(160);
            NimBLEScanResults results = pScan->start(2, false);
            for (int i = 0; i < results.getCount(); i++) {
                NimBLEAdvertisedDevice dev = results.getDevice(i);
                String devMac = dev.getAddress().toString().c_str();
                devMac.toLowerCase();
                String devName = dev.getName().c_str();
                if (devMac == targetMouseMac) {
                    advDevice = new NimBLEAdvertisedDevice(dev);
                    logPrint("[BLE Host] Fast probe scan found mouse: %s (name: %s)!", devMac.c_str(), devName.c_str());
                    break;
                }
            }
            pScan->clearResults();
        }
    }

    bool wasAdv = false;
    NimBLEScan* pScan = NimBLEDevice::getScan();
    if (pScan && pScan->isScanning()) {
        pScan->stop();
        int waitCount = 0;
        while (pScan->isScanning() && waitCount++ < 50) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    if (ble_gap_disc_active()) {
        ble_gap_disc_cancel();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (NimBLEDevice::getAdvertising() && NimBLEDevice::getAdvertising()->isAdvertising()) {
        wasAdv = true;
        NimBLEDevice::getAdvertising()->stop();
    }
    if (ble_gap_adv_active()) {
        wasAdv = true;
        ble_gap_adv_stop();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (ble_gap_conn_active()) {
        logPrint("[BLE Host] Lingering connection attempt detected, cancelling...");
        ble_gap_conn_cancel();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    logPrint("[BLE Host GAP Status] conn_active=%d, disc_active=%d, adv_active=%d",
             ble_gap_conn_active(), ble_gap_disc_active(), ble_gap_adv_active());

    bool connRes = false;

    if (advDevice) {
        logPrint("[BLE Host] Direct Link-Layer Connecting to Mouse: %s (Type: %d)...", 
                 advDevice->getAddress().toString().c_str(), advDevice->getAddressType());
        connRes = pClient->connect(advDevice, false);
        delete advDevice;
        advDevice = nullptr;
    } else if (targetMouseMac.length() > 0) {
        logPrint("[BLE Host] Direct Link-Layer Connecting to MAC: %s...", targetMouseMac.c_str());
        connRes = pClient->connect(NimBLEAddress(targetMouseMac.c_str(), BLE_ADDR_RANDOM), false);
        if (!connRes) {
            connRes = pClient->connect(NimBLEAddress(targetMouseMac.c_str(), BLE_ADDR_PUBLIC), false);
        }
    }

    if (!connRes) {
        int errCode = pClient ? pClient->getLastError() : -1;
        logPrint("[BLE Host] Connection attempt failed: rc=%d (%s)", 
                 errCode, NimBLEUtils::returnCodeToString(errCode));
        if (pClient) {
            NimBLEDevice::deleteClient(pClient);
            pClient = nullptr;
        }
        vTaskDelay(pdMS_TO_TICKS(1500));
        isConnectingToMouse = false;
        if (!kbConnected || !mouseConnected) startHostReconnectTask();
        return false;
    }

    logPrint("[BLE Host] Connected! Securing connection (Pairing)...");
    pClient->setConnectionParams(6, 12, 0, 500);
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
                    pChar->subscribe(true, notifyCallback, false);
                    logPrint("[BLE Host] Subscribed to HID report!");
                }
            }
        }
    } else {
        pClient->disconnect();
        isConnectingToMouse = false;
        checkAndResumeAdvertising();
        if (!kbConnected || !mouseConnected) startHostReconnectTask();
        return false;
    }
    mouseConnected = true;
    isConnectingToMouse = false;
    checkAndResumeAdvertising();
    if (!kbConnected) startHostReconnectTask();
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
  targetKeyboardMac = "";
  targetKeyboardName = "";
  if (json.length() > 2) {
    JsonDocument doc;
    if (!deserializeJson(doc, json) && doc.is<JsonObject>()) {
      targetMouseMac = doc["mouseMac"] | "";
      targetMouseName = doc["mouseName"] | "";
      targetKeyboardMac = doc["keyboardMac"] | "";
      targetKeyboardName = doc["keyboardName"] | "";
      if (targetKeyboardMac.length() > 0 && targetKeyboardMac == targetMouseMac) {
        targetKeyboardMac = "";
        targetKeyboardName = "";
      }
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
  String json = loadLayoutJsonFromNVS();
  JsonDocument doc;
  if (deserializeJson(doc, json) || !doc.is<JsonObject>()) {
    initDefaultConfigDoc(doc);
  }

  targetMouseMac = mac;
  targetMouseName = name;
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

void saveKeyboardToNvsLayout(String mac, String name) {
  String json = loadLayoutJsonFromNVS();
  JsonDocument doc;
  if (deserializeJson(doc, json) || !doc.is<JsonObject>()) {
    initDefaultConfigDoc(doc);
  }

  targetKeyboardMac = mac;
  targetKeyboardName = name;
  doc["keyboardMac"] = targetKeyboardMac;
  doc["keyboardName"] = targetKeyboardName;

  String unifiedJson;
  serializeJson(doc, unifiedJson);

  preferences.begin(NVS_NAMESPACE, false);
  preferences.remove(NVS_KEY_LAYOUT);
  size_t bytesWritten = preferences.putBytes(NVS_KEY_LAYOUT, unifiedJson.c_str(), unifiedJson.length() + 1);
  preferences.end();

  if (bytesWritten > 0) {
    logPrint("[NVS] Persisted keyboard (%s, '%s') in unified JSON layout (%u bytes)!", targetKeyboardMac.c_str(), targetKeyboardName.c_str(), bytesWritten);
  } else {
    logPrint("[NVS ERROR] Failed to save keyboard to layout (putBytes returned 0)!");
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
          monitors[monitorCount].os = repo["os"] | OS_WINDOWS;
          monitors[monitorCount].scale = repo["scale"] | 100;
          monitors[monitorCount].isPrimary = repo["isPrimary"] | false;
          monitorCount++;
        }
      }
      // Calculate distinct PCs from the loaded layout screens
      int pcCount = 0;
      String uniquePcMacs[MAX_SUPPORTED_KVM_CLIENTS];
      for (int i = 0; i < monitorCount; i++) {
        String mMac = monitors[i].mac;
        mMac.toLowerCase();
        mMac.trim();
        if (mMac.length() > 0) {
          bool found = false;
          for (int p = 0; p < pcCount; p++) {
            if (uniquePcMacs[p].equalsIgnoreCase(mMac)) {
              found = true;
              break;
            }
          }
          if (!found && pcCount < MAX_SUPPORTED_KVM_CLIENTS) {
            uniquePcMacs[pcCount++] = mMac;
          }
        }
      }

      // Also include doc["clients"] if present
      if (doc["clients"].is<JsonArray>()) {
        for (JsonObject client : doc["clients"].as<JsonArray>()) {
          String cMac = client["mac"] | "";
          cMac.toLowerCase();
          cMac.trim();
          if (cMac.length() > 0) {
            bool found = false;
            for (int p = 0; p < pcCount; p++) {
              if (uniquePcMacs[p].equalsIgnoreCase(cMac)) {
                found = true;
                break;
              }
            }
            if (!found && pcCount < MAX_SUPPORTED_KVM_CLIENTS) {
              uniquePcMacs[pcCount++] = cMac;
            }
          }
        }
      }

      // Support up to MAX_SUPPORTED_KVM_CLIENTS (6 PCs)
      maxKvmClients = MAX_SUPPORTED_KVM_CLIENTS;

      // Populate / update kvmClients array while preserving existing active connection handles
      if (doc["clients"].is<JsonArray>()) {
        int clientCount = 0;
        for (JsonObject client : doc["clients"].as<JsonArray>()) {
          if (clientCount >= maxKvmClients) break;
          String mac = client["mac"] | "";
          mac.toLowerCase();
          mac.trim();
          if (mac.length() > 0) {
            bool alreadyConnected = false;
            uint16_t existingConn = BLE_HS_CONN_HANDLE_NONE;
            for (int k = 0; k < MAX_SUPPORTED_KVM_CLIENTS; k++) {
              if (kvmClients[k].mac.equalsIgnoreCase(mac) && kvmClients[k].active) {
                alreadyConnected = true;
                existingConn = kvmClients[k].conn_id;
                break;
              }
            }
            kvmClients[clientCount].mac = mac;
            kvmClients[clientCount].name = client["name"] | "Unknown PC";
            kvmClients[clientCount].conn_id = existingConn;
            kvmClients[clientCount].active = alreadyConnected;
            clientCount++;
          }
        }
      }

      logPrint("Loaded %d monitors, %d KVM PC clients (maxKvmClients = %d) from NVS. Mouse: %s (%s) | Keyboard: %s (%s)",
               monitorCount, pcCount, maxKvmClients, targetMouseMac.c_str(), targetMouseName.c_str(), targetKeyboardMac.c_str(), targetKeyboardName.c_str());
    }
  }
  checkAndResumeAdvertising();
}

void saveConfiguration(const String& jsonString) {
  JsonDocument doc;
  String finalJson = jsonString;

  if (!deserializeJson(doc, jsonString)) {
    String mac = doc["mouseMac"] | "";
    if (mac.length() > 0) {
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

    String kbMac = doc["keyboardMac"] | "";
    if (kbMac.length() > 0) {
      targetKeyboardMac = kbMac;
    } else if (targetKeyboardMac.length() > 0) {
      doc["keyboardMac"] = targetKeyboardMac;
    }

    String kbName = doc["keyboardName"] | "";
    if (kbName.length() > 0) {
      kbName.trim();
      targetKeyboardName = kbName;
    } else if (targetKeyboardName.length() > 0) {
      doc["keyboardName"] = targetKeyboardName;
    }

    serializeJson(doc, finalJson);
    logPrint("[NVS] Persisted Mouse '%s' and Keyboard '%s' in unified JSON layout", targetMouseMac.c_str(), targetKeyboardMac.c_str());
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

void processCommand(String input, bool isBleSource = false) {
  input.trim();
  if (input.length() == 0) return;

  lastConfigActivityTime = millis();

  // Web Bluetooth Authorization Check (Challenge-Response SHA-256)
  if (isBleSource) {
    if (input.equalsIgnoreCase("GET_CHALLENGE") || input.equalsIgnoreCase("AUTH_CHALLENGE")) {
      char nonceBuf[17];
      uint32_t r1 = esp_random();
      uint32_t r2 = esp_random();
      snprintf(nonceBuf, sizeof(nonceBuf), "%08lx%08lx", (unsigned long)r1, (unsigned long)r2);
      currentAuthNonce = String(nonceBuf);
      logPrint("[BLE AUTH] Issued new Challenge Nonce: %s", currentAuthNonce.c_str());
      sendConfigResponse("CHALLENGE " + currentAuthNonce);
      return;
    }

    if (input.startsWith("AUTH_RESPONSE ") || input.startsWith("AUTH_HASH ")) {
      String clientHash = input.substring(input.indexOf(' ') + 1);
      clientHash.trim();
      clientHash.toLowerCase();

      if (currentAuthNonce.length() > 0) {
        String expectedHash = calculateSha256(String(WEB_BLE_AUTH_PASSPHRASE) + ":" + currentAuthNonce);
        expectedHash.toLowerCase();

        if (String(WEB_BLE_AUTH_PASSPHRASE).length() == 0 || clientHash.equals(expectedHash)) {
          isWebBleAuthenticated = true;
          currentAuthNonce = ""; // Invalidate nonce immediately to prevent replay attacks
          logPrint("[BLE AUTH] Challenge-Response SHA-256 verified successfully!");
          sendConfigResponse("OK_AUTH " WEB_BLE_AUTH_PASSPHRASE);
          return;
        } else {
          isWebBleAuthenticated = false;
          currentAuthNonce = "";
          logPrint("[BLE AUTH ERROR] Signature verification failed (Received: %s, Expected: %s)", clientHash.c_str(), expectedHash.c_str());
          sendConfigResponse("ERROR_AUTH Invalid signature");
          return;
        }
      } else {
        logPrint("[BLE AUTH ERROR] Received AUTH_RESPONSE without active challenge nonce");
        sendConfigResponse("ERROR_AUTH No active challenge. Send 'GET_CHALLENGE'");
        return;
      }
    }

    if (input.equalsIgnoreCase("AUTH_STATUS")) {
      sendConfigResponse(isWebBleAuthenticated ? "AUTH_OK" : "AUTH_REQUIRED");
      return;
    }

    if (!isWebBleAuthenticated) {
      logPrint("[BLE AUTH] Rejected unauthorized command '%s'. Authentication required.", input.c_str());
      sendConfigResponse("ERROR_UNAUTHORIZED Authentication required. Request challenge via 'GET_CHALLENGE'");
      return;
    }
  }

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

    // Include bound mouse & keyboard in unified JSON payload
    doc["mouseMac"] = targetMouseMac;
    doc["mouseName"] = targetMouseName.length() > 0 ? targetMouseName : (targetMouseMac.length() > 0 ? "BLE Mouse" : "");
    doc["keyboardMac"] = targetKeyboardMac;
    doc["keyboardName"] = targetKeyboardName.length() > 0 ? targetKeyboardName : (targetKeyboardMac.length() > 0 ? "BLE Keyboard" : "");

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
    for (int i = 0; i < maxKvmClients; i++) {
      if (kvmClients[i].active && kvmClients[i].mac.length() > 0) {
        String activeMac = kvmClients[i].mac;

        bool exists = false;
        for (JsonObject c : clientsArr) {
          String cMac = c["mac"] | "";
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
  } else if (input == "SCAN_MICE" || input == "SCAN_KEYBOARDS" || input == "SCAN_DEVICES") {
    scannedMiceDoc.clear();
    scannedMiceDoc.to<JsonArray>();

    isScanningForMice = true;

    if (pClient && pClient->isConnected()) {
      pClient->disconnect();
    }
    if (pKbClient && pKbClient->isConnected()) {
      pKbClient->disconnect();
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

      logPrint("[BLE Scan] Starting 5-second active discovery scan for devices...");
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

    if (pClient && pClient->isConnected()) {
      pClient->disconnect();
    }
    if (targetMouseMac.length() > 0) {
      doConnectMouse = true;
    }
  } else if (input == "UNBIND_MOUSE") {
    saveMouseToNvsLayout("", "");
    if (pClient && pClient->isConnected()) {
      pClient->disconnect();
    }
    sendConfigResponse("OK_UNBIND_MOUSE");
  } else if (input == "GET_TARGET_MOUSE") {
    sendConfigResponse("TARGET_MOUSE " + targetMouseMac);
  } else if (input.startsWith("BIND_KEYBOARD ")) {
    String param = input.substring(14);
    param.trim();
    String mac = param;
    String name = "Logitech MX Keys S";
    int spaceIdx = param.indexOf(' ');
    if (spaceIdx != -1) {
      mac = param.substring(0, spaceIdx);
      name = param.substring(spaceIdx + 1);
      name.trim();
    }
    saveKeyboardToNvsLayout(mac, name);
    sendConfigResponse("OK_BIND_KEYBOARD " + targetKeyboardMac);

    if (pKbClient && pKbClient->isConnected()) {
      pKbClient->disconnect();
    }
    if (targetKeyboardMac.length() > 0) {
      doConnectKeyboard = true;
    }
  } else if (input == "UNBIND_KEYBOARD") {
    saveKeyboardToNvsLayout("", "");
    if (pKbClient && pKbClient->isConnected()) {
      pKbClient->disconnect();
    }
    sendConfigResponse("OK_UNBIND_KEYBOARD");
  } else if (input == "GET_TARGET_KEYBOARD") {
    sendConfigResponse("TARGET_KEYBOARD " + targetKeyboardMac);
  } else if (input == "DUMP_FLASH") {
    String json = loadLayoutJsonFromNVS();
    logPrint("--- [NVS FLASH DUMP] ---");
    logPrint("Flash layout string length: %d bytes", json.length());
    logPrint("Flash Mouse: %s, Keyboard: %s", targetMouseMac.c_str(), targetKeyboardMac.c_str());
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
    customMac[5] += 27; // Increment to present fresh identity to PCs so OS prompts for 6-digit PIN entry
    esp_base_mac_addr_set(customMac);
    logPrint("[BLE] Custom Base MAC: %02X:%02X:%02X:%02X:%02X:%02X (PC Pairing PIN: %06lu)",
             customMac[0], customMac[1], customMac[2], customMac[3], customMac[4], customMac[5],
             (unsigned long)BLE_PAIRING_PIN);

    NimBLEDevice::init(BLE_DEVICE_NAME);
    NimBLEDevice::setMTU(512);
    NimBLEDevice::setSecurityAuth(true, true, true); // (bonding=true, mitm=true -> Enforces 6-digit PIN passkey, sc=true)
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY); // Display Only: Tells PC that ESP32 has display, prompting user for PIN
    NimBLEDevice::setSecurityPasskey(BLE_PAIRING_PIN);
    NimBLEDevice::setSecurityCallbacks(new SecurityCallbacks());
    NimBLEDevice::setSecurityInitKey(BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);
    NimBLEDevice::setSecurityRespKey(BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);
    
    int numBonds = NimBLEDevice::getNumBonds();
    logPrint("[BLE NVS BONDS] Saved bonded devices count: %d", numBonds);
    if (numBonds >= 14) {
        logPrint("[BLE NVS BONDS] Bond table full (%d/16). Clearing stale bonds to ensure reliable pairing...", numBonds);
        NimBLEDevice::deleteAllBonds();
        numBonds = 0;
    }
    for (int i = 0; i < numBonds; i++) {
        NimBLEAddress bondAddr = NimBLEDevice::getBondedAddress(i);
        logPrint("  -> Bonded Device #%d: MAC %s", i + 1, bondAddr.toString().c_str());
    }
    
    // Setup BLE Server (Peripheral)
    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());
    hidDevice = new NimBLEHIDDevice(pServer);
    keyboardInputChar = hidDevice->inputReport(1); // Report ID 1: Standard Keyboard (6KRO)
    keyboardOutputChar = hidDevice->outputReport(1); // Report ID 1: Keyboard LEDs (Output)
    inputChar = hidDevice->inputReport(2);         // Report ID 2: Relative Mouse
    absInputChar = hidDevice->inputReport(3);      // Report ID 3: Absolute Mouse / Pointer
    macAbsInputChar = hidDevice->inputReport(5);   // Report ID 5: macOS / iPadOS Digitizer
    mediaInputChar = hidDevice->inputReport(4);    // Report ID 4: Media / Consumer Keys
    
    hidDevice->manufacturer()->setValue("Logitech");
    hidDevice->pnp(0x01, 0x046d, 0xc52b, 0x0100);
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
    pAdvertising->setAppearance(0x03C0); // HID Generic / Combo (Mouse + Keyboard)
    pAdvertising->addServiceUUID(hidDevice->hidService()->getUUID());
    pAdvertising->addServiceUUID(CONFIG_SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->start();
    logPrint("[BLE Server] Advertising HID Combo (Mouse+Keyboard) & ESP32 KVM Server Config Service...");

    // If target devices are bound, start persistent background reconnect task
    startHostReconnectTask();
}

void loop() {
    // Continuous Advertising Watchdog: ensures ESP32 is discoverable without log spam
    static uint32_t lastAdvCheck = 0;
    if (millis() - lastAdvCheck > 2000) {
        lastAdvCheck = millis();
        checkAndResumeAdvertising();
    }

    if (doSaveConfig) {
        executePendingSave();
    }

    if (!bleCmdQueue.empty()) {
        String cmd = bleCmdQueue.front();
        bleCmdQueue.erase(bleCmdQueue.begin());
        logPrint("[BLE RX CMD]: %s", cmd.c_str());
        processCommand(cmd, true); // isBleSource = true (requires WEB_BLE_AUTH_PASSPHRASE)
    }

    if (doConnectMouse) {
        doConnectMouse = false;
        xTaskCreate([](void* param) {
            connectToServer();
            vTaskDelete(NULL);
        }, "mouseConnTask", 4096, NULL, 5, NULL);
    }

    if (doConnectKeyboard) {
        doConnectKeyboard = false;
        xTaskCreate([](void* param) {
            connectToKeyboard();
            vTaskDelete(NULL);
        }, "kbConnTask", 4096, NULL, 5, NULL);
    }

    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        if (input.length() > 0) {
            logPrint("[UART RX CMD]: %s", input.c_str());
            processCommand(input, false);
        }
    }

#if CONFIG_IDF_TARGET_ESP32S3
    if (USBSerial.available()) {
        String input = USBSerial.readStringUntil('\n');
        input.trim();
        if (input.length() > 0) {
            logPrint("[USB CDC RX CMD]: %s", input.c_str());
            processCommand(input, false);
        }
    }
#endif
}
