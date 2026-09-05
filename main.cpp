#include <Arduino.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <NimBLEDevice.h>
#include <NimBLEHIDDevice.h>
#include "nimble/nimble/host/include/host/ble_gap.h"
#include "nimble/nimble/host/services/gatt/include/services/gatt/ble_svc_gatt.h"
#include <esp_mac.h>
#include <mbedtls/sha256.h>

#include "logi_bolt.h"

#define BLE_DEVICE_NAME "ESP32 KVM Combo"

Preferences preferences;

// NVS Flash Storage Constants
const char* NVS_NAMESPACE = "kvm_config";
const char* NVS_KEY_ACT_LAYOUT_ID = "actLayoutId";
const char* NVS_KEY_TOTAL_LAYOUTS = "totalLayouts";
const char* NVS_KEY_LAYOUTS       = "layouts";
const char* NVS_KEY_CLIENTS       = "clients";
const char* NVS_KEY_MOUSE_MAC     = "mouseMac";
const char* NVS_KEY_MOUSE_NAME    = "mouseName";
const char* NVS_KEY_KB_MAC        = "keyboardMac";
const char* NVS_KEY_KB_NAME       = "keyboardName";

enum os {
    OS_WINDOWS = 0,
    OS_MAC = 1
};
static String firstConnectedPcMac = "";
static bool isCalibrated = false;
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
    int keepAlive = 0;
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
static NimBLERemoteCharacteristic* pKbLedChar = nullptr;
static NimBLERemoteCharacteristic* pKbBootLedChar = nullptr;

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
  bool isTurbo;
  bool isHandshaking;
  uint32_t handshakeStartMs;
  uint8_t ledState; // Saved keyboard LED state (Caps/Num/Scroll) for this PC
};
#define MAX_SUPPORTED_KVM_CLIENTS 6 // 6 PCs + 1 Mouse + 1 Keyboard + 1 Web = 9 max NimBLE connections
KVMClient kvmClients[MAX_SUPPORTED_KVM_CLIENTS];
int maxKvmClients = 3; // Dynamic variable based on number of PCs in current configuration

// Dedicated tracking for non-KVM / Web Bluetooth connections
struct NonKvmClient {
  uint16_t conn_id = BLE_HS_CONN_HANDLE_NONE;
  String mac = "";
  uint32_t connectedTimeMs = 0;
  bool isWebConfig = false;
};
#define MAX_NON_KVM_CLIENTS 2
static NonKvmClient nonKvmClients[MAX_NON_KVM_CLIENTS];

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

    // --- REPORT ID 2: Native 12-bit High-Resolution Mouse (Logitech Darkfield Standard) ---
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
    0x16, 0x01, 0xF8,  //     Logical Minimum (-2047)
    0x26, 0xFF, 0x07,  //     Logical Maximum (2047)
    0x75, 0x0C,        //     Report Size (12)
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

bool isAnyPcHandshaking() {
    uint32_t now = millis();
    for (int i = 0; i < MAX_SUPPORTED_KVM_CLIENTS; i++) {
        if (kvmClients[i].active && kvmClients[i].isHandshaking) {
            // Failsafe Watchdog: if handshake has taken > 2500ms, unlock it to prevent deadlock
            if (now - kvmClients[i].handshakeStartMs > 2500) {
                kvmClients[i].isHandshaking = false;
                logPrint("[BLE Watchdog] Handshake timeout for %s -> Failsafe unlocked", kvmClients[i].mac.c_str());
            } else {
                return true;
            }
        }
    }
    return false;
}

bool isMacInActiveLayout(const String& mac) {
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
    for (int i = 0; i < MAX_NON_KVM_CLIENTS; i++) {
        if (nonKvmClients[i].conn_id != BLE_HS_CONN_HANDLE_NONE) activeCount++;
    }

    // Hardware ACL budget: 8 max simultaneous connections in ESP32-S3 controller (10 activities).
    int maxAllowedPcConnections = max(2, 8 - (targetMouseMac.length() > 0 ? 1 : 0) - (targetKeyboardMac.length() > 0 ? 1 : 0));

    int layoutPcs = getActiveLayoutPcCount();

    // Keep advertising active as long as hardware handles are available (up to maxAllowedPcConnections)
    // so Web Bluetooth browser interface can discover and connect to ESP32 at any time!
    if (activeCount < maxAllowedPcConnections) {
        if (NimBLEDevice::getAdvertising() && !NimBLEDevice::getAdvertising()->isAdvertising()) {
            logPrint("[BLE Server] Advertising active (PCs: %d/%d [Layout: %d] | Mouse: %d | KB: %d)", 
                     activeCount, maxAllowedPcConnections, layoutPcs,
                     mouseConnected ? 1 : 0, kbConnected ? 1 : 0);
            NimBLEDevice::getAdvertising()->start();
        }
    } else {
        if (NimBLEDevice::getAdvertising() && NimBLEDevice::getAdvertising()->isAdvertising()) {
            logPrint("[BLE Server] Advertising paused (All %d connection slots full)", activeCount);
            NimBLEDevice::getAdvertising()->stop();
        }
    }
}

void markClientAsWebConfig(uint16_t connHandle) {
    for (int i = 0; i < MAX_NON_KVM_CLIENTS; i++) {
        if (nonKvmClients[i].conn_id != BLE_HS_CONN_HANDLE_NONE && 
            (nonKvmClients[i].conn_id == connHandle || connHandle == BLE_HS_CONN_HANDLE_NONE)) {
            if (!nonKvmClients[i].isWebConfig) {
                nonKvmClients[i].isWebConfig = true;
                logPrint("[BLE Server] Client %s (conn: %d) identified as Web Configurator (Grace Period cancelled)",
                         nonKvmClients[i].mac.c_str(), nonKvmClients[i].conn_id);
            }
        }
    }
}

void checkWebGracePeriod() {
    if (monitorCount == 0) return; // Allow unconfigured setup
    NimBLEServer* pServer = NimBLEDevice::getServer();
    if (!pServer) return;

    uint32_t now = millis();
    for (int i = 0; i < MAX_NON_KVM_CLIENTS; i++) {
        if (nonKvmClients[i].conn_id != BLE_HS_CONN_HANDLE_NONE) {
            uint32_t elapsed = now - nonKvmClients[i].connectedTimeMs;
            if (!nonKvmClients[i].isWebConfig && elapsed >= 3000) {
                logPrint("[BLE Server] ⛔ REJECTED: PC %s is NOT in active layout and no Web Config activity after 3s! Disconnecting...",
                         nonKvmClients[i].mac.c_str());
                pServer->disconnect(nonKvmClients[i].conn_id);
                nonKvmClients[i].conn_id = BLE_HS_CONN_HANDLE_NONE;
                nonKvmClients[i].mac = "";
                nonKvmClients[i].connectedTimeMs = 0;
                nonKvmClients[i].isWebConfig = false;
            }
        }
    }
}

void checkAndLogPhyStatus(uint16_t connHandle, const char* deviceLabel);

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
        if (desc->sec_state.encrypted) {
            ble_gap_set_prefered_le_phy(desc->conn_handle, BLE_GAP_LE_PHY_2M_MASK | BLE_GAP_LE_PHY_1M_MASK, BLE_GAP_LE_PHY_2M_MASK | BLE_GAP_LE_PHY_1M_MASK, 0);
            ble_svc_gatt_changed(0x0001, 0xffff);
        }
        checkAndResumeAdvertising();
    }
};

// --- Asynchronous BLE Link Metrics & PHY Status Logger ---
void checkAndLogPhyStatus(uint16_t connHandle, const char* deviceLabel) {
    if (connHandle == BLE_HS_CONN_HANDLE_NONE) return;
    struct PhyCheckParams {
        uint16_t handle;
        char label[32];
    };
    PhyCheckParams* params = new PhyCheckParams();
    params->handle = connHandle;
    strncpy(params->label, deviceLabel, sizeof(params->label) - 1);
    params->label[sizeof(params->label) - 1] = '\0';

    xTaskCreate([](void* param) {
        PhyCheckParams* p = (PhyCheckParams*)param;
        vTaskDelay(pdMS_TO_TICKS(1500));
        uint8_t txPhy = 0, rxPhy = 0;
        ble_gap_read_le_phy(p->handle, &txPhy, &rxPhy);
        if (txPhy != 2 && rxPhy != 2) {
            vTaskDelay(pdMS_TO_TICKS(1200));
            ble_gap_read_le_phy(p->handle, &txPhy, &rxPhy);
        }
        ble_gap_conn_desc desc;
        if (ble_gap_conn_find(p->handle, &desc) == 0) {
            logPrint("[BLE LINK METRICS] %s (conn: %d) -> Itvl: %.2f ms (itvl: %d) | Latency: %d | Timeout: %d ms | TX: %s | RX: %s",
                     p->label, p->handle,
                     desc.conn_itvl * 1.25f, desc.conn_itvl,
                     desc.conn_latency,
                     desc.supervision_timeout * 10,
                     txPhy == 2 ? "2M (⚡)" : (txPhy == 1 ? "1M" : "CODED"),
                     rxPhy == 2 ? "2M (⚡)" : (rxPhy == 1 ? "1M" : "CODED"));
        }
        delete p;
        vTaskDelete(NULL);
    }, "phyCheckTask", 4096, params, 1, NULL);
}

/**
 * @brief Dynamic Link Optimization Subsystem ("Active Turbo + Background Standby")
 * Elevates the currently active PC (where the mouse cursor is located) to high-speed Turbo profile (11.25ms - 15.0ms, latency 0),
 * while shifting all background PCs to Standby profile (60ms - 80ms, latency 4) to free 90% radio bandwidth for Mouse + Active PC.
 */
void updateKvmPowerAndRateProfiles(String activeMac = "", bool force = false) {
    NimBLEServer* pServer = NimBLEDevice::getServer();
    if (!pServer) return;

    if (activeMac.length() == 0 && monitorCount > 0 && currentMonitorIndex < monitorCount) {
        activeMac = monitors[currentMonitorIndex].mac;
    }

    bool boltActive = logi_bolt_is_mouse_connected();
    bool isMouseActive = mouseConnected || boltActive;

    for (int i = 0; i < MAX_SUPPORTED_KVM_CLIENTS; i++) {
        if (kvmClients[i].active && kvmClients[i].conn_id != BLE_HS_CONN_HANDLE_NONE) {
            String clientMac = kvmClients[i].mac;
            // Detect OS type from monitor configuration
            int clientOs = OS_WINDOWS;
            for (int m = 0; m < monitorCount; m++) {
                if (monitors[m].mac.equals(clientMac)) {
                    clientOs = monitors[m].os;
                    break;
                }
            }

            // DUAL-MODE SMART POWER & BANDWIDTH ALLOCATION:
            // 1. With Logi Bolt (USB): Radio has NO BLE mouse traffic -> ALL connected PCs stay in PERMANENT TURBO (0ms switch lag!)
            // 2. With BLE Mouse: Active PC gets TURBO, background PCs get STANDBY (frees 90% radio bandwidth for BLE mouse!)
            bool isCurrentActivePc = (activeMac.length() > 0 && clientMac.equals(activeMac));
            bool shouldBeTurbo = isMouseActive && (boltActive || isCurrentActivePc);

            bool wasTurbo = kvmClients[i].isTurbo;
            if (force || wasTurbo != shouldBeTurbo) {
                kvmClients[i].isTurbo = shouldBeTurbo;
                ble_gap_conn_desc desc;
                bool hasDesc = (ble_gap_conn_find(kvmClients[i].conn_id, &desc) == 0);

                if (shouldBeTurbo) {
                    if (clientOs == OS_MAC) {
                        if (hasDesc && desc.conn_latency == 0 && desc.conn_itvl == 12) {
                            continue; // Already in Apple Turbo!
                        }
                        // macOS ACTIVE TURBO: Apple strictly mandates itvl: 12 (15.00ms). Latency 0 gives 0ms slave latency!
                        pServer->updateConnParams(kvmClients[i].conn_id, 12, 12, 0, 216);
                        logPrint("[BLE Server] Enforcing macOS PERMANENT TURBO for PC: %s (Target: 15.00 ms, Latency: 0, ⚡)", kvmClients[i].mac.c_str());
                    } else {
                        if (hasDesc && desc.conn_itvl <= 8 && (desc.conn_latency == 0 || (wasTurbo && (boltActive || isCurrentActivePc)))) {
                            continue; // Already in Windows Turbo!
                        }
                        // Windows ACTIVE TURBO: 7.50ms..10.00ms (itvl 6..8), latency 0, timeout 6000ms (Ultra-fast 100..133Hz)
                        pServer->updateConnParams(kvmClients[i].conn_id, 6, 8, 0, 600);
                        logPrint("[BLE Server] Enforcing Windows PERMANENT TURBO for PC: %s (Target: 7.50..10.00 ms, Latency: 0, ⚡)", kvmClients[i].mac.c_str());
                    }
                } else {
                    // Safe idle standby only when no mouse is active at all
                    if (clientOs == OS_MAC) {
                        pServer->updateConnParams(kvmClients[i].conn_id, 12, 12, 4, 216);
                        logPrint("[BLE Server] Idle Standby for macOS PC: %s (Latency: 4, 💤)", kvmClients[i].mac.c_str());
                    } else {
                        pServer->updateConnParams(kvmClients[i].conn_id, 6, 8, 4, 600);
                        logPrint("[BLE Server] Idle Standby for Windows PC: %s (Latency: 4, 💤)", kvmClients[i].mac.c_str());
                    }
                }
            }
        }
    }
}

void calibrateFirstConnectedPcToCenter(String targetMac);
static TaskHandle_t bootCalibTaskHandle = NULL;

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

static bool s_capsLockLedState = false;

void syncPhysicalKeyboardLedsForPc(const String& targetMac) {
    if (targetMac.length() == 0) return;
    uint8_t targetLeds = 0;
    for (int k = 0; k < MAX_SUPPORTED_KVM_CLIENTS; k++) {
        if (kvmClients[k].active && kvmClients[k].mac.equalsIgnoreCase(targetMac)) {
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
            if (kvmClients[i].active && kvmClients[i].mac.equalsIgnoreCase(activeMac)) {
                kvmClients[i].ledState ^= 0x02; // Toggle CapsLock bit for currently active PC
                break;
            }
        }
        syncPhysicalKeyboardLedsForPc(activeMac);
    }
    s_lastCapsLockPressed = currentCapsLockPressed;
}

class KeyboardOutputCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pCharacteristic, ble_gap_conn_desc* desc) override {
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
            bool isActivePc = (senderMac.length() > 0 && senderMac.equalsIgnoreCase(activeMac));
            
            logPrint("[KEYBOARD LED] PC %s (conn %d) sent LED state: 0x%02X (Caps: %d, Active: %s)",
                     senderMac.c_str(), connHandle, leds, (leds & 0x02) ? 1 : 0, isActivePc ? "YES" : "NO");
            
            if (isActivePc || senderMac.length() == 0) {
                syncPhysicalKeyboardLedsForPc(activeMac.length() > 0 ? activeMac : senderMac);
            }
        }
    }
};

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnUpdate(NimBLEServer* pServer, ble_gap_conn_desc* desc) override {
        String peerMac = NimBLEAddress(desc->peer_ota_addr).toString().c_str();
        peerMac.toLowerCase();
        peerMac.trim();
        const char* mode = (desc->conn_itvl <= 16 && desc->conn_latency == 0) ? "ACTIVE TURBO (⚡)" : "BACKGROUND STANDBY (💤)";
        logPrint("[BLE Server] Connection Metrics Applied -> PC: %s (conn: %d) | Itvl: %.2f ms (itvl: %d) | Latency: %d | Timeout: %d ms -> %s",
                 peerMac.c_str(), desc->conn_handle,
                 desc->conn_itvl * 1.25f, desc->conn_itvl,
                 desc->conn_latency,
                 desc->supervision_timeout * 10,
                 mode);

        // If Bolt is active, NO PC is ever allowed to sleep (PERMANENT TURBO for all PCs).
        // If BLE mouse is used, only the active PC (or sole connected PC) is kept in TURBO.
        // If NO mouse is active (e.g. Bolt unplugged and no BLE mouse), background standby is expected.
        bool boltActive = logi_bolt_is_mouse_connected();
        bool isMouseActive = mouseConnected || boltActive;
        String activeMac = (monitorCount > 0 && currentMonitorIndex < monitorCount) ? monitors[currentMonitorIndex].mac : "";
        bool isCurrentActivePc = (activeMac.length() > 0 && peerMac.equalsIgnoreCase(activeMac));
        bool shouldBeTurbo = isMouseActive && isCurrentActivePc;

        static uint32_t lastReassertTime[MAX_SUPPORTED_KVM_CLIENTS] = {0};
        uint16_t handle = desc->conn_handle;
        uint32_t now = millis();

        if (shouldBeTurbo && desc->conn_latency > 0 && handle < MAX_SUPPORTED_KVM_CLIENTS && (now - lastReassertTime[handle] > 15000)) {
            lastReassertTime[handle] = now;
            logPrint("[BLE Server] Active PC %s in Latency %d (shouldBeTurbo=1). Scheduling single TURBO re-assertion...",
                     peerMac.c_str(), desc->conn_latency);
            xTaskCreate([](void* param) {
                uint16_t connId = (uint16_t)(uintptr_t)param;
                vTaskDelay(pdMS_TO_TICKS(250));
                NimBLEServer* srv = NimBLEDevice::getServer();
                if (srv) {
                    ble_gap_conn_desc d;
                    if (ble_gap_conn_find(connId, &d) == 0 && d.conn_latency > 0) {
                        String pMac = NimBLEAddress(d.peer_ota_addr).toString().c_str();
                        pMac.toLowerCase();
                        pMac.trim();
                        int pcOs = OS_WINDOWS;
                        for (int m = 0; m < monitorCount; m++) {
                            if (monitors[m].mac.equalsIgnoreCase(pMac)) {
                                pcOs = monitors[m].os;
                                break;
                            }
                        }
                        if (pcOs == OS_MAC) {
                            srv->updateConnParams(connId, 12, 12, 0, 216);
                            logPrint("[BLE Server] Re-asserted macOS TURBO for conn %d (Target: 15.00ms, Latency: 0, ⚡)!", connId);
                        } else {
                            srv->updateConnParams(connId, 6, 8, 0, 600);
                            logPrint("[BLE Server] Re-asserted Windows TURBO for conn %d (Target: 7.50..10.00ms, Latency: 0, ⚡)!", connId);
                        }
                    }
                }
                vTaskDelete(NULL);
            }, "reTurboTask", 4096, (void*)(uintptr_t)desc->conn_handle, 1, NULL);
        }
    }
    void onConnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) {
        String peerMac = NimBLEAddress(desc->peer_ota_addr).toString().c_str();
        peerMac.toLowerCase();
        peerMac.trim();
        logPrint("[BLE Server] PC Connected! MAC: %s (conn_handle: %d | itvl: %d | latency: %d | timeout: %d)",
                  peerMac.c_str(), desc->conn_handle, desc->conn_itvl, desc->conn_latency, desc->supervision_timeout);

        // Save connection
        bool isLayoutPc = (monitorCount == 0) || isMacInActiveLayout(peerMac);

        // Request BLE 5.0 2M PHY (2 Mbps ultra-low latency) only for KVM layout PCs
        if (isLayoutPc) {
            ble_gap_set_prefered_le_phy(desc->conn_handle, BLE_GAP_LE_PHY_2M_MASK | BLE_GAP_LE_PHY_1M_MASK, BLE_GAP_LE_PHY_2M_MASK | BLE_GAP_LE_PHY_1M_MASK, 0);
            checkAndLogPhyStatus(desc->conn_handle, peerMac.c_str());
        }

        if (isLayoutPc) {
            bool updated = false;
            for (int i = 0; i < MAX_SUPPORTED_KVM_CLIENTS; i++) {
                if (kvmClients[i].mac.equals(peerMac)) {
                    kvmClients[i].conn_id = desc->conn_handle;
                    kvmClients[i].active = true;
                    kvmClients[i].isTurbo = false;
                    updated = true;
                    break;
                }
            }
            if (!updated) {
                for (int i = 0; i < MAX_SUPPORTED_KVM_CLIENTS; i++) {
                    if (!kvmClients[i].active && kvmClients[i].mac.length() == 0) {
                        kvmClients[i].conn_id = desc->conn_handle;
                        kvmClients[i].mac = peerMac;
                        kvmClients[i].name = ""; // Always reset name to prevent leaking stale name from previous device
                        kvmClients[i].active = true;
                        kvmClients[i].isTurbo = false;
                        break;
                    }
                }
            }
        } else {
            // Non-KVM Client (Candidate Web Bluetooth Configurator) - do NOT pollute kvmClients!
            bool slotted = false;
            for (int i = 0; i < MAX_NON_KVM_CLIENTS; i++) {
                if (nonKvmClients[i].conn_id == BLE_HS_CONN_HANDLE_NONE || nonKvmClients[i].mac.equals(peerMac)) {
                    nonKvmClients[i].conn_id = desc->conn_handle;
                    nonKvmClients[i].mac = peerMac;
                    nonKvmClients[i].connectedTimeMs = millis();
                    nonKvmClients[i].isWebConfig = false;
                    slotted = true;
                    break;
                }
            }
            if (!slotted) {
                nonKvmClients[0].conn_id = desc->conn_handle;
                nonKvmClients[0].mac = peerMac;
                nonKvmClients[0].connectedTimeMs = millis();
                nonKvmClients[0].isWebConfig = false;
            }
            logPrint("[BLE Server] ⏳ Non-KVM Client %s (conn: %d) connected. Starting 3s Web Config Grace Period...", 
                     peerMac.c_str(), desc->conn_handle);
        }

        int activeCount = 0;
        for (int i = 0; i < MAX_SUPPORTED_KVM_CLIENTS; i++) {
            if (kvmClients[i].active) activeCount++;
        }
        isCalibrated = false;
        if (activeCount == 1 && isMacInActiveLayout(peerMac)) {
            firstConnectedPcMac = peerMac;
            scheduleBootCalibration();
        }

        // Do not update connection parameters immediately in onConnect to prevent sch_prog.c assertion
        checkAndResumeAdvertising();
    }

    void onDisconnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) {
        String peerMac = NimBLEAddress(desc->peer_ota_addr).toString().c_str();
        peerMac.toLowerCase();
        peerMac.trim();
        logPrint("[BLE Server] PC Disconnected! MAC: %s (conn_handle: %d)", peerMac.c_str(), desc->conn_handle);

        isWebBleAuthenticated = false; // Reset Web Bluetooth authorization on client disconnect
        currentAuthNonce = "";
        
        // Check if disconnected client was a non-KVM / Web client
        bool isNonKvm = false;
        for (int i = 0; i < MAX_NON_KVM_CLIENTS; i++) {
            if (nonKvmClients[i].conn_id == desc->conn_handle || nonKvmClients[i].mac.equals(peerMac)) {
                nonKvmClients[i].conn_id = BLE_HS_CONN_HANDLE_NONE;
                nonKvmClients[i].mac = "";
                nonKvmClients[i].connectedTimeMs = 0;
                nonKvmClients[i].isWebConfig = false;
                isNonKvm = true;
                break;
            }
        }

        for (int i = 0; i < MAX_SUPPORTED_KVM_CLIENTS; i++) {
            if (kvmClients[i].conn_id == desc->conn_handle || kvmClients[i].mac.equals(peerMac)) {
                kvmClients[i].active = false;
                kvmClients[i].isTurbo = false;
                break;
            }
        }

        // If the disconnected PC was the current active PC, failover to another connected PC!
        isCalibrated = false;
        if (monitorCount > 0 && monitors[currentMonitorIndex].mac.equals(peerMac)) {
            firstConnectedPcMac = "";
            for (int i = 0; i < MAX_SUPPORTED_KVM_CLIENTS; i++) {
                if (kvmClients[i].active && !kvmClients[i].mac.equals(peerMac) && isMacInActiveLayout(kvmClients[i].mac)) {
                    firstConnectedPcMac = kvmClients[i].mac;
                    break;
                }
            }
            if (firstConnectedPcMac.length() > 0) {
                scheduleBootCalibration();
                logPrint("[FAILOVER] Current PC %s disconnected! Switched control to active PC %s (Cursor at %ld, %ld)", peerMac.c_str(), firstConnectedPcMac.c_str(), virtualX, virtualY);
            } else {
                logPrint("[FAILOVER] Current PC %s disconnected and no other active PC available.", peerMac.c_str());
            }
        }
        updateKvmPowerAndRateProfiles(isCalibrated ? firstConnectedPcMac : "");
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
        } else {
            int activeCount = 0;
            for (int i = 0; i < MAX_SUPPORTED_KVM_CLIENTS; i++) {
                if (kvmClients[i].active) activeCount++;
            }
            if (activeCount == 1) {
                firstConnectedPcMac = peerMac;
                isCalibrated = false;
                scheduleBootCalibration();
            }
            // Connection is securely bonded and link layer is stable: apply rate profile safely!
            updateKvmPowerAndRateProfiles(peerMac, true);
        }
    }
};

String getMonDisplayName(int idx) {
    return monitors[idx].name;
}

int getActiveClientOs() {
    if (monitorCount > 0 && currentMonitorIndex < monitorCount) {
        return monitors[currentMonitorIndex].os;
    }
    return OS_WINDOWS;
}

uint16_t getTargetConnHandle(const String& targetMac) {
    for (int i = 0; i < MAX_SUPPORTED_KVM_CLIENTS; i++) {
        if (kvmClients[i].active && kvmClients[i].mac.equals(targetMac)) {
            return kvmClients[i].conn_id;
        }
    }
    return BLE_HS_CONN_HANDLE_NONE;
}

// Send HID report directly to target connection handle
void sendHidReport(NimBLECharacteristic* pChar, uint16_t connHandle, const uint8_t* report, size_t length) {
    if (!pChar || !report || length == 0 || connHandle == BLE_HS_CONN_HANDLE_NONE) return;
    os_mbuf *om = ble_hs_mbuf_from_flat(report, length);
    if (!om) return;
    if (ble_gatts_notify_custom(connHandle, pChar->getHandle(), om) != 0) {
        os_mbuf_free_chain(om);
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

void sendRelative12Bit(uint16_t connHandle, int32_t dx, int32_t dy, uint8_t buttons = 0, int8_t scroll = 0, int8_t hScroll = 0) {
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
    absReport[0] = 0x00;                    // In Range = OFF
    sendHidReport(macAbsInputChar, connHandle, absReport, sizeof(absReport));
    logPrint("[%s] Sent macOS1 digitizer position to PC %s at (%ld, %ld) [Rel: %ld, %ld -> Norm: %u, %u] on Mon #%d (%s)",
             contextLabel, targetMon.mac.c_str(), targetGlobalX, targetGlobalY, relX, relY, absX, absY, targetMon.id, targetMon.name.c_str());
}

// --- Absolute HID Positioning Function ---
void sendAbsoluteCoordinates(uint16_t connHandle, int monIndex, long targetGlobalX, long targetGlobalY, const char* contextLabel) {
    if (monitors[monIndex].os == OS_MAC) {
        sendAbsoluteCoordinatesMacOs(connHandle, monIndex, targetGlobalX, targetGlobalY, contextLabel);
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
                updateKvmPowerAndRateProfiles(monitors[newMonitorIndex].mac, true);
                syncPhysicalKeyboardLedsForPc(monitors[newMonitorIndex].mac);
                sendAbsoluteCoordinates(targetConn, newMonitorIndex, virtualX, virtualY, "PC SWITCH");
            }
        }
        resetSubpixelAccumulators();
    }

    sendRelative12Bit(connHandle, sendDx, sendDy, buttons, scroll, hScroll);
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

// Dynamic Modifier Remapping for Mac / Windows
// Logitech keyboards (like MX Keys) send standard PC/Windows modifier codes:
// - Key next to Spacebar sends Left Alt (0x04)
// - Key to the left of it (Start) sends Left GUI/Win (0x08)
// On macOS, users expect the key next to Spacebar (labeled 'cmd') to act as Command (0x08),
// and the 'opt' key to act as Option (0x04).
// When target OS is Mac, swap Left Alt <-> Left GUI and Right Alt <-> Right GUI.
static inline uint8_t remapModifiersForTargetOs(uint8_t mods, int targetOs) {
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

// --- macOS Ctrl+Shift -> Globe Key Detection ---
static bool s_ctrlShiftArmed = false;
static bool s_ctrlShiftOtherKey = false;

void checkCtrlShiftGlobeTrigger(uint8_t rawMods, const uint8_t* rep8, int targetOs, uint16_t targetConn) {
    if (targetOs != OS_MAC) {
        s_ctrlShiftArmed = false;
        s_ctrlShiftOtherKey = false;
        return;
    }

    bool hasCtrl = (rawMods & (0x01 | 0x10)) != 0;
    bool hasShift = (rawMods & (0x02 | 0x20)) != 0;
    bool hasOtherMods = (rawMods & ~(0x01 | 0x02 | 0x10 | 0x20)) != 0; // Alt or GUI pressed

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
            if (!s_ctrlShiftArmed) {
                s_ctrlShiftArmed = true;
                s_ctrlShiftOtherKey = false; // Reset: Fresh clean Ctrl+Shift engagement
            }
        } else {
            s_ctrlShiftOtherKey = true; // Non-modifier key was pressed while holding Ctrl+Shift (e.g. Ctrl+Shift+T)
        }
    } else {
        if (s_ctrlShiftArmed) {
            if (!s_ctrlShiftOtherKey) {
                logPrint("[GLOBE KEY] Ctrl+Shift release detected -> Triggering Globe key on macOS!");
                sendGlobePulseToMac(targetConn);
            }
            s_ctrlShiftArmed = false;
            s_ctrlShiftOtherKey = false;
        }
    }
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
        sendHidReport(keyboardInputChar, targetConn, rep8, length);
        checkAndSyncCapsLock(rep8);
        checkCtrlShiftGlobeTrigger(pData[0], rep8, targetOs, targetConn);
        logPrint("[KEYBOARD FWD] 8B [Mods: 0x%02X] -> Conn %d (Mon #%d, OS: %s)",
                 rep8[0], targetConn, currentMonitorIndex + 1, targetOs == OS_MAC ? "Mac" : "Win");
    } else if (length == 9) {
        // 9-byte report with Report ID prepended: forward payload without Report ID
        uint8_t rep8[8];
        memcpy(rep8, &pData[1], 8);
        rep8[0] = remapModifiersForTargetOs(pData[1], targetOs);
        sendHidReport(keyboardInputChar, targetConn, rep8, 8);
        checkAndSyncCapsLock(rep8);
        checkCtrlShiftGlobeTrigger(pData[1], rep8, targetOs, targetConn);
        logPrint("[KEYBOARD FWD] 9B (ID 0x%02X) [Mods: 0x%02X] -> Conn %d (Mon #%d, OS: %s)",
                 pData[0], rep8[0], targetConn, currentMonitorIndex + 1, targetOs == OS_MAC ? "Mac" : "Win");
    } else if (length == 2) {
        // Consumer Control report (Media keys)
        sendHidReport(mediaInputChar, targetConn, pData, length);
        logPrint("[MEDIA FWD] 2B -> Conn %d (Mon #%d)", targetConn, currentMonitorIndex + 1);
    } else if (length == 3) {
        // Consumer Control report with Report ID prepended
        sendHidReport(mediaInputChar, targetConn, &pData[1], 2);
        logPrint("[MEDIA FWD] 3B (ID 0x%02X) -> Conn %d (Mon #%d)", pData[0], targetConn, currentMonitorIndex + 1);
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
        sendHidReport(keyboardInputChar, targetConn, rep8, 8);
        checkAndSyncCapsLock(rep8);
        checkCtrlShiftGlobeTrigger(pData[0], rep8, targetOs, targetConn);
        logPrint("[KEYBOARD FWD] Bolt Bitmap %dB->8B [Mods: 0x%02X, Keys: %02X %02X %02X] -> Conn %d (OS: %s)",
                 (int)length, rep8[0], rep8[2], rep8[3], rep8[4], targetConn, targetOs == OS_MAC ? "Mac" : "Win");
    } else if (length == 19 || pData[0] == 0xFF) {
        // Logitech HID++ vendor packet: ignore
    } else {
        uint8_t repBuf[8];
        size_t copyLen = min((size_t)8, length);
        memcpy(repBuf, pData, copyLen);
        if (copyLen > 0) {
            repBuf[0] = remapModifiersForTargetOs(pData[0], targetOs);
            checkCtrlShiftGlobeTrigger(pData[0], repBuf, targetOs, targetConn);
        }
        sendHidReport(keyboardInputChar, targetConn, repBuf, copyLen);
        logPrint("[KEYBOARD FWD] %dB -> Conn %d", (int)length, targetConn);
    }
}

bool connectToMouse();
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

        if (!mouseConnected && !logi_bolt_is_mouse_connected() && !isConnectingToMouse && mouseMatch) {
            isConnectingToMouse = true; // Set flag immediately to throttle multiple advertising packets
            logPrint("[BLE Scan] TARGET MOUSE MATCH! Connecting to %s (%s)", devName.c_str(), devMac.c_str());
            NimBLEDevice::getScan()->stop();
            advDevice = new NimBLEAdvertisedDevice(*advertisedDevice);
            doConnectMouse = true;
        }

        // 2. Keyboard Check (Strict exact MAC match only)
        bool kbMatch = false;
        if (!mouseMatch && targetKeyboardMac.length() > 0 && devMac == targetKeyboardMac) {
            kbMatch = true;
        }

        if (!kbConnected && !logi_bolt_is_keyboard_connected() && !isConnectingToKeyboard && kbMatch) {
            isConnectingToKeyboard = true; // Set flag immediately to throttle multiple advertising packets
            logPrint("[BLE Scan] TARGET KEYBOARD MATCH! Connecting to %s (%s)...", devName.c_str(), devMac.c_str());
            NimBLEDevice::getScan()->stop();
            advKbDevice = new NimBLEAdvertisedDevice(*advertisedDevice);
            doConnectKeyboard = true;
        }
    }
};

// =========================================================================================
// ULTRA-FAST HOST RECONNECTION & LINK-LAYER SUBSYSTEM (OS-Level Speed Architecture)
// =========================================================================================
TaskHandle_t hostScanTaskHandle = NULL;
static ScanCallbacks* globalScanCallbacks = nullptr;

/**
 * @brief Background daemon maintaining active reconnection with bonded HID peripherals.
 * Implements 30ms 50%-duty-cycle scanning to reliably catch peripheral advertisements
 * without exhausting controller radio scheduler resources.
 * Uses FreeRTOS Task Notifications for 0ms instant wakeups upon peripheral disconnects.
 */
void startHostReconnectTask() {
    if (hostScanTaskHandle != NULL) return; // Daemon already active
    xTaskCreate([](void* param) {
        logPrint("[BLE Host] Host Reconnect Daemon started (Instant Wakeup Mode).");
        while (true) {
            bool needMouse = !mouseConnected && !logi_bolt_is_mouse_connected();
            bool needKb = !kbConnected && !logi_bolt_is_keyboard_connected();

            if (!needMouse && !needKb) {
                // Both peripherals connected: Stop radio scanner to reserve 100% bandwidth for HID traffic.
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
                    if (!globalScanCallbacks) globalScanCallbacks = new ScanCallbacks();
                    pScan->setAdvertisedDeviceCallbacks(globalScanCallbacks, false);
                    pScan->setActiveScan(true);
                    pScan->setInterval(48);  // 30ms interval
                    pScan->setWindow(24);    // 15ms window (50% duty cycle, clean radio coexistence)
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
    }
    void onDisconnect(NimBLEClient* pClientArg) {
        logPrint("[BLE Host] Disconnected from mouse!");
        mouseConnected = false;
        isConnectingToMouse = false;
        isCalibrated = false;
        // Instantly return all PCs to Standby (80ms) to free 98% radio airtime for mouse reconnect
        updateKvmPowerAndRateProfiles("", true);
        if (pClient) {
            NimBLEDevice::deleteClient(pClient);
            pClient = nullptr;
        }
        // Instantly wake up the reconnect daemon without waiting for periodic timer tick
        if (hostScanTaskHandle != NULL) {
            xTaskNotifyGive(hostScanTaskHandle);
        }
    }
    bool onConnParamsUpdateRequest(NimBLEClient* pClient, const ble_gap_upd_params* params) {
        return true;
    }
};

// Callback for BLE Keyboard Connection Status
class KeyboardClientCallbacks : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient* pClient) {
        logPrint("[BLE Host] Connected to Keyboard!");
        kbConnected = true;
    }
    void onDisconnect(NimBLEClient* pClientArg) {
        logPrint("[BLE Host] Disconnected from Keyboard!");
        kbConnected = false;
        isConnectingToKeyboard = false;
        pKbLedChar = nullptr;
        pKbBootLedChar = nullptr;
        if (pKbClient) {
            NimBLEDevice::deleteClient(pKbClient);
            pKbClient = nullptr;
        }
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
                logPrint("[BLE KB Char] UUID: %s | Notify: %d | Write: %d | WriteNR: %d",
                         pChar->getUUID().toString().c_str(),
                         pChar->canNotify(), pChar->canWrite(), pChar->canWriteNoResponse());

                if (pChar->canNotify()) {
                    // Async subscribe (response=false) completes in 0ms without blocking FreeRTOS queue
                    pChar->subscribe(true, keyboardNotifyCallback, false);
                    subCount++;
                }
                
                if (pChar->canWrite() || pChar->canWriteNoResponse()) {
                    if (pChar->getUUID() == NimBLEUUID((uint16_t)0x2A32)) {
                        pKbBootLedChar = pChar;
                        logPrint("[BLE Host] Identified Boot Output (0x2A32) for Keyboard LEDs!");
                    } else if (pChar->getUUID() == NimBLEUUID((uint16_t)0x2A4D)) {
                        NimBLERemoteDescriptor* pDesc = pChar->getDescriptor(NimBLEUUID((uint16_t)0x2908));
                        if (pDesc != nullptr) {
                            std::string descVal = pDesc->readValue();
                            if (descVal.length() >= 2) {
                                uint8_t repId = (uint8_t)descVal[0];
                                uint8_t repType = (uint8_t)descVal[1];
                                logPrint("[BLE Host] Report (0x2A4D) -> ID: %d, Type: %d (1=In, 2=Out, 3=Feat)", repId, repType);
                                if (repType == 2 && repId == 1) { // Standard Keyboard Output Report (LEDs)
                                    pKbLedChar = pChar;
                                    logPrint("[BLE Host] *** MATCH! Selected Standard Keyboard Output Report (ID 1) for LEDs! ***");
                                }
                            }
                        }
                    }
                }
            }
        }

        kbConnected = true;
        isConnectingToKeyboard = false;
        ble_gap_set_prefered_le_phy(pKbClient->getConnId(), BLE_GAP_LE_PHY_2M_MASK | BLE_GAP_LE_PHY_1M_MASK, BLE_GAP_LE_PHY_2M_MASK | BLE_GAP_LE_PHY_1M_MASK, 0);
        checkAndLogPhyStatus(pKbClient->getConnId(), "Keyboard");
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

bool connectToMouse() {
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
        vTaskDelay(pdMS_TO_TICKS(1500));
        isConnectingToMouse = false;
        if (!kbConnected || !mouseConnected) startHostReconnectTask();
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
                    pChar->subscribe(true, notifyCallback, true); // Synchronous: waits for ATT_WRITE_RSP confirmation
                    logPrint("[BLE Host] Subscribed to HID report (Acknowledged)!");
                }
            }
        }
        ble_gap_set_prefered_le_phy(pClient->getConnId(), BLE_GAP_LE_PHY_2M_MASK | BLE_GAP_LE_PHY_1M_MASK, BLE_GAP_LE_PHY_2M_MASK | BLE_GAP_LE_PHY_1M_MASK, 0);
        checkAndLogPhyStatus(pClient->getConnId(), "Mouse");
        logPrint("[BLE Host] GATT Setup Complete Event -> Mouse HID ready!");
        ble_gap_conn_desc mouseDesc;
        if (ble_gap_conn_find(pClient->getConnId(), &mouseDesc) == 0 && mouseDesc.conn_itvl > 9) {
            logPrint("[BLE Host] Requesting low-latency connection params for mouse (Target: 7.50..11.25ms)...");
            pClient->updateConnParams(6, 9, 44, 216);
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
    if (!isCalibrated) {
        scheduleBootCalibration();
    } else {
        updateKvmPowerAndRateProfiles(monitors[currentMonitorIndex].mac, true);
    }
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
  if (configTxChar) {
    size_t len = fullResp.length();
    size_t chunkSize = 240; // Safe chunk size to avoid exhausting BLE packet memory

    logPrint("[BLE TX] Sending %d bytes in %d-byte chunks...", (int)len, (int)chunkSize);

    for (size_t i = 0; i < len; i += chunkSize) {
      String chunk = fullResp.substring(i, min(i + chunkSize, len));
      configTxChar->setValue((const uint8_t*)chunk.c_str(), chunk.length());
      configTxChar->notify();
      vTaskDelay(pdMS_TO_TICKS(35)); // Yield CPU to IDLE task and let NimBLE host flush HCI buffers
    }
  }
}


static String readNvsBlob(Preferences& pref, const char* key, const String& fallback = "[]") {
    if (!pref.isKey(key)) {
        return fallback;
    }
    size_t len = pref.getBytesLength(key);
    if (len > 0) {
        char* buf = (char*)malloc(len + 1);
        if (buf) {
            pref.getBytes(key, buf, len);
            buf[len] = '\0';
            String res = String(buf);
            free(buf);
            return res;
        }
    }
    return fallback;
}

void saveMouseToNvsLayout(String mac, String name) {
    targetMouseMac = mac;
    targetMouseName = name;

    preferences.begin(NVS_NAMESPACE, false);
    preferences.putString(NVS_KEY_MOUSE_MAC, targetMouseMac);
    preferences.putString(NVS_KEY_MOUSE_NAME, targetMouseName);
    preferences.end();

    logPrint("[NVS] Persisted mouse (%s, '%s') to granular NVS keys.", targetMouseMac.c_str(), targetMouseName.c_str());
}

void saveKeyboardToNvsLayout(String mac, String name) {
    targetKeyboardMac = mac;
    targetKeyboardName = name;

    preferences.begin(NVS_NAMESPACE, false);
    preferences.putString(NVS_KEY_KB_MAC, targetKeyboardMac);
    preferences.putString(NVS_KEY_KB_NAME, targetKeyboardName);
    preferences.end();

    logPrint("[NVS] Persisted keyboard (%s, '%s') to granular NVS keys.", targetKeyboardMac.c_str(), targetKeyboardName.c_str());
}

String buildConfigJson() {
    preferences.begin(NVS_NAMESPACE, true);
    int activeLayoutId = preferences.getInt(NVS_KEY_ACT_LAYOUT_ID, 3);
    int totalLayouts = preferences.getInt(NVS_KEY_TOTAL_LAYOUTS, 3);
    String mouseMac = preferences.getString(NVS_KEY_MOUSE_MAC, targetMouseMac);
    String mouseName = preferences.getString(NVS_KEY_MOUSE_NAME, targetMouseName);
    String kbMac = preferences.getString(NVS_KEY_KB_MAC, targetKeyboardMac);
    String kbName = preferences.getString(NVS_KEY_KB_NAME, targetKeyboardName);
    String layoutsJson = readNvsBlob(preferences, NVS_KEY_LAYOUTS, "[]");
    String clientsJson = readNvsBlob(preferences, NVS_KEY_CLIENTS, "[]");
    preferences.end();

    JsonDocument doc;
    doc["activeLayoutId"] = activeLayoutId;
    doc["totalLayouts"] = totalLayouts;

    deserializeJson(doc["layouts"], layoutsJson);
    if (!doc["layouts"].is<JsonArray>()) {
        doc["layouts"].to<JsonArray>();
    }

    deserializeJson(doc["clients"], clientsJson);
    if (!doc["clients"].is<JsonArray>()) {
        doc["clients"].to<JsonArray>();
    }

    doc["mouseMac"] = mouseMac;
    doc["mouseName"] = mouseName.length() > 0 ? mouseName : (mouseMac.length() > 0 ? "BLE Mouse" : "");
    doc["keyboardMac"] = kbMac;
    doc["keyboardName"] = kbName.length() > 0 ? kbName : (kbMac.length() > 0 ? "BLE Keyboard" : "");

    // Update connected status for clients based on live kvmClients[]
    JsonArray clientsArr = doc["clients"].as<JsonArray>();
    for (JsonObject c : clientsArr) {
        c["connected"] = false;
    }
    for (int i = 0; i < maxKvmClients; i++) {
        if (kvmClients[i].active && kvmClients[i].mac.length() > 0) {
            String activeMac = kvmClients[i].mac;
            bool exists = false;
            for (JsonObject c : clientsArr) {
                String cMac = c["mac"] | "";
                if (cMac.equalsIgnoreCase(activeMac)) {
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
    return unifiedJson;
}

String loadLayoutJsonFromNVS() {
    return buildConfigJson();
}

void loadConfiguration() {
    preferences.begin(NVS_NAMESPACE, true);
    targetMouseMac = preferences.getString(NVS_KEY_MOUSE_MAC, "");
    targetMouseName = preferences.getString(NVS_KEY_MOUSE_NAME, "");
    targetKeyboardMac = preferences.getString(NVS_KEY_KB_MAC, "");
    targetKeyboardName = preferences.getString(NVS_KEY_KB_NAME, "");
    if (targetKeyboardMac.length() > 0 && targetKeyboardMac == targetMouseMac) {
        targetKeyboardMac = "";
        targetKeyboardName = "";
    }

    int targetId = preferences.getInt(NVS_KEY_ACT_LAYOUT_ID, 1);
    String layoutsJson = readNvsBlob(preferences, NVS_KEY_LAYOUTS, "[]");
    String clientsJson = readNvsBlob(preferences, NVS_KEY_CLIENTS, "[]");
    preferences.end();

    JsonDocument docLayouts;
    deserializeJson(docLayouts, layoutsJson);

    JsonArray arr;
    if (docLayouts.is<JsonArray>() && docLayouts.size() > 0) {
        JsonObject activeLayout = docLayouts[0].as<JsonObject>();
        for (JsonObject l : docLayouts.as<JsonArray>()) {
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
            monitors[monitorCount].keepAlive = repo["keepAlive"] | 0;
            monitorCount++;
        }
    }

    // Calculate distinct PCs from the loaded layout screens
    int pcCount = 0;
    String uniquePcMacs[MAX_SUPPORTED_KVM_CLIENTS];
    for (int i = 0; i < monitorCount; i++) {
        String mMac = monitors[i].mac;
        if (mMac.length() > 0) {
            bool found = false;
            for (int p = 0; p < pcCount; p++) {
                if (uniquePcMacs[p].equals(mMac)) {
                    found = true;
                    break;
                }
            }
            if (!found && pcCount < MAX_SUPPORTED_KVM_CLIENTS) {
                uniquePcMacs[pcCount++] = mMac;
            }
        }
    }

    JsonDocument docClients;
    deserializeJson(docClients, clientsJson);

    // Also include docClients if present
    if (docClients.is<JsonArray>()) {
        for (JsonObject client : docClients.as<JsonArray>()) {
            String cMac = client["mac"] | "";
            if (cMac.length() > 0) {
                bool found = false;
                for (int p = 0; p < pcCount; p++) {
                    if (uniquePcMacs[p].equals(cMac)) {
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

    // Dynamically update maxKvmClients based on current configuration
    maxKvmClients = (pcCount > 0) ? pcCount : 2;

    // Populate / update kvmClients array while preserving existing active connection handles
    if (docClients.is<JsonArray>()) {
        int clientCount = 0;
        for (JsonObject client : docClients.as<JsonArray>()) {
            if (clientCount >= maxKvmClients) break;
            String mac = client["mac"] | "";
            mac.toLowerCase();
            mac.trim();
            if (mac.length() > 0) {
                bool alreadyConnected = false;
                uint16_t existingConn = BLE_HS_CONN_HANDLE_NONE;
                for (int k = 0; k < MAX_SUPPORTED_KVM_CLIENTS; k++) {
                    if (kvmClients[k].mac.equals(mac) && kvmClients[k].active) {
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

    logPrint("Loaded %d monitors, %d KVM PC clients (maxKvmClients = %d) from granular NVS. Mouse: %s (%s) | Keyboard: %s (%s)",
             monitorCount, pcCount, maxKvmClients, targetMouseMac.c_str(), targetMouseName.c_str(), targetKeyboardMac.c_str(), targetKeyboardName.c_str());

    checkAndResumeAdvertising();
}

void saveConfiguration(const String& jsonString) {
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, jsonString);
    if (err || !doc.is<JsonObject>()) {
        logPrint("[NVS ERROR] saveConfiguration failed: Invalid JSON or not an object!");
        return;
    }
    preferences.begin(NVS_NAMESPACE, false);
    int actId = doc["activeLayoutId"] | 0;
    if (actId > 0) {
        preferences.putInt(NVS_KEY_ACT_LAYOUT_ID, actId);
    }
    int totLay = doc["totalLayouts"] | 0;
    if (totLay > 0) {
        preferences.putInt(NVS_KEY_TOTAL_LAYOUTS, totLay);
    }
    if (doc["layouts"].is<JsonArray>()) {
        String layoutsJson;
        serializeJson(doc["layouts"], layoutsJson);
        preferences.remove(NVS_KEY_LAYOUTS);
        preferences.putBytes(NVS_KEY_LAYOUTS, layoutsJson.c_str(), layoutsJson.length() + 1);
    }
    if (doc["clients"].is<JsonArray>()) {
        String clientsJson;
        serializeJson(doc["clients"], clientsJson);
        preferences.remove(NVS_KEY_CLIENTS);
        preferences.putBytes(NVS_KEY_CLIENTS, clientsJson.c_str(), clientsJson.length() + 1);
    }
    String mac = doc["mouseMac"] | "";
    if (mac.length() > 0) {
        targetMouseMac = mac;
        preferences.putString(NVS_KEY_MOUSE_MAC, targetMouseMac);
    }
    String name = doc["mouseName"] | "";
    name.trim();
    if (name.length() > 0) {
        targetMouseName = name;
        preferences.putString(NVS_KEY_MOUSE_NAME, targetMouseName);
    }
    String kbMac = doc["keyboardMac"] | "";
    if (kbMac.length() > 0) {
        targetKeyboardMac = kbMac;
        preferences.putString(NVS_KEY_KB_MAC, targetKeyboardMac);
    }
    String kbName = doc["keyboardName"] | "";
    kbName.trim();
    if (kbName.length() > 0) {
        targetKeyboardName = kbName;
        preferences.putString(NVS_KEY_KB_NAME, targetKeyboardName);
    }
    preferences.end();
    logPrint("[NVS] Configuration successfully saved to separate NVS keys!");
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
    String unifiedJson = buildConfigJson();
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
        preferences.begin(NVS_NAMESPACE, true);
        int actId = preferences.getInt(NVS_KEY_ACT_LAYOUT_ID, 1);
        int totLay = preferences.getInt(NVS_KEY_TOTAL_LAYOUTS, 1);
        String mMac = preferences.getString(NVS_KEY_MOUSE_MAC, "");
        String mName = preferences.getString(NVS_KEY_MOUSE_NAME, "");
        String kMac = preferences.getString(NVS_KEY_KB_MAC, "");
        String kName = preferences.getString(NVS_KEY_KB_NAME, "");
        String layJson = readNvsBlob(preferences, NVS_KEY_LAYOUTS, "[]");
        String cliJson = readNvsBlob(preferences, NVS_KEY_CLIENTS, "[]");
        preferences.end();

        logPrint("--- [NVS FLASH DUMP] ---");
        logPrint("Namespace: '%s'", NVS_NAMESPACE);
        logPrint("  %s: %d", NVS_KEY_ACT_LAYOUT_ID, actId);
        logPrint("  %s: %d", NVS_KEY_TOTAL_LAYOUTS, totLay);
        logPrint("  %s: '%s' (%s)", NVS_KEY_MOUSE_MAC, mMac.c_str(), mName.c_str());
        logPrint("  %s: '%s' (%s)", NVS_KEY_KB_MAC, kMac.c_str(), kName.c_str());
        logPrint("  %s (len %d): %s", NVS_KEY_LAYOUTS, layJson.length(), layJson.c_str());
        logPrint("  %s (len %d): %s", NVS_KEY_CLIENTS, cliJson.length(), cliJson.c_str());
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

class ConfigTxCallbacks : public NimBLECharacteristicCallbacks {
    void onSubscribe(NimBLECharacteristic* pCharacteristic, ble_gap_conn_desc* desc, uint16_t subValue) override {
        if (subValue > 0 && desc) {
            markClientAsWebConfig(desc->conn_handle);
        }
    }
};

class ConfigRxCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pCharacteristic, ble_gap_conn_desc* desc) override {
        uint16_t connHandle = desc ? desc->conn_handle : BLE_HS_CONN_HANDLE_NONE;
        markClientAsWebConfig(connHandle);
        std::string rxValue = pCharacteristic->getValue();
        if (rxValue.length() > 0) {
            bleRxBuffer += String(rxValue.c_str());
            while (bleRxBuffer.indexOf('\n') != -1) {
                int newlineIdx = bleRxBuffer.indexOf('\n');
                String completeCmd = bleRxBuffer.substring(0, newlineIdx);
                bleRxBuffer = bleRxBuffer.substring(newlineIdx + 1);
                completeCmd.trim();
                if (completeCmd.length() > 0) {
                    bleCmdQueue.push_back(completeCmd);
                }
            }
        }
    }
};

void setup() {
    Serial.setRxBufferSize(16384);
    Serial.begin(115200);
    delay(2000);
    
    logPrint("--- ESP32 KVM Switcher Started ---");
    logi_bolt_init();
    loadConfiguration();
    
    logPrint("[BLE] Initializing NimBLE...");
    uint8_t customMac[6];
    esp_read_mac(customMac, ESP_MAC_BT);
    customMac[5] += 30; // Increment to present fresh identity to PCs to reload new 12-bit Logitech HID descriptor
    esp_base_mac_addr_set(customMac);
    logPrint("[BLE] Custom Base MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             customMac[0], customMac[1], customMac[2], customMac[3], customMac[4], customMac[5]);

    NimBLEDevice::init(BLE_DEVICE_NAME);
    NimBLEDevice::setMTU(512);
    NimBLEDevice::setSecurityAuth(true, false, true); // (bonding=true, mitm=false -> Just Works, sc=true)
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT); // Standard Combo IO Capability
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
    keyboardOutputChar->setCallbacks(new KeyboardOutputCallbacks());
    NimBLECharacteristic* bootOutputChar = hidDevice->bootOutput(); // UUID 0x2A32 (Boot Keyboard Output)
    bootOutputChar->setCallbacks(new KeyboardOutputCallbacks());
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
    configTxChar->setCallbacks(new ConfigTxCallbacks());
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
    logPrint("[BLE Server] Advertising HID Combo '%s' & Web Bluetooth Service...", BLE_DEVICE_NAME);

    // If target devices are bound, start persistent background reconnect task
    startHostReconnectTask();
}

void checkKeepAlive() {
    static uint32_t lastKeepAliveCheck = 0;
    if (millis() - lastKeepAliveCheck < 60000) return;
    lastKeepAliveCheck = millis();

    String currentActiveMac = "";
    if (monitorCount > 0 && currentMonitorIndex >= 0 && currentMonitorIndex < monitorCount) {
        currentActiveMac = monitors[currentMonitorIndex].mac;
    }

    String handledMacs[MAX_SUPPORTED_KVM_CLIENTS];
    int handledCount = 0;

    for (int i = 0; i < monitorCount; i++) {
        if (monitors[i].keepAlive && monitors[i].mac.length() > 0) {
            String targetMac = monitors[i].mac;

            // Do not send keepAlive if cursor is currently on this PC!
            if (currentActiveMac.length() > 0 && targetMac.equals(currentActiveMac)) {
                continue;
            }

            // Check if we already handled this MAC in this cycle
            bool alreadyDone = false;
            for (int h = 0; h < handledCount; h++) {
                if (handledMacs[h].equals(targetMac)) {
                    alreadyDone = true;
                    break;
                }
            }
            if (alreadyDone) continue;
            if (handledCount < MAX_SUPPORTED_KVM_CLIENTS) {
                handledMacs[handledCount++] = targetMac;
            }

            uint16_t connHandle = getTargetConnHandle(targetMac);
            if (connHandle != BLE_HS_CONN_HANDLE_NONE) {
                logPrint("[KeepAlive] Sending 60s micro-jiggle to %s (conn: %d)", targetMac.c_str(), connHandle);
                sendRelative12Bit(connHandle, 1, 0);
                sendRelative12Bit(connHandle, -1, 0);
            }
        }
    }
}

void loop() {
    logi_bolt_loop();

    // Continuous Advertising Watchdog: ensures ESP32 is discoverable without log spam
    static uint32_t lastAdvCheck = 0;
    if (millis() - lastAdvCheck > 2000) {
        lastAdvCheck = millis();
        checkAndResumeAdvertising();
    }

    // Smart Web Grace Period Watchdog: disconnects unconfigured PCs after 3s
    static uint32_t lastGraceCheck = 0;
    if (millis() - lastGraceCheck > 500) {
        lastGraceCheck = millis();
        checkWebGracePeriod();
    }

    // Smart Keep-Alive Watchdog: sends 60s micro-jiggle to background PCs with keepAlive enabled
    checkKeepAlive();

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
            connectToMouse();
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
}
