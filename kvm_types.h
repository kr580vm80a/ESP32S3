#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include <NimBLEDevice.h>
#include <NimBLEHIDDevice.h>
#include "nimble/nimble/host/include/host/ble_gap.h"

#define BLE_DEVICE_NAME "ESP32 KVM Combo"
// Previous: #define FIRMWARE_VERSION 81
#define FIRMWARE_VERSION 82
#define KEEPALIVE_INTERVAL_SEC 60
#define KEEPALIVE_INTERVAL_MS (KEEPALIVE_INTERVAL_SEC * 1000)
#define KEYBOARD_LOG false

// Dual-Core Input Event Queue Definitions
enum InputEventType : uint8_t {
    INPUT_EVENT_MOUSE_RAW = 0,
    INPUT_EVENT_MOUSE_MOVE = 1,
    INPUT_EVENT_KEYBOARD_RAW = 2
};

struct InputEvent {
    InputEventType type;
    uint16_t charHandle;
    uint8_t length;
    union {
        uint8_t raw[24];
        struct {
            uint8_t buttons;
            int16_t dx;
            int16_t dy;
            int8_t scroll;
            int8_t hScroll;
        } mouse;
    };
};

extern QueueHandle_t g_inputEventQueue;

// NVS Flash Storage Constants
extern const char* NVS_NAMESPACE;
extern const char* NVS_KEY_ACT_LAYOUT_ID;
extern const char* NVS_KEY_LAYOUTS;
extern const char* NVS_KEY_CLIENTS;
extern const char* NVS_KEY_MOUSE_MAC;
extern const char* NVS_KEY_MOUSE_NAME;
extern const char* NVS_KEY_KB_MAC;
extern const char* NVS_KEY_KB_NAME;

enum os {
    OS_WINDOWS = 0,
    OS_MAC = 1,
    OS_ANDROID = 2
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
    int keepAlive = 0;
};

#define MAX_MONITORS 10
extern MonitorConfig monitors[MAX_MONITORS];
extern int monitorCount;

// --- Security & Authentication Constants ---
#define BLE_PAIRING_PIN             123456           // Static 6-digit PIN code for PC Bluetooth pairing
#define WEB_BLE_AUTH_PASSPHRASE     "esp32s3"        // Passphrase required for Web Bluetooth browser access

#define CONN_HANDLE_USB_DEVICE      0xFFFE           // Virtual connection handle for wired USB-C PC/Mac
#define MAX_SUPPORTED_KVM_CLIENTS 6 // 6 PCs + 1 Mouse + 1 Keyboard + 1 Web = 9 max NimBLE connections
struct KVMClient {
    uint16_t conn_id = BLE_HS_CONN_HANDLE_NONE;
    String mac = "";
    String name = "";
    bool active = false;
    bool isTurbo = false;
    bool isHandshaking = false;
    bool hidSubscribed = false;
    uint32_t handshakeStartMs = 0;
    uint8_t ledState = 0; // Saved keyboard LED state (Caps/Num/Scroll) for this PC
    uint32_t lastParamUpdateMs = 0; // Debounce duplicate updateConnParams calls
};
extern KVMClient kvmClients[MAX_SUPPORTED_KVM_CLIENTS];

// Dedicated tracking for non-KVM / Web Bluetooth connections
struct NonKvmClient {
    uint16_t conn_id = BLE_HS_CONN_HANDLE_NONE;
    String mac = "";
    uint32_t connectedTimeMs = 0;
    bool isWebConfig = false;
};
#define MAX_NON_KVM_CLIENTS 2
extern NonKvmClient nonKvmClients[MAX_NON_KVM_CLIENTS];

// Virtual Cursor Position & State
extern long virtualX;
extern long virtualY;
extern int currentMonitorIndex;

// Global BLE Server & HID Characteristics
extern NimBLEServer* pServer;
extern NimBLEHIDDevice* hidDevice;
extern NimBLECharacteristic* inputChar;
extern NimBLECharacteristic* absInputChar;
extern NimBLECharacteristic* macAbsInputChar;
extern NimBLECharacteristic* keyboardInputChar;
extern NimBLECharacteristic* keyboardOutputChar;
extern NimBLECharacteristic* mediaInputChar;
extern NimBLERemoteCharacteristic* pKbLedChar;
extern NimBLERemoteCharacteristic* pKbBootLedChar;

// Config Characteristics
extern NimBLECharacteristic* configTxChar;
extern NimBLECharacteristic* configRxChar;

// Preferences & Target Devices
extern Preferences preferences;
extern String targetMouseMac;
extern String targetMouseName;
// Keyboard Central Variables
extern String targetKeyboardMac;
extern String targetKeyboardName;
extern bool mouseConnected;
extern bool kbConnected;
extern bool doConnectMouse;
extern bool doConnectKeyboard;
extern bool isScanningForMice;

// Web Bluetooth Auth & State
extern bool isWebBleAuthenticated;
extern String currentAuthNonce;
extern uint32_t lastConfigActivityTime;
extern std::vector<String> bleCmdQueue;

// Common KVM / BLE helpers
void sendHidReport(NimBLECharacteristic* pChar, uint16_t connHandle, const uint8_t* report, size_t length);
uint16_t getTargetConnHandle(const String& targetMac);
int getActiveClientOs();
String getMonDisplayName(int idx);
bool isMacInActiveLayout(const String& mac);
bool isKnownKvmClient(const String& mac);
bool isAnyPcHandshaking();
int getActiveLayoutPcCount();
bool isConfigModeActive();
void updateKvmPowerAndRateProfiles(String activeMac = "", bool force = false);
void checkAndResumeAdvertising();
void markClientAsWebConfig(uint16_t connHandle);
void checkWebGracePeriod();
void checkKeepAlive();
extern volatile uint32_t g_lastUserActivityMs;
void syncPhysicalKeyboardLedsForPc(const String& targetMac);

// BLE Host / Central helpers
void startHostReconnectTask();
bool connectToMouse();
bool connectToKeyboard();
bool isHostConnectingPeripheral();
void disconnectMouse();
void disconnectKeyboard();
void triggerDeviceDiscoveryScan();

// Command Processor helpers
void sendConfigResponse(const String& response);
void processCommand(String input, bool isBleSource = false);
String calculateSha256(const String& input);

// Logging
void logPrint(const char* format, ...);

