#include <Arduino.h>
#include <Preferences.h>
#include <NimBLEDevice.h>
#include <NimBLEHIDDevice.h>

#include "kvm_types.h"
#include "hid_descriptors.h"
#include "kvm_config.h"
#include "cursor_engine.h"
#include "keyboard_engine.h"
#include "cmd_processor.h"
#include "ble_server.h"
#include "ble_central.h"
#include "logi_bolt.h"

Preferences preferences;

// NVS Flash Storage Constants
const char* NVS_NAMESPACE = "kvm_config";
const char* NVS_KEY_ACT_LAYOUT_ID = "actLayoutId";
const char* NVS_KEY_LAYOUTS       = "layouts";
const char* NVS_KEY_CLIENTS       = "clients";
const char* NVS_KEY_MOUSE_MAC     = "mouseMac";
const char* NVS_KEY_MOUSE_NAME    = "mouseName";
const char* NVS_KEY_KB_MAC        = "keyboardMac";
const char* NVS_KEY_KB_NAME       = "keyboardName";

String firstConnectedPcMac = "";
bool isCalibrated = false;

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
NimBLERemoteCharacteristic* pKbLedChar = nullptr;
NimBLERemoteCharacteristic* pKbBootLedChar = nullptr;

// Config Characteristics
NimBLECharacteristic* configTxChar = nullptr;
NimBLECharacteristic* configRxChar = nullptr;

bool isWebBleAuthenticated = false;
String currentAuthNonce = "";
std::vector<String> bleCmdQueue;

KVMClient kvmClients[MAX_SUPPORTED_KVM_CLIENTS];
NonKvmClient nonKvmClients[MAX_NON_KVM_CLIENTS];

// Virtual Cursor Position & State
long virtualX = 0;
long virtualY = 0;
int currentMonitorIndex = 0;

// Central Target Variables
String targetMouseMac = "";
String targetMouseName = "";
bool isScanningForMice = false;
bool mouseConnected = false;
bool doConnectMouse = false;

String targetKeyboardMac = "";
String targetKeyboardName = "";
bool kbConnected = false;
bool doConnectKeyboard = false;

uint32_t lastConfigActivityTime = 0;

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

void setup() {
    Serial.setRxBufferSize(16384);
    Serial.begin(115200);
    delay(2000);
    
    logPrint("--- ESP32 KVM Switcher Started ---");
    logi_bolt_init();
    loadConfiguration();
    
    initBleServer();

    // If target devices are bound, start persistent background reconnect task
    startHostReconnectTask();
}

void loop() {
    logi_bolt_loop();
    checkWindowsCtrlShiftDwell();

    // Continuous Advertising Watchdog: ensures ESP32 is discoverable without log spam
    static uint32_t lastAdvCheck = 0;
    if (millis() - lastAdvCheck > 2000) {
        lastAdvCheck = millis();
        checkAndResumeAdvertising();
    }

    // Smart Web Grace Period Watchdog: disconnects unconfigured PCs after 45s
    static uint32_t lastGraceCheck = 0;
    if (millis() - lastGraceCheck > 500) {
        lastGraceCheck = millis();
        checkWebGracePeriod();
    }

    // Smart Keep-Alive Watchdog: sends 60s micro-jiggle to background PCs with keepAlive enabled
    checkKeepAlive();

    executePendingSave();

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
