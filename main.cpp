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
#include "usb_manager.h"

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

// Dual-Core FreeRTOS Input Queue Handle
QueueHandle_t g_inputEventQueue = nullptr;

// Fast decoding helper for Logitech standard 12-bit HID mouse reports
static inline bool decodeMouseRaw(const uint8_t* pData, size_t length, uint8_t& buttons, int16_t& dx, int16_t& dy, int8_t& scroll, int8_t& hScroll) {
    if (!pData || length < 6) return false;
    buttons = pData[0] & 0x1F;
    dx = pData[2] | ((pData[3] & 0x0F) << 8);
    if (dx & 0x800) dx |= 0xF000;
    dy = (pData[3] >> 4) | (pData[4] << 4);
    if (dy & 0x800) dy |= 0xF000;
    scroll = (int8_t)pData[5];
    hScroll = (length > 6) ? (int8_t)pData[6] : 0;
    return true;
}

// Dedicated KVM processing task pinned to Core 1 (APP CPU) with Event Coalescing
static void kvmEngineTask(void* pvParameters) {
    InputEvent ev;
    while (true) {
        if (xQueueReceive(g_inputEventQueue, &ev, portMAX_DELAY) == pdTRUE) {
            switch (ev.type) {
                case INPUT_EVENT_MOUSE_RAW:
                case INPUT_EVENT_MOUSE_MOVE: {
                    uint8_t buttons = 0;
                    int32_t totalDx = 0;
                    int32_t totalDy = 0;
                    int32_t totalScroll = 0;
                    int32_t totalHScroll = 0;

                    if (ev.type == INPUT_EVENT_MOUSE_RAW) {
                        int16_t dx = 0, dy = 0;
                        int8_t scroll = 0, hScroll = 0;
                        if (!decodeMouseRaw(ev.raw, ev.length, buttons, dx, dy, scroll, hScroll)) break;
                        totalDx = dx;
                        totalDy = dy;
                        totalScroll = scroll;
                        totalHScroll = hScroll;
                    } else { // INPUT_EVENT_MOUSE_MOVE
                        buttons = ev.mouse.buttons;
                        totalDx = ev.mouse.dx;
                        totalDy = ev.mouse.dy;
                        totalScroll = ev.mouse.scroll;
                        totalHScroll = ev.mouse.hScroll;
                    }

                    // Event Coalescing: Drain & sum all contiguous pending mouse events with identical button state
                    InputEvent nextEv;
                    while (xQueuePeek(g_inputEventQueue, &nextEv, 0) == pdTRUE) {
                        if (nextEv.type == INPUT_EVENT_MOUSE_RAW) {
                            uint8_t nextButtons = 0;
                            int16_t nextDx = 0, nextDy = 0;
                            int8_t nextScroll = 0, nextHScroll = 0;
                            if (decodeMouseRaw(nextEv.raw, nextEv.length, nextButtons, nextDx, nextDy, nextScroll, nextHScroll)) {
                                if (nextButtons == buttons) {
                                    xQueueReceive(g_inputEventQueue, &nextEv, 0); // Pop consumed event
                                    totalDx += nextDx;
                                    totalDy += nextDy;
                                    totalScroll += nextScroll;
                                    totalHScroll += nextHScroll;
                                    continue;
                                }
                            }
                        } else if (nextEv.type == INPUT_EVENT_MOUSE_MOVE) {
                            if (nextEv.mouse.buttons == buttons) {
                                xQueueReceive(g_inputEventQueue, &nextEv, 0); // Pop consumed event
                                totalDx += nextEv.mouse.dx;
                                totalDy += nextEv.mouse.dy;
                                totalScroll += nextEv.mouse.scroll;
                                totalHScroll += nextEv.mouse.hScroll;
                                continue;
                            }
                        }
                        // Stop coalescing if next event is keyboard or mouse with different button state
                        break;
                    }

                    int16_t clampedDx = (int16_t)constrain(totalDx, -32767, 32767);
                    int16_t clampedDy = (int16_t)constrain(totalDy, -32767, 32767);
                    int8_t clampedScroll = (int8_t)constrain(totalScroll, -127, 127);
                    int8_t clampedHScroll = (int8_t)constrain(totalHScroll, -127, 127);

                    // Mac BLE 14-15ms rate-matching
                    static int32_t s_macAccumDx = 0;
                    static int32_t s_macAccumDy = 0;
                    static uint32_t s_macLastSendMs = 0;
                    static uint8_t s_macLastButtons = 0;

                    bool isMacBle = false;
                    if (monitorCount > 0 && currentMonitorIndex < monitorCount) {
                        MonitorConfig& curMon = monitors[currentMonitorIndex];
                        isMacBle = (curMon.os == OS_MAC && getTargetConnHandle(curMon.mac) != CONN_HANDLE_USB_DEVICE);
                    }

                    if (isMacBle) {
                        uint32_t now = millis();
                        bool shouldSend = (buttons != s_macLastButtons) ||
                                        (clampedScroll != 0 || clampedHScroll != 0) ||
                                        (now - s_macLastSendMs >= 14);

                        s_macAccumDx += clampedDx;
                        s_macAccumDy += clampedDy;
                        s_macLastButtons = buttons;

                        if (!shouldSend) break; // Skip sending until 14ms or click/scroll

                        clampedDx = (int16_t)constrain(s_macAccumDx, -32767, 32767);
                        clampedDy = (int16_t)constrain(s_macAccumDy, -32767, 32767);
                        s_macLastSendMs = now;
                    }
                    s_macAccumDx = 0;
                    s_macAccumDy = 0;
                    
                    updateVirtualCursorAndSend(buttons, clampedDx, clampedDy, clampedScroll, clampedHScroll);
                    break;
                }
                case INPUT_EVENT_KEYBOARD_RAW:
                    processKeyboardEvent(ev.charHandle, ev.raw, ev.length);
                    break;
            }
        }
    }
}

void logPrint(const char* format, ...) {
    unsigned long ms = millis();
    char buffer[408];
    snprintf(buffer, sizeof(buffer), "[%02lu:%02lu:%02lu.%03lu] ", (ms / 3600000) % 24, (ms / 60000) % 60, (ms / 1000) % 60, ms % 1000);
    size_t offset = strlen(buffer);
    va_list args;
    va_start(args, format);
    vsnprintf(buffer + offset, sizeof(buffer) - offset, format, args);
    va_end(args);
    Serial.println(buffer);
}

void setup() {
    Serial.setRxBufferSize(16384);
    Serial.begin(115200);
    delay(2000);
    
    logPrint("--- ESP32 KVM Switcher Started (Firmware v%d) ---", FIRMWARE_VERSION);

    // Initialize FreeRTOS event queue and start dedicated KVM engine on Core 1 (Priority 10)
    g_inputEventQueue = xQueueCreate(64, sizeof(InputEvent));
    xTaskCreatePinnedToCore(kvmEngineTask, "kvm_engine", 4096, NULL, 10, NULL, 1);
    logPrint("[CORE CONFIG] Core 0: NimBLE Radio Stack | Core 1: KVM Engine Task (Pri 10)");

    usb_manager_init();
    loadConfiguration();
    
    initBleServer();

    // If target devices are bound, start persistent background reconnect task
    startHostReconnectTask();
}

void loop() {
    usb_manager_loop();
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
