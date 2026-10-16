#include "ble_central.h"
#include "ble_server.h"
#include "cursor_engine.h"
#include "keyboard_engine.h"
#include "cmd_processor.h"
#include "kvm_config.h"
#include "logi_bolt.h"
#include <ArduinoJson.h>

// --- BLE Host (Central) Functions ---

static bool isConnectingToMouse = false;
static NimBLEClient* pMouseClient = nullptr;
static NimBLEAdvertisedDevice* advDevice = nullptr;

static bool isConnectingToKeyboard = false;
static NimBLEClient* pKbClient = nullptr;
static NimBLEAdvertisedDevice* advKbDevice = nullptr;

static NimBLEUUID hidServiceUUID("1812");
static NimBLEUUID reportCharUUID("2a4d");

static JsonDocument scannedMiceDoc;
static TaskHandle_t hostScanTaskHandle = NULL;

static bool isSameDeviceMacPrefix(const String& mac1, const String& mac2, int maxLastByteDiff = 3) {
    if (mac1.length() < 17 || mac2.length() < 17) return false;
    if (!mac1.substring(0, 15).equalsIgnoreCase(mac2.substring(0, 15))) {
        return false;
    }
    int b1 = (int)strtol(mac1.substring(15, 17).c_str(), NULL, 16);
    int b2 = (int)strtol(mac2.substring(15, 17).c_str(), NULL, 16);
    return abs(b1 - b2) <= maxLastByteDiff;
}

class ScanCallbacks : public NimBLEAdvertisedDeviceCallbacks {
    void onResult(NimBLEAdvertisedDevice* advertisedDevice) {
        String devMac = advertisedDevice->getAddress().toString().c_str();

        String devName = advertisedDevice->getName().c_str();
        int rssi = advertisedDevice->getRSSI();

        if (isScanningForMice) {
            String nameLower = devName;
            nameLower.toLowerCase();

            bool hasHidService = advertisedDevice->haveServiceUUID() && advertisedDevice->isAdvertisingService(NimBLEUUID((uint16_t)0x1812));
            uint16_t appearance = advertisedDevice->haveAppearance() ? advertisedDevice->getAppearance() : 0;
            bool hasHidAppearance = (appearance == GENERIC_HID || appearance == HID_KEYBOARD || appearance == HID_MOUSE);
            bool isTargetDevice = (targetMouseMac.length() > 0 && devMac.equals(targetMouseMac)) || 
                                 (targetKeyboardMac.length() > 0 && devMac.equals(targetKeyboardMac));

            // Standard Bluetooth SIG Filter: Only include genuine HID input peripherals (Mice, Keyboards)
            if (!hasHidService && !hasHidAppearance && !isTargetDevice) {
                return; // Ignore smartphones, TVs, smart meters, beacons, etc.
            }

            String devType = "unknown";
            if (appearance == HID_MOUSE || nameLower.indexOf("mouse") != -1 || devMac.equals(targetMouseMac)) {
                devType = "mouse";
            } else if (appearance == HID_KEYBOARD || nameLower.indexOf("keyboard") != -1 || devMac.equals(targetKeyboardMac)) {
                devType = "keyboard";
            } else {
                devType = "hid";
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

        bool hasHidService = advertisedDevice->haveServiceUUID() && advertisedDevice->isAdvertisingService(NimBLEUUID((uint16_t)0x1812));
        uint16_t appearance = advertisedDevice->haveAppearance() ? advertisedDevice->getAppearance() : 0;
        bool isKeyboardAppearance = (appearance == HID_KEYBOARD);
        bool isMouseAppearance = (appearance == HID_MOUSE);

        // 1. Mouse Check (Exact match OR dynamic +/-3 MAC shift from same physical device)
        bool isKbMac = (targetKeyboardMac.length() > 0 && devMac.equals(targetKeyboardMac));
        bool mouseMatch = false;

        if (!isKbMac && targetMouseMac.length() > 0) {
            if (devMac.equalsIgnoreCase(targetMouseMac)) {
                mouseMatch = true;
            } else if (isSameDeviceMacPrefix(targetMouseMac, devMac, 3) && (isMouseAppearance || hasHidService)) {
                int diff = (int)strtol(devMac.substring(15, 17).c_str(), NULL, 16) - (int)strtol(targetMouseMac.substring(15, 17).c_str(), NULL, 16);
                logPrint("[BLE Scan] ⚡ TARGET MOUSE MAC SHIFT DETECTED (%+d)! Old: %s -> New: %s. Auto-updating target...",
                         diff, targetMouseMac.c_str(), devMac.c_str());
                saveMouseToNvsLayout(devMac, targetMouseName.length() > 0 ? targetMouseName : devName);
                mouseMatch = true;
            }
        }

        if (!mouseConnected && !isConnectingToMouse && mouseMatch) {
            isConnectingToMouse = true; // Set flag immediately to throttle multiple advertising packets
            logPrint("[BLE Scan] TARGET MOUSE MATCH! Connecting to %s (%s)", devName.c_str(), devMac.c_str());
            NimBLEDevice::getScan()->stop();
            advDevice = new NimBLEAdvertisedDevice(*advertisedDevice);
            doConnectMouse = true;
        }

        // 2. Keyboard Check (Exact match OR dynamic +/-3 MAC shift from same physical device)
        bool kbMatch = false;
        if (!mouseMatch && targetKeyboardMac.length() > 0) {
            if (devMac.equalsIgnoreCase(targetKeyboardMac)) {
                kbMatch = true;
            } else if (isSameDeviceMacPrefix(targetKeyboardMac, devMac, 3) && (isKeyboardAppearance || hasHidService)) {
                int diff = (int)strtol(devMac.substring(15, 17).c_str(), NULL, 16) - (int)strtol(targetKeyboardMac.substring(15, 17).c_str(), NULL, 16);
                logPrint("[BLE Scan] ⚡ TARGET KEYBOARD MAC SHIFT DETECTED (%+d)! Old: %s -> New: %s. Auto-updating target...",
                         diff, targetKeyboardMac.c_str(), devMac.c_str());
                saveKeyboardToNvsLayout(devMac, targetKeyboardName.length() > 0 ? targetKeyboardName : devName);
                kbMatch = true;
            }
        }

        if (!kbConnected && !isConnectingToKeyboard && kbMatch) {
            isConnectingToKeyboard = true; // Set flag immediately to throttle multiple advertising packets
            logPrint("[BLE Scan] TARGET KEYBOARD MATCH! Connecting to %s (%s)...", devName.c_str(), devMac.c_str());
            NimBLEDevice::getScan()->stop();
            advKbDevice = new NimBLEAdvertisedDevice(*advertisedDevice);
            doConnectKeyboard = true;
        } else if (!kbConnected && !isConnectingToKeyboard) {
            // Universal Bluetooth SIG standard detection (Appearance 0x03C1 or HID Service, zero brand dependency!)
            if (isKeyboardAppearance || (hasHidService && !isMouseAppearance)) {
                static uint32_t lastKbSeenTime = 0;
                if (millis() - lastKbSeenTime > 2500) {
                    lastKbSeenTime = millis();
                    logPrint("[BLE Scan] 🔍 Discovered BLE Keyboard (Appearance: 0x%04X, HID: %d) MAC: %s '%s' (Target: %s)",
                             appearance, hasHidService ? 1 : 0, devMac.c_str(), devName.c_str(), targetKeyboardMac.c_str());
                }
            }
        }
    }
};

bool isHostConnectingPeripheral() {
    return isConnectingToMouse || isConnectingToKeyboard || doConnectMouse || doConnectKeyboard;
}

static ScanCallbacks* globalScanCallbacks = nullptr;

// =========================================================================================
// ULTRA-FAST HOST RECONNECTION & LINK-LAYER SUBSYSTEM (OS-Level Speed Architecture)
// =========================================================================================

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
            bool needMouse = (targetMouseMac.length() > 0) && !mouseConnected;
            bool needKb = (targetKeyboardMac.length() > 0) && !kbConnected;

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
class MouseCallbacks : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient* pClient) {
        logPrint("[BLE Host] Connected to mouse!");
    }
    void onDisconnect(NimBLEClient* pClientArg) {
        logPrint("[BLE Host] Disconnected from mouse!");
        mouseConnected = false;
        isConnectingToMouse = false;
        isCalibrated = false;
        // Shift PCs to standby profile when mouse is disconnected
        updateKvmPowerAndRateProfiles("", true);
        // Instantly wake up the reconnect daemon without waiting for periodic timer tick
        if (hostScanTaskHandle != NULL) {
            xTaskNotifyGive(hostScanTaskHandle);
        }
    }
    bool onConnParamsUpdateRequest(NimBLEClient* pClient, const ble_gap_upd_params* params) {
        logPrint("[BLE Host] Mouse requested params: itvl_min=%d (%.2f ms), itvl_max=%d (%.2f ms), lat=%d, to=%d",
                 params->itvl_min, params->itvl_min * 1.25f,
                 params->itvl_max, params->itvl_max * 1.25f,
                 params->latency, params->supervision_timeout * 10);
        // Scenario B: Enforce strict 7.50ms Turbo (itvl 6..6) so controller cannot pick 11.25ms (itvl 9)
        ble_gap_upd_params* mutableParams = const_cast<ble_gap_upd_params*>(params);
        mutableParams->itvl_min = 6;
        mutableParams->itvl_max = 6;
        mutableParams->latency = 44;
        mutableParams->supervision_timeout = 216;
        logPrint("[BLE Host] Enforced strict 7.50 ms Turbo for Mouse (itvl: 6..6, lat: 44, to: 2160 ms)");
        updateSmartAccumulatorProfile();
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
        resetKeyboardPressedState();
        isConnectingToKeyboard = false;
        pKbLedChar = nullptr;
        pKbBootLedChar = nullptr;
        // Instantly wake up the reconnect daemon without waiting for periodic timer tick
        if (hostScanTaskHandle != NULL) {
            xTaskNotifyGive(hostScanTaskHandle);
        }
    }
    bool onConnParamsUpdateRequest(NimBLEClient* pClient, const ble_gap_upd_params* params) {
        logPrint("[BLE Host] Keyboard requested params: itvl_min=%d (%.2f ms), itvl_max=%d (%.2f ms), lat=%d, to=%d",
                 params->itvl_min, params->itvl_min * 1.25f,
                 params->itvl_max, params->itvl_max * 1.25f,
                 params->latency, params->supervision_timeout * 10);
        return true;
    }
};

// Returns true for transient / retryable link errors (OS-style backoff recovery)
inline bool isTransientConnectionError(int err) {
    // 525 (HCI 0x0D): Limited Resources (peripheral busy finishing channel switch / buffer teardown)
    // 574 (HCI 0x3E): Connection Failed to be Established (RF packet miss / anchor collision)
    // 524 (HCI 0x0C): Command Disallowed (HCI controller busy transitioning)
    // 523 (HCI 0x0B): Connection Already Exists (half-open link handle clearing)
    return (err == 525 || err == 574 || err == 524 || err == 523);
}

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

    isConnectingToKeyboard = true;

    if (!pKbClient) {
        pKbClient = NimBLEDevice::createClient();
        pKbClient->setClientCallbacks(new KeyboardClientCallbacks());
        pKbClient->setConnectTimeout(5);
    }
    // Flexible initial connection interval (24..40 -> 30..50ms) for 100% reliable connection without slot collision.
    // MX KEYS S autonomously sends L2CAP Connection Parameter Update Request right after pairing and encryption complete.
    pKbClient->setConnectionParams(24, 40, 0, 216, 24, 4);

    if (pKbClient->isConnected()) {
        kbConnected = true;
        isConnectingToKeyboard = false;
        return true;
    }

    NimBLEScan* pScan = NimBLEDevice::getScan();
    if (pScan && pScan->isScanning()) {
        pScan->stop();
    }
    if (ble_gap_disc_active()) {
        ble_gap_disc_cancel();
    }
    if (NimBLEDevice::getAdvertising() && NimBLEDevice::getAdvertising()->isAdvertising()) {
        NimBLEDevice::getAdvertising()->stop();
        logPrint("[BLE Host] Advertising STOPPED");
    }
    if (ble_gap_adv_active()) {
        ble_gap_adv_stop();
    }
    if (ble_gap_conn_active()) {
        logPrint("[BLE Host] Lingering connection attempt detected, cancelling...");
        ble_gap_conn_cancel();
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    // Dynamic Yield: Shift PCs to Standby profile while connecting keyboard to ensure 100% collision-free link creation
    // 650ms allows connected Windows/macOS hosts to complete their LLCP interval exchange before initiating central connection
    updateKvmPowerAndRateProfiles("", true);
    vTaskDelay(pdMS_TO_TICKS(650));

    logPrint("[BLE Host GAP Status] conn_active=%d, disc_active=%d, adv_active=%d",
             ble_gap_conn_active(), ble_gap_disc_active(), ble_gap_adv_active());

    NimBLEAdvertisedDevice* targetKbAdv = advKbDevice;
    advKbDevice = nullptr;

    NimBLEAddress kbAddr;
    uint8_t kbAddrType = BLE_ADDR_PUBLIC;
    bool hasExactAddrType = false;
    if (targetKbAdv) {
        kbAddr = targetKbAdv->getAddress();
        kbAddrType = targetKbAdv->getAddressType();
        hasExactAddrType = true;
    } else if (targetKeyboardMac.length() > 0) {
        kbAddr = NimBLEAddress(targetKeyboardMac.c_str(), BLE_ADDR_PUBLIC);
        kbAddrType = BLE_ADDR_PUBLIC;
    }

    // Direct Link-Layer Connection with OS-level adaptive retry for transient link states (rc=525, 574, etc.)
    bool connRes = false;
    const int MAX_CONNECT_RETRIES = 12;
    for (int retry = 0; retry < MAX_CONNECT_RETRIES && !connRes; retry++) {
        if (retry > 0) {
            logPrint("[BLE Host] Retrying Keyboard connection (attempt %d/%d)...", retry + 1, MAX_CONNECT_RETRIES);
        }
        logPrint("[BLE Host] Direct Link-Layer Connecting to Keyboard: %s (Type: %d)...", 
                 kbAddr.toString().c_str(), kbAddrType);
        
        // Fast-path: connect via advertised device object (synchronized timing anchor point)
        if (targetKbAdv && retry == 0) {
            connRes = pKbClient->connect(targetKbAdv, false);
        } else {
            connRes = pKbClient->connect(kbAddr, false);
        }

        if (!connRes) {
            int err = pKbClient ? pKbClient->getLastError() : -1;
            if (isTransientConnectionError(err) && retry + 1 < MAX_CONNECT_RETRIES) {
                // Non-harmonic prime backoff (65ms + retry*25ms) prevents lock-step collisions with 7.5ms PC intervals
                uint32_t backoffMs = 65 + (retry * 25);
                logPrint("[BLE Host] Keyboard busy / transient link state (rc=%d: %s), waiting %ums before retry...",
                         err, NimBLEUtils::returnCodeToString(err), backoffMs);
                vTaskDelay(pdMS_TO_TICKS(backoffMs));
            } else if (!hasExactAddrType && targetKeyboardMac.length() > 0) {
                // Blind MAC fallback only if we didn't receive an exact advertisement packet
                uint8_t altType = (kbAddrType == BLE_ADDR_PUBLIC) ? BLE_ADDR_RANDOM : BLE_ADDR_PUBLIC;
                connRes = pKbClient->connect(NimBLEAddress(targetKeyboardMac.c_str(), altType), false);
            }
        }
    }

    if (targetKbAdv) {
        delete targetKbAdv;
        targetKbAdv = nullptr;
    }

    if (!connRes) {
        int errCode = pKbClient ? pKbClient->getLastError() : -1;
        logPrint("[BLE Host] Keyboard connection attempt failed: rc=%d (%s)", 
                 errCode, NimBLEUtils::returnCodeToString(errCode));
        vTaskDelay(pdMS_TO_TICKS(1500));
        isConnectingToKeyboard = false;
        updateKvmPowerAndRateProfiles("", true);
        checkAndResumeAdvertising();
        if (!kbConnected || !mouseConnected) startHostReconnectTask();
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(150));
    logPrint("[BLE Host] Connected to Keyboard! Securing connection (Pairing)...");
    bool kbSecured = false;
    for (int secRetry = 0; secRetry < 6 && !kbSecured; secRetry++) {
        ble_gap_conn_desc d;
        if (ble_gap_conn_find(pKbClient->getConnId(), &d) == 0 && d.sec_state.encrypted) {
            kbSecured = true;
            break;
        }
        if (pKbClient->secureConnection()) {
            kbSecured = true;
            break;
        }
        int err = pKbClient->getLastError();
        logPrint("[BLE Host] Keyboard secureConnection attempt %d/6 failed: rc=%d (%s). Waiting 350ms...",
                 secRetry + 1, err, NimBLEUtils::returnCodeToString(err));
        vTaskDelay(pdMS_TO_TICKS(350));
    }

    if (!kbSecured) {
        logPrint("[BLE Host] ❌ Keyboard link encryption FAILED! Disconnecting to re-attempt cleanly...");
        pKbClient->disconnect();
        vTaskDelay(pdMS_TO_TICKS(500));
        isConnectingToKeyboard = false;
        updateKvmPowerAndRateProfiles("", true);
        checkAndResumeAdvertising();
        if (!kbConnected || !mouseConnected) startHostReconnectTask();
        return false;
    }
    logPrint("[BLE Host] Keyboard connection secured!");

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
                if (KEYBOARD_LOG) logPrint("[BLE KB Char] UUID: %s | Notify: %d | Write: %d | WriteNR: %d",
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
        pKbClient->updateConnParams(16, 20, 20, 210);
        logPrint("[BLE Host] Keyboard FULLY CONNECTED & READY (%d active chars)!", subCount);
        updateKvmPowerAndRateProfiles("", true);
        checkAndResumeAdvertising();
        return true;
    } else {
        logPrint("[BLE Host] HID Service 0x1812 not found on Keyboard.");
        pKbClient->disconnect();
        isConnectingToKeyboard = false;
        updateKvmPowerAndRateProfiles("", true);
        checkAndResumeAdvertising();
        if (!kbConnected || !mouseConnected) startHostReconnectTask();
        return false;
    }
}

bool connectToMouse() {
    if (isScanningForMice) return false;
    if (targetMouseMac.length() == 0 && !advDevice) return false;

    isConnectingToMouse = true;

    if (!pMouseClient) {
        pMouseClient = NimBLEDevice::createClient();
        pMouseClient->setClientCallbacks(new MouseCallbacks());
        pMouseClient->setConnectTimeout(2);
    }
    // Scenario A: Flexible initial connection interval (15..30ms) for 100% reliable connection without slot collision.
    // MX Master 3S autonomously sends L2CAP Connection Parameter Update Request to 7.50ms (min=6, max=6, lat=44)
    // right after pairing and encryption complete.
    pMouseClient->setConnectionParams(12, 24, 0, 216, 16, 4);

    if (pMouseClient->isConnected()) {
        mouseConnected = true;
        isConnectingToMouse = false;
        return true;
    }

    if (!advDevice && targetMouseMac.length() > 0) {
        logPrint("[BLE Host] Performing targeted fast probe scan for mouse (%s)...", targetMouseMac.c_str());
        NimBLEScan* pScan = NimBLEDevice::getScan();
        if (pScan) {
            pScan->setActiveScan(true);
            pScan->setInterval(48); // 30ms interval
            pScan->setWindow(24);   // 15ms window (50% duty cycle)
            NimBLEScanResults results = pScan->start(2, false);
            for (int i = 0; i < results.getCount(); i++) {
                NimBLEAdvertisedDevice dev = results.getDevice(i);
                String devMac = dev.getAddress().toString().c_str();
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

    NimBLEScan* pScan = NimBLEDevice::getScan();
    if (pScan && pScan->isScanning()) {
        pScan->stop();
    }
    if (ble_gap_disc_active()) {
        ble_gap_disc_cancel();
    }
    if (NimBLEDevice::getAdvertising() && NimBLEDevice::getAdvertising()->isAdvertising()) {
        NimBLEDevice::getAdvertising()->stop();
        logPrint("[BLE Host] Advertising STOPPED");
    }
    if (ble_gap_adv_active()) {
        ble_gap_adv_stop();
    }
    if (ble_gap_conn_active()) {
        logPrint("[BLE Host] Lingering connection attempt detected, cancelling...");
        ble_gap_conn_cancel();
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    // Dynamic Yield: Shift PCs to Standby profile while connecting mouse to ensure collision-free link creation
    updateKvmPowerAndRateProfiles("", true);
    vTaskDelay(pdMS_TO_TICKS(650));

    logPrint("[BLE Host GAP Status] conn_active=%d, disc_active=%d, adv_active=%d",
             ble_gap_conn_active(), ble_gap_disc_active(), ble_gap_adv_active());

    NimBLEAddress mouseAddr;
    uint8_t mouseAddrType = BLE_ADDR_PUBLIC;
    bool hasExactMouseAddrType = false;
    NimBLEAdvertisedDevice* targetAdv = advDevice; // Keep pointer for direct connect
    advDevice = nullptr; // Clear global ownership

    if (targetAdv) {
        mouseAddr = targetAdv->getAddress();
        mouseAddrType = targetAdv->getAddressType();
        hasExactMouseAddrType = true;
    } else if (targetMouseMac.length() > 0) {
        mouseAddr = NimBLEAddress(targetMouseMac.c_str(), BLE_ADDR_PUBLIC);
        mouseAddrType = BLE_ADDR_PUBLIC;
    }

    // Direct Link-Layer Connection with OS-level adaptive phase-sliding retry for transient link states (rc=525, 574, etc.)
    bool connRes = false;
    const int MAX_CONNECT_RETRIES = 12;
    for (int retry = 0; retry < MAX_CONNECT_RETRIES && !connRes; retry++) {
        if (retry > 0) {
            logPrint("[BLE Host] Retrying Mouse connection (attempt %d/%d)...", retry + 1, MAX_CONNECT_RETRIES);
        }
        logPrint("[BLE Host] Direct Link-Layer Connecting to Mouse: %s (Type: %d)...", 
                 mouseAddr.toString().c_str(), mouseAddrType);
        
        // Fast-path: connect via advertised device object (synchronized timing anchor point)
        if (targetAdv && retry == 0) {
            connRes = pMouseClient->connect(targetAdv, false);
        } else {
            connRes = pMouseClient->connect(mouseAddr, false);
        }
        if (!connRes) {
            int err = pMouseClient ? pMouseClient->getLastError() : -1;
            if (isTransientConnectionError(err) && retry + 1 < MAX_CONNECT_RETRIES) {
                // Non-harmonic prime backoff (65ms + retry*25ms) prevents lock-step collisions with 7.5ms PC intervals
                uint32_t backoffMs = 65 + (retry * 25);
                logPrint("[BLE Host] Mouse busy / transient link state (rc=%d: %s), waiting %ums before retry...",
                         err, NimBLEUtils::returnCodeToString(err), backoffMs);
                vTaskDelay(pdMS_TO_TICKS(backoffMs));
            } else if (!hasExactMouseAddrType && targetMouseMac.length() > 0) {
                // Blind MAC fallback only if we didn't receive an exact advertisement packet
                uint8_t altType = (mouseAddrType == BLE_ADDR_PUBLIC) ? BLE_ADDR_RANDOM : BLE_ADDR_PUBLIC;
                connRes = pMouseClient->connect(NimBLEAddress(targetMouseMac.c_str(), altType), false);
            }
        }
    }

    if (targetAdv) {
        delete targetAdv;
        targetAdv = nullptr;
    }

    if (!connRes) {
        int errCode = pMouseClient ? pMouseClient->getLastError() : -1;
        logPrint("[BLE Host] Connection attempt failed: rc=%d (%s)", 
                 errCode, NimBLEUtils::returnCodeToString(errCode));
        vTaskDelay(pdMS_TO_TICKS(1500));
        isConnectingToMouse = false;
        updateKvmPowerAndRateProfiles("", true);
        if (!kbConnected || !mouseConnected) startHostReconnectTask();
        return false;
    }

    logPrint("[BLE Host] Connected to mouse! Securing connection (Pairing)...");
    bool mouseSecured = false;
    for (int secRetry = 0; secRetry < 6 && !mouseSecured; secRetry++) {
        ble_gap_conn_desc d;
        if (ble_gap_conn_find(pMouseClient->getConnId(), &d) == 0 && d.sec_state.encrypted) {
            mouseSecured = true;
            break;
        }
        if (pMouseClient->secureConnection()) {
            mouseSecured = true;
            break;
        }
        int err = pMouseClient->getLastError();
        logPrint("[BLE Host] Mouse secureConnection attempt %d/6 failed: rc=%d (%s). Waiting 350ms...",
                 secRetry + 1, err, NimBLEUtils::returnCodeToString(err));
        vTaskDelay(pdMS_TO_TICKS(350));
    }

    if (!mouseSecured) {
        logPrint("[BLE Host] ❌ Mouse link encryption FAILED! Disconnecting to re-attempt cleanly...");
        pMouseClient->disconnect();
        vTaskDelay(pdMS_TO_TICKS(500));
        isConnectingToMouse = false;
        updateKvmPowerAndRateProfiles("", true);
        if (!kbConnected || !mouseConnected) startHostReconnectTask();
        return false;
    }
    logPrint("[BLE Host] Mouse connection secured!");

    NimBLERemoteService* pService = pMouseClient->getService(hidServiceUUID);
    if (pService != nullptr) {
        std::vector<NimBLERemoteCharacteristic*>* pChars = pService->getCharacteristics(true);
        for (auto &pChar : *pChars) {
            if (pChar->getUUID() == reportCharUUID) {
                if(pChar->canNotify()) {
                    pChar->subscribe(true, MouseNotifyCallback, true); // Synchronous: waits for ATT_WRITE_RSP confirmation
                    logPrint("[BLE Host] Subscribed to HID report (Acknowledged)!");
                }
            }
        }
        ble_gap_set_prefered_le_phy(pMouseClient->getConnId(), BLE_GAP_LE_PHY_2M_MASK | BLE_GAP_LE_PHY_1M_MASK, BLE_GAP_LE_PHY_2M_MASK | BLE_GAP_LE_PHY_1M_MASK, 0);
        checkAndLogPhyStatus(pMouseClient->getConnId(), "Mouse");
        // Scenario B: Explicitly ensure mouse is at 7.50ms Turbo (itvl 6..6, lat 44)
        ble_gap_conn_desc mouseDesc;
        if (ble_gap_conn_find(pMouseClient->getConnId(), &mouseDesc) == 0 && mouseDesc.conn_itvl != 6) {
            logPrint("[BLE Host] Explicitly enforcing 7.50ms Turbo for Mouse (current itvl: %d)...", mouseDesc.conn_itvl);
            pMouseClient->updateConnParams(6, 6, 44, 216);
        }
    } else {
        pMouseClient->disconnect();
        isConnectingToMouse = false;
        updateKvmPowerAndRateProfiles("", true);
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
    updateSmartAccumulatorProfile();
    if (!kbConnected) startHostReconnectTask();
    return true;
}

uint16_t getBleMouseConnItvl() {
    if (pMouseClient && pMouseClient->isConnected()) {
        ble_gap_conn_desc d;
        if (ble_gap_conn_find(pMouseClient->getConnId(), &d) == 0) {
            return d.conn_itvl;
        }
    }
    return 6; // default 7.50ms
}

void disconnectMouse() {
    if (pMouseClient && pMouseClient->isConnected()) {
        pMouseClient->disconnect();
    }
}

void disconnectKeyboard() {
    if (pKbClient && pKbClient->isConnected()) {
        pKbClient->disconnect();
    }
}

void triggerDeviceDiscoveryScan() {
    scannedMiceDoc.clear();
    scannedMiceDoc.to<JsonArray>();

    isScanningForMice = true;

    disconnectMouse();
    disconnectKeyboard();
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
}

uint16_t getBleMouseConnHandle() {
    return (mouseConnected && pMouseClient && pMouseClient->isConnected()) ? pMouseClient->getConnId() : BLE_HS_CONN_HANDLE_NONE;
}

uint16_t getBleKeyboardConnHandle() {
    return (kbConnected && pKbClient && pKbClient->isConnected()) ? pKbClient->getConnId() : BLE_HS_CONN_HANDLE_NONE;
}

