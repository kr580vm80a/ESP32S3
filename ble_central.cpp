#include "ble_central.h"
#include "ble_server.h"
#include "cursor_engine.h"
#include "keyboard_engine.h"
#include "cmd_processor.h"
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
            NimBLEDevice::getScan()->stop();
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
            NimBLEDevice::getScan()->stop();
            advKbDevice = new NimBLEAdvertisedDevice(*advertisedDevice);
            doConnectKeyboard = true;
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
        updateKvmPowerAndRateProfiles("", false);
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
        pKbClient->setConnectTimeout(2);
    }
    // 50% Initiator Duty Cycle (scanInterval=32 (20ms), scanWindow=16 (10ms)) - matched with mouse
    // Previous (v45-v48): pKbClient->setConnectionParams(6, 24, 0, 216, 32, 8);
    pKbClient->setConnectionParams(6, 24, 0, 216, 32, 16);

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
    }
    if (ble_gap_adv_active()) {
        ble_gap_adv_stop();
    }
    if (ble_gap_conn_active()) {
        logPrint("[BLE Host] Lingering connection attempt detected, cancelling...");
        ble_gap_conn_cancel();
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    logPrint("[BLE Host GAP Status] conn_active=%d, disc_active=%d, adv_active=%d",
             ble_gap_conn_active(), ble_gap_disc_active(), ble_gap_adv_active());

    NimBLEAddress kbAddr;
    uint8_t kbAddrType = BLE_ADDR_PUBLIC;
    bool hasExactAddrType = false;
    if (advKbDevice) {
        kbAddr = advKbDevice->getAddress();
        kbAddrType = advKbDevice->getAddressType();
        hasExactAddrType = true;
        delete advKbDevice;
        advKbDevice = nullptr;
    } else if (targetKeyboardMac.length() > 0) {
        kbAddr = NimBLEAddress(targetKeyboardMac.c_str(), BLE_ADDR_PUBLIC);
        kbAddrType = BLE_ADDR_PUBLIC;
    }

    // Direct Link-Layer Connection with OS-level adaptive retry for transient link states (rc=525, 574, etc.)
    bool connRes = false;
    const int MAX_CONNECT_RETRIES = 5;
    for (int retry = 0; retry < MAX_CONNECT_RETRIES && !connRes; retry++) {
        if (retry > 0) {
            logPrint("[BLE Host] Retrying Keyboard connection (attempt %d/%d)...", retry + 1, MAX_CONNECT_RETRIES);
        }
        logPrint("[BLE Host] Direct Link-Layer Connecting to Keyboard: %s (Type: %d)...", 
                 kbAddr.toString().c_str(), kbAddrType);
        connRes = pKbClient->connect(kbAddr, false);
        if (!connRes) {
            int err = pKbClient ? pKbClient->getLastError() : -1;
            if (isTransientConnectionError(err) && retry + 1 < MAX_CONNECT_RETRIES) {
                logPrint("[BLE Host] Keyboard busy / transient link state (rc=%d: %s), waiting 400ms before retry...",
                         err, NimBLEUtils::returnCodeToString(err));
                vTaskDelay(pdMS_TO_TICKS(400));
            } else if (!hasExactAddrType && targetKeyboardMac.length() > 0) {
                // Blind MAC fallback only if we didn't receive an exact advertisement packet
                uint8_t altType = (kbAddrType == BLE_ADDR_PUBLIC) ? BLE_ADDR_RANDOM : BLE_ADDR_PUBLIC;
                connRes = pKbClient->connect(NimBLEAddress(targetKeyboardMac.c_str(), altType), false);
            }
        }
    }

    if (!connRes) {
        int errCode = pKbClient ? pKbClient->getLastError() : -1;
        logPrint("[BLE Host] Keyboard connection attempt failed: rc=%d (%s)", 
                 errCode, NimBLEUtils::returnCodeToString(errCode));
        vTaskDelay(pdMS_TO_TICKS(1500));
        isConnectingToKeyboard = false;
        checkAndResumeAdvertising();
        if (!kbConnected || !mouseConnected) startHostReconnectTask();
        return false;
    }

    // Native Logitech MX Keys S PPCP profile: 20.00..25.00 ms (itvl: 16..20), Latency 20, Timeout 2100 ms (210)
    // Previous (v46): pKbClient->updateConnParams(16, 24, 20, 500);
    pKbClient->updateConnParams(16, 20, 20, 210);
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
        logPrint("[BLE Host] Keyboard FULLY CONNECTED & READY (%d active chars)!", subCount);
        checkAndResumeAdvertising();
        return true;
    } else {
        logPrint("[BLE Host] HID Service 0x1812 not found on Keyboard.");
        pKbClient->disconnect();
        isConnectingToKeyboard = false;
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
    // Variant 1: 50% Initiator Duty Cycle (scanInterval=32 (20ms), scanWindow=16 (10ms))
    // Frees 50% radio airtime for ESP32 BT controller to service active PC links (Mac/Windows)
    // without rejecting connection initiation with HCI 0x0D (rc=525: Limited Resources)
    pMouseClient->setConnectionParams(6, 24, 0, 216, 32, 16);

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
    }
    if (ble_gap_adv_active()) {
        ble_gap_adv_stop();
    }
    if (ble_gap_conn_active()) {
        logPrint("[BLE Host] Lingering connection attempt detected, cancelling...");
        ble_gap_conn_cancel();
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    logPrint("[BLE Host GAP Status] conn_active=%d, disc_active=%d, adv_active=%d",
             ble_gap_conn_active(), ble_gap_disc_active(), ble_gap_adv_active());

    NimBLEAddress mouseAddr;
    uint8_t mouseAddrType = BLE_ADDR_PUBLIC;
    bool hasExactMouseAddrType = false;
    if (advDevice) {
        mouseAddr = advDevice->getAddress();
        mouseAddrType = advDevice->getAddressType();
        hasExactMouseAddrType = true;
        delete advDevice;
        advDevice = nullptr;
    } else if (targetMouseMac.length() > 0) {
        mouseAddr = NimBLEAddress(targetMouseMac.c_str(), BLE_ADDR_PUBLIC);
        mouseAddrType = BLE_ADDR_PUBLIC;
    }

    // Direct Link-Layer Connection with OS-level adaptive retry for transient link states (rc=525, 574, etc.)
    bool connRes = false;
    const int MAX_CONNECT_RETRIES = 5;
    for (int retry = 0; retry < MAX_CONNECT_RETRIES && !connRes; retry++) {
        if (retry > 0) {
            logPrint("[BLE Host] Retrying Mouse connection (attempt %d/%d)...", retry + 1, MAX_CONNECT_RETRIES);
        }
        logPrint("[BLE Host] Direct Link-Layer Connecting to Mouse: %s (Type: %d)...", 
                 mouseAddr.toString().c_str(), mouseAddrType);
        connRes = pMouseClient->connect(mouseAddr, false);
        if (!connRes) {
            int err = pMouseClient ? pMouseClient->getLastError() : -1;
            if (isTransientConnectionError(err) && retry + 1 < MAX_CONNECT_RETRIES) {
                logPrint("[BLE Host] Mouse busy / transient link state (rc=%d: %s), waiting 150ms before retry...",
                         err, NimBLEUtils::returnCodeToString(err));
                vTaskDelay(pdMS_TO_TICKS(150));
            } else if (!hasExactMouseAddrType && targetMouseMac.length() > 0) {
                // Blind MAC fallback only if we didn't receive an exact advertisement packet
                uint8_t altType = (mouseAddrType == BLE_ADDR_PUBLIC) ? BLE_ADDR_RANDOM : BLE_ADDR_PUBLIC;
                connRes = pMouseClient->connect(NimBLEAddress(targetMouseMac.c_str(), altType), false);
            }
        }
    }

    if (!connRes) {
        int errCode = pMouseClient ? pMouseClient->getLastError() : -1;
        logPrint("[BLE Host] Connection attempt failed: rc=%d (%s)", 
                 errCode, NimBLEUtils::returnCodeToString(errCode));
        vTaskDelay(pdMS_TO_TICKS(1500));
        isConnectingToMouse = false;
        if (!kbConnected || !mouseConnected) startHostReconnectTask();
        return false;
    }

    logPrint("[BLE Host] Connected! Securing connection (Pairing)...");
    if (!pMouseClient->secureConnection()) {
        logPrint("[BLE Host] Initial secureConnection failed. Retrying in 100ms...");
        delay(100);
        if (!pMouseClient->secureConnection()) {
            logPrint("[BLE Host] Secure connection retry failed. Proceeding with service discovery...");
        } else {
            logPrint("[BLE Host] Connection secured on retry!");
        }
    } else {
        logPrint("[BLE Host] Connection secured!");
    }

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
        // The mouse automatically requests its preferred connection parameters (interval, latency, timeout)
        // via L2CAP update request right after pairing/encryption. Our adaptive stack in ble_gap.c
        // automatically honors the peripheral's requested latency while finding the fastest stable interval.
    } else {
        pMouseClient->disconnect();
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
        updateKvmPowerAndRateProfiles(monitors[currentMonitorIndex].mac, false);
    }
    checkAndResumeAdvertising();
    if (!kbConnected) startHostReconnectTask();
    return true;
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
