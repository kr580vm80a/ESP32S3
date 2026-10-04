#include "cmd_processor.h"
#include "kvm_config.h"
#include "usb_manager.h"
#include "ble_server.h"
#include "ble_central.h"
#include "logi_bolt.h"
#include "usb_device_engine.h"
#include <ArduinoJson.h>
#include "mbedtls/sha256.h"

static String bleRxBuffer = "";

void ConfigTxCallbacks::onSubscribe(NimBLECharacteristic* pCharacteristic, ble_gap_conn_desc* desc, uint16_t subValue) {
    if (subValue > 0 && desc) {
        markClientAsWebConfig(desc->conn_handle);
    } else if (subValue == 0) {
        logPrint("[BLE Server] 🟣 Web client unsubscribed from Config TX notifications.");
        deactivateWebServiceMode();
    }
}

void ConfigRxCallbacks::onWrite(NimBLECharacteristic* pCharacteristic, ble_gap_conn_desc* desc) {
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

void processCommand(String input, bool isBleSource) {
    input.trim();
    if (input.length() == 0) return;

    lastConfigActivityTime = millis();

    if (input.equalsIgnoreCase("CLOSE_WEB")) {
        logPrint("[BLE Server] 🟣 Web client sent CLOSE_WEB. Shutting down Web Service.");
        isWebBleAuthenticated = false;
        currentAuthNonce = "";
        deactivateWebServiceMode();
        sendConfigResponse("OK_DISCONNECTED");
        return;
    }

    if (input.equalsIgnoreCase("PING")) {
        sendConfigResponse("PONG");
        return;
    }

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
            scheduleSaveConfig(jsonStr);
        }
    } else if (input == "GET_CONFIG") {
        String unifiedJson = buildConfigJson();
        sendConfigResponse("CONFIG " + String(unifiedJson.length()) + " " + unifiedJson);
    } else if (input == "SCAN_DEVICES") {
        triggerDeviceDiscoveryScan();
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

        disconnectMouse();
        if (targetMouseMac.length() > 0) {
            doConnectMouse = true;
        }
    } else if (input == "UNBIND_MOUSE") {
        saveMouseToNvsLayout("", "");
        disconnectMouse();
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

        disconnectKeyboard();
        if (targetKeyboardMac.length() > 0) {
            doConnectKeyboard = true;
        }
    } else if (input == "UNBIND_KEYBOARD") {
        saveKeyboardToNvsLayout("", "");
        disconnectKeyboard();
        sendConfigResponse("OK_UNBIND_KEYBOARD");
    } else if (input == "GET_TARGET_KEYBOARD") {
        sendConfigResponse("TARGET_KEYBOARD " + targetKeyboardMac);
    } else if (input.startsWith("DUMP")) {
        preferences.begin(NVS_NAMESPACE, true);
        int actId = preferences.getInt(NVS_KEY_ACT_LAYOUT_ID, 1);
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
        logPrint("  %s: '%s' (%s)", NVS_KEY_MOUSE_MAC, mMac.c_str(), mName.c_str());
        logPrint("  %s: '%s' (%s)", NVS_KEY_KB_MAC, kMac.c_str(), kName.c_str());
        logPrint("  %s (len %d): %s", NVS_KEY_LAYOUTS, layJson.length(), layJson.c_str());
        logPrint("  %s (len %d): %s", NVS_KEY_CLIENTS, cliJson.length(), cliJson.c_str());
        logPrint("--- [END NVS FLASH DUMP] ---");

        logPrint("--- [CONNECTED DEVICES STATUS] ---");

        // 1. USB-C PC Connection
        if (usb_device_is_connected()) {
            int uOs = usb_device_get_detected_os();
            const char* osStr = (uOs == OS_MAC) ? "macOS" : (uOs == OS_ANDROID ? "Android" : "Windows");
            String boundMac = usb_device_get_bound_mac();
            logPrint("[USB-C PC] Connected (1000Hz HID, Latency: 0) | Bound MAC: %s | Detected OS: %s",
                     boundMac.length() > 0 ? boundMac.c_str() : "None", osStr);
        } else {
            logPrint("[USB-C PC] Disconnected");
        }

        // 2. Logitech Bolt USB Receiver
        if (logi_bolt_is_device_attached()) {
            bool boltMouse = logi_bolt_is_mouse_connected();
            bool boltKb = logi_bolt_is_keyboard_connected();
            logPrint("[Logi Bolt Dongle] Attached (Host Port) | Mouse: %s | Keyboard: %s",
                     boltMouse ? "Connected (Wireless)" : "Disconnected",
                     boltKb ? "Connected (Wireless)" : "Disconnected");
        } else {
            logPrint("[Logi Bolt Dongle] Not attached");
        }

        // 3. BLE Peripherals (Mouse & Keyboard)
        logPrint("[BLE Peripherals]");
        uint16_t mHandle = getBleMouseConnHandle();
        if (mHandle != BLE_HS_CONN_HANDLE_NONE) {
            ble_gap_conn_desc mDesc;
            if (ble_gap_conn_find(mHandle, &mDesc) == 0) {
                uint8_t txPhy = 0, rxPhy = 0;
                ble_gap_read_le_phy(mHandle, &txPhy, &rxPhy);
                logPrint("  Mouse: %s ('%s') | conn: %d | Itvl: %.2f ms (%d) | Latency: %d | Timeout: %d ms | PHY: TX %s / RX %s | Sec: Enc=%d, Bond=%d",
                         targetMouseMac.c_str(), targetMouseName.c_str(), mHandle,
                         mDesc.conn_itvl * 1.25f, mDesc.conn_itvl,
                         mDesc.conn_latency,
                         mDesc.supervision_timeout * 10,
                         txPhy == 2 ? "2M" : (txPhy == 1 ? "1M" : "CODED"),
                         rxPhy == 2 ? "2M" : (rxPhy == 1 ? "1M" : "CODED"),
                         mDesc.sec_state.encrypted, mDesc.sec_state.bonded);
            } else {
                logPrint("  Mouse: %s ('%s') | conn: %d | Connected", targetMouseMac.c_str(), targetMouseName.c_str(), mHandle);
            }
        } else {
            logPrint("  Mouse: %s ('%s') | Disconnected", 
                     targetMouseMac.length() > 0 ? targetMouseMac.c_str() : "None",
                     targetMouseName.length() > 0 ? targetMouseName.c_str() : "");
        }

        uint16_t kHandle = getBleKeyboardConnHandle();
        if (kHandle != BLE_HS_CONN_HANDLE_NONE) {
            ble_gap_conn_desc kDesc;
            if (ble_gap_conn_find(kHandle, &kDesc) == 0) {
                uint8_t txPhy = 0, rxPhy = 0;
                ble_gap_read_le_phy(kHandle, &txPhy, &rxPhy);
                logPrint("  Keyboard: %s ('%s') | conn: %d | Itvl: %.2f ms (%d) | Latency: %d | Timeout: %d ms | PHY: TX %s / RX %s | Sec: Enc=%d, Bond=%d",
                         targetKeyboardMac.c_str(), targetKeyboardName.c_str(), kHandle,
                         kDesc.conn_itvl * 1.25f, kDesc.conn_itvl,
                         kDesc.conn_latency,
                         kDesc.supervision_timeout * 10,
                         txPhy == 2 ? "2M" : (txPhy == 1 ? "1M" : "CODED"),
                         rxPhy == 2 ? "2M" : (rxPhy == 1 ? "1M" : "CODED"),
                         kDesc.sec_state.encrypted, kDesc.sec_state.bonded);
            } else {
                logPrint("  Keyboard: %s ('%s') | conn: %d | Connected", targetKeyboardMac.c_str(), targetKeyboardName.c_str(), kHandle);
            }
        } else {
            logPrint("  Keyboard: %s ('%s') | Disconnected", 
                     targetKeyboardMac.length() > 0 ? targetKeyboardMac.c_str() : "None",
                     targetKeyboardName.length() > 0 ? targetKeyboardName.c_str() : "");
        }

        // 4. BLE PC Clients
        logPrint("[BLE PC Clients]");
        int connectedPcCount = 0;
        for (int i = 0; i < MAX_SUPPORTED_KVM_CLIENTS; i++) {
            if (kvmClients[i].active && kvmClients[i].conn_id != BLE_HS_CONN_HANDLE_NONE) {
                connectedPcCount++;
                ble_gap_conn_desc cDesc;
                bool hasDesc = (ble_gap_conn_find(kvmClients[i].conn_id, &cDesc) == 0);
                uint8_t txPhy = 0, rxPhy = 0;
                ble_gap_read_le_phy(kvmClients[i].conn_id, &txPhy, &rxPhy);
                bool inLayout = isMacInActiveLayout(kvmClients[i].mac);

                if (hasDesc) {
                    logPrint("  PC #%d: %s | conn: %d | Layout: %s | Mode: %s | Itvl: %.2f ms (%d) | Latency: %d | Timeout: %d ms | PHY: TX %s / RX %s | Sec: Enc=%d, Bond=%d",
                             i + 1, kvmClients[i].mac.c_str(), kvmClients[i].conn_id,
                             inLayout ? "ACTIVE" : "NON-LAYOUT",
                             kvmClients[i].isTurbo ? "TURBO ⚡" : "STANDBY",
                             cDesc.conn_itvl * 1.25f, cDesc.conn_itvl,
                             cDesc.conn_latency,
                             cDesc.supervision_timeout * 10,
                             txPhy == 2 ? "2M" : (txPhy == 1 ? "1M" : "CODED"),
                             rxPhy == 2 ? "2M" : (rxPhy == 1 ? "1M" : "CODED"),
                             cDesc.sec_state.encrypted, cDesc.sec_state.bonded);
                } else {
                    logPrint("  PC #%d: %s | conn: %d | Layout: %s | Mode: %s",
                             i + 1, kvmClients[i].mac.c_str(), kvmClients[i].conn_id,
                             inLayout ? "ACTIVE" : "NON-LAYOUT",
                             kvmClients[i].isTurbo ? "TURBO ⚡" : "STANDBY");
                }
            }
        }
        if (connectedPcCount == 0) {
            logPrint("  None connected");
        }

        // 5. Non-KVM / Web Bluetooth Clients
        for (int i = 0; i < MAX_NON_KVM_CLIENTS; i++) {
            if (nonKvmClients[i].conn_id != BLE_HS_CONN_HANDLE_NONE) {
                ble_gap_conn_desc nDesc;
                bool hasDesc = (ble_gap_conn_find(nonKvmClients[i].conn_id, &nDesc) == 0);
                if (hasDesc) {
                    logPrint("  Non-KVM: %s | conn: %d (%s) | Itvl: %.2f ms | Latency: %d | Timeout: %d ms",
                             nonKvmClients[i].mac.c_str(), nonKvmClients[i].conn_id,
                             nonKvmClients[i].isWebConfig ? "Web Config" : "Other",
                             nDesc.conn_itvl * 1.25f, nDesc.conn_latency, nDesc.supervision_timeout * 10);
                } else {
                    logPrint("  Non-KVM: %s | conn: %d (%s)",
                             nonKvmClients[i].mac.c_str(), nonKvmClients[i].conn_id,
                             nonKvmClients[i].isWebConfig ? "Web Config" : "Other");
                }
            }
        }
        logPrint("--- [END CONNECTED DEVICES STATUS] ---");
    } else if (input.startsWith("CLEAR_BONDS")) {
        int count = NimBLEDevice::getNumBonds();
        logPrint("[BLE] CLEAR_BONDS requested. Found %d bonded device(s).", count);

        // 1. Disconnect all active BLE PC clients and peripherals to flush connection handles
        NimBLEServer* pServer = NimBLEDevice::getServer();
        if (pServer) {
            for (int i = 0; i < MAX_SUPPORTED_KVM_CLIENTS; i++) {
                if (kvmClients[i].active && kvmClients[i].conn_id != BLE_HS_CONN_HANDLE_NONE) {
                    logPrint("[BLE]   -> Disconnecting PC %s (conn: %d)", kvmClients[i].mac.c_str(), kvmClients[i].conn_id);
                    pServer->disconnect(kvmClients[i].conn_id);
                    kvmClients[i].active = false;
                    kvmClients[i].conn_id = BLE_HS_CONN_HANDLE_NONE;
                }
            }
        }
        disconnectMouse();
        disconnectKeyboard();
        vTaskDelay(pdMS_TO_TICKS(100));

        // 2. Wipe NimBLE NVS security bond database cleanly
        NimBLEDevice::deleteAllBonds();

        logPrint("[BLE] ✅ CLEAR_BONDS complete! Deleted %d bond(s). Restarting ESP32 in 500ms...", count);
        Serial.printf("OK_CLEAR_BONDS %d\n", count);
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_restart();
    }
}
