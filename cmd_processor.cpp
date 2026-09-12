#include "cmd_processor.h"
#include "kvm_config.h"
#include <ArduinoJson.h>
#include "mbedtls/sha256.h"

static String bleRxBuffer = "";

void ConfigTxCallbacks::onSubscribe(NimBLECharacteristic* pCharacteristic, ble_gap_conn_desc* desc, uint16_t subValue) {
    if (subValue > 0 && desc) {
        markClientAsWebConfig(desc->conn_handle);
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
    } else if (input == "SCAN_MICE" || input == "SCAN_KEYBOARDS" || input == "SCAN_DEVICES") {
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
    } else if (input == "DUMP_FLASH") {
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
    } else if (input == "CLEAR_BONDS") {
        int count = NimBLEDevice::getNumBonds();
        NimBLEDevice::deleteAllBonds();
        logPrint("[BLE] Deleted %d bonded devices from NVS. Fresh pairing required for all PCs.", count);
        sendConfigResponse("OK_CLEAR_BONDS " + String(count));
    }
}
