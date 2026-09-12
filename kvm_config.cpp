#include "kvm_config.h"

static String pendingSaveJson = "";
static bool doSaveConfig = false;

String readNvsBlob(Preferences& pref, const char* key, const String& fallback) {
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

void saveKvmClientsToPreferences() {
    preferences.begin(NVS_NAMESPACE, false);
    JsonDocument doc;
    JsonArray docClients = doc.to<JsonArray>();

    for (int i = 0; i < MAX_SUPPORTED_KVM_CLIENTS; i++) {
        String mac = kvmClients[i].mac;
        if (mac.length() == 0) continue;
        JsonObject client = docClients.add<JsonObject>();
        client["mac"] = mac;
        client["name"] = (kvmClients[i].name.length() > 0) ? kvmClients[i].name : "Paired Device";
    }

    String clientsJson;
    serializeJson(doc, clientsJson);
    preferences.remove(NVS_KEY_CLIENTS);
    preferences.putBytes(NVS_KEY_CLIENTS, clientsJson.c_str(), clientsJson.length() + 1);
    preferences.end();

    logPrint("[NVS] 💾 Successfully saved %d KVM client(s) to preferences (%d bytes): %s",
             docClients.size(), clientsJson.length() + 1, clientsJson.c_str());
}

void syncOrphanBonds() {
    int numBonds = NimBLEDevice::getNumBonds();
    for (int b = numBonds - 1; b >= 0; b--) {
        NimBLEAddress bondAddr = NimBLEDevice::getBondedAddress(b);
        String mac = bondAddr.toString().c_str();
        if (targetMouseMac.length() > 0 && mac.equals(targetMouseMac)) continue;
        if (targetKeyboardMac.length() > 0 && mac.equals(targetKeyboardMac)) continue;
        bool found = false;
        for (int k = 0; k < MAX_SUPPORTED_KVM_CLIENTS; k++) {
            if (kvmClients[k].mac.equals(mac)) {
                found = true;
                break;
            }
        }
        if (!found) {
            NimBLEDevice::deleteBond(bondAddr);
            logPrint("[BLE SYNC]   -> Deleted orphan bond: %s", mac.c_str());
        }
    }
}

String buildConfigJson() {
    preferences.begin(NVS_NAMESPACE, true);
    int activeLayoutId = preferences.getInt(NVS_KEY_ACT_LAYOUT_ID, 1);
    String mouseMac = preferences.getString(NVS_KEY_MOUSE_MAC, targetMouseMac);
    String mouseName = preferences.getString(NVS_KEY_MOUSE_NAME, targetMouseName);
    String kbMac = preferences.getString(NVS_KEY_KB_MAC, targetKeyboardMac);
    String kbName = preferences.getString(NVS_KEY_KB_NAME, targetKeyboardName);
    String layoutsJson = readNvsBlob(preferences, NVS_KEY_LAYOUTS, "[]");
    String clientsJson = readNvsBlob(preferences, NVS_KEY_CLIENTS, "[]");
    preferences.end();

    JsonDocument doc;
    doc["activeLayoutId"] = activeLayoutId;
    deserializeJson(doc["layouts"], layoutsJson);
    if (!doc["layouts"].is<JsonArray>()) {
        doc["layouts"].to<JsonArray>();
    }

    deserializeJson(doc["clients"], clientsJson);
    if (!doc["clients"].is<JsonArray>()) {
        doc["clients"].to<JsonArray>();
    }

    doc["mouseMac"] = mouseMac;
    doc["mouseName"] = mouseName;
    doc["keyboardMac"] = kbMac;
    doc["keyboardName"] = kbName;

    // Update connected status for clients based on live kvmClients[]
    JsonArray clientsArr = doc["clients"].as<JsonArray>();
    for (JsonObject c : clientsArr) {
        c["connected"] = false;
    }
    for (int i = 0; i < MAX_SUPPORTED_KVM_CLIENTS; i++) {
        if (kvmClients[i].active && kvmClients[i].mac.length() > 0) {
            String activeMac = kvmClients[i].mac;
            bool exists = false;
            for (JsonObject c : clientsArr) {
                String cMac = c["mac"] | "";
                if (cMac.equals(activeMac)) {
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

void loadConfiguration() {
    preferences.begin(NVS_NAMESPACE, true);
    targetMouseMac = preferences.getString(NVS_KEY_MOUSE_MAC, "");
    targetMouseName = preferences.getString(NVS_KEY_MOUSE_NAME, "");
    targetKeyboardMac = preferences.getString(NVS_KEY_KB_MAC, "");
    targetKeyboardName = preferences.getString(NVS_KEY_KB_NAME, "");

    int activeLayoutId = preferences.getInt(NVS_KEY_ACT_LAYOUT_ID, 1);
    String layoutsJson = readNvsBlob(preferences, NVS_KEY_LAYOUTS, "[]");
    String clientsJson = readNvsBlob(preferences, NVS_KEY_CLIENTS, "[]");
    preferences.end();

    JsonDocument docLayouts;
    deserializeJson(docLayouts, layoutsJson);

    JsonArray screens;
    if (docLayouts.is<JsonArray>() && docLayouts.size() > 0) {
        JsonObject activeLayout = docLayouts[0].as<JsonObject>();
        for (JsonObject l : docLayouts.as<JsonArray>()) {
            int lId = l["id"] | 0;
            if (lId == activeLayoutId) {
                activeLayout = l;
                break;
            }
        }
        screens = activeLayout["screens"].as<JsonArray>();
    }

    monitorCount = 0;
    if (screens) {
        for (JsonObject screen : screens) {
            int defId = monitorCount + 1;
            monitors[monitorCount].id = screen["id"] | defId;
            monitors[monitorCount].name = screen["name"] | "";
            monitors[monitorCount].x = screen["x"] | 0;
            monitors[monitorCount].y = screen["y"] | 0;
            monitors[monitorCount].width = screen["width"] | 1920;
            monitors[monitorCount].height = screen["height"] | 1080;
            monitors[monitorCount].mac = screen["mac"] | "";
            monitors[monitorCount].os = screen["os"] | OS_WINDOWS;
            monitors[monitorCount].scale = screen["scale"] | 100;
            monitors[monitorCount].isPrimary = screen["isPrimary"] | false;
            monitors[monitorCount].keepAlive = screen["keepAlive"] | 0;
            monitorCount++;
        }
    }

    JsonDocument docClients;
    deserializeJson(docClients, clientsJson);

    int clientCount = 0;
    if (docClients.is<JsonArray>()) {
        KVMClient kvmClientsOld[MAX_SUPPORTED_KVM_CLIENTS];
        for (int i = 0; i < MAX_SUPPORTED_KVM_CLIENTS; i++) kvmClientsOld[i] = kvmClients[i];

        for (JsonObject client : docClients.as<JsonArray>()) { 
            if (clientCount >= MAX_SUPPORTED_KVM_CLIENTS) break;
            String mac = client["mac"] | "";
            if (mac.length() == 0) continue;
            if (targetMouseMac.length() > 0 && mac.equalsIgnoreCase(targetMouseMac)) continue;
            if (targetKeyboardMac.length() > 0 && mac.equalsIgnoreCase(targetKeyboardMac)) continue;
            uint16_t conn_id = BLE_HS_CONN_HANDLE_NONE;
            bool active = false;
            bool isTurbo = false;
            for (int k = 0; k < MAX_SUPPORTED_KVM_CLIENTS; k++) {
                if (kvmClientsOld[k].mac.equals(mac)) {
                    conn_id = kvmClientsOld[k].conn_id;
                    active = kvmClientsOld[k].active;
                    isTurbo = kvmClientsOld[k].isTurbo;
                    break;
                }
            }
            kvmClients[clientCount].mac = mac;
            kvmClients[clientCount].name = client["name"] | "";
            kvmClients[clientCount].conn_id = conn_id;
            kvmClients[clientCount].active = active;
            kvmClients[clientCount].isTurbo = isTurbo;
            clientCount++;
        }
        int i = clientCount;
        while (i < MAX_SUPPORTED_KVM_CLIENTS) {
            kvmClients[i].mac = "";
            kvmClients[i].name = "";
            kvmClients[i].conn_id = BLE_HS_CONN_HANDLE_NONE;
            kvmClients[i].active = false;
            kvmClients[i].isTurbo = false;
            i++;
        }
    }
    for (int k = 0; k < MAX_SUPPORTED_KVM_CLIENTS; k++) logPrint("    -> %s PC", kvmClients[k].mac.c_str());
    logPrint("Loaded %d monitors, %d KVM PC clients from granular NVS. Mouse: %s (%s) | Keyboard: %s (%s)",
             monitorCount, clientCount, targetMouseMac.c_str(), targetMouseName.c_str(), targetKeyboardMac.c_str(), targetKeyboardName.c_str());
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

void scheduleSaveConfig(const String& json) {
    pendingSaveJson = json;
    doSaveConfig = true;
}

void executePendingSave() {
    if (doSaveConfig && pendingSaveJson.length() > 0) {
        doSaveConfig = false;
        JsonDocument doc;
        if (!deserializeJson(doc, pendingSaveJson)) {
            saveConfiguration(pendingSaveJson);
            loadConfiguration();
            syncOrphanBonds();
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
