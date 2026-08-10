#include <Arduino.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <NimBLEDevice.h>
#include <NimBLEHIDDevice.h>

#if CONFIG_IDF_TARGET_ESP32S3
#include <HWCDC.h>
#endif

Preferences preferences;

// NVS Flash Storage Constants
const char* NVS_NAMESPACE = "kvm_config";
const char* NVS_KEY_LAYOUT = "layout";
const char* NVS_KEY_MOUSE_MAC = "mouse_mac";

// Structure to store monitor configuration
struct MonitorConfig {
  String id;
  int x;
  int y;
  int width;
  int height;
  String mac;
};

#define MAX_MONITORS 10
MonitorConfig monitors[MAX_MONITORS];
int monitorCount = 0;

// Virtual Cursor Position
long virtualX = 0;
long virtualY = 0;
int currentMonitorIndex = 0;

// --- BLE Peripheral (Server) Variables ---
NimBLEServer* pServer = nullptr;
NimBLEHIDDevice* hidDevice = nullptr;
NimBLECharacteristic* inputChar = nullptr;

// Active KVM Connections (Mac addresses of connected PCs)
struct KVMClient {
  uint16_t conn_id;
  String mac;
  String name;
  bool active;
};
#define MAX_KVM_CLIENTS 2
KVMClient kvmClients[MAX_KVM_CLIENTS];

const uint8_t hidReportMap[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
    0x09, 0x02,        // Usage (Mouse)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x01,        //   Report ID (1)
    0x09, 0x01,        //   Usage (Pointer)
    0xA1, 0x00,        //   Collection (Physical)
    0x05, 0x09,        //     Usage Page (Button)
    0x19, 0x01,        //     Usage Minimum (0x01)
    0x29, 0x03,        //     Usage Maximum (0x03)
    0x15, 0x00,        //     Logical Minimum (0)
    0x25, 0x01,        //     Logical Maximum (1)
    0x95, 0x03,        //     Report Count (3)
    0x75, 0x01,        //     Report Size (1)
    0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x95, 0x01,        //     Report Count (1)
    0x75, 0x05,        //     Report Size (5)
    0x81, 0x03,        //     Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x05, 0x01,        //     Usage Page (Generic Desktop Ctrls)
    0x09, 0x30,        //     Usage (X)
    0x09, 0x31,        //     Usage (Y)
    0x09, 0x38,        //     Usage (Wheel)
    0x15, 0x81,        //     Logical Minimum (-127)
    0x25, 0x7F,        //     Logical Maximum (127)
    0x75, 0x08,        //     Report Size (8)
    0x95, 0x03,        //     Report Count (3)
    0x81, 0x06,        //     Input (Data,Var,Rel,No Wrap,Linear,Preferred State,No Null Position)
    0x05, 0x0C,        //     Usage Page (Consumer)
    0x0A, 0x38, 0x02,  //     Usage (AC Pan)
    0x15, 0x81,        //     Logical Minimum (-127)
    0x25, 0x7F,        //     Logical Maximum (127)
    0x75, 0x08,        //     Report Size (8)
    0x95, 0x01,        //     Report Count (1)
    0x81, 0x06,        //     Input (Data,Var,Rel,No Wrap,Linear,Preferred State,No Null Position)
    0xC0,              //   End Collection
    0xC0               // End Collection
};

static NimBLEClient* pClient = nullptr;
static String targetMouseMac = "";
static bool isScanningForMice = false;
static bool connected = false;

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) {
        String peerMac = NimBLEAddress(desc->peer_ota_addr).toString().c_str();
        peerMac.toLowerCase();
        peerMac.trim();
        Serial.printf("[BLE Server] PC Connected! MAC: %s (conn_handle: %d | itvl: %d | latency: %d | timeout: %d)\n",
                      peerMac.c_str(), desc->conn_handle, desc->conn_itvl, desc->conn_latency, desc->supervision_timeout);
        
        // Save connection
        bool updated = false;
        for (int i = 0; i < MAX_KVM_CLIENTS; i++) {
            if (kvmClients[i].mac.equalsIgnoreCase(peerMac)) {
                kvmClients[i].conn_id = desc->conn_handle;
                kvmClients[i].active = true;
                updated = true;
                break;
            }
        }
        if (!updated) {
            for (int i = 0; i < MAX_KVM_CLIENTS; i++) {
                if (!kvmClients[i].active) {
                    kvmClients[i].conn_id = desc->conn_handle;
                    kvmClients[i].mac = peerMac;
                    kvmClients[i].active = true;
                    break;
                }
            }
        }
        // Resume advertising so 2nd PC (Mortar) can discover and connect
        int activeCount = 0;
        for (int i = 0; i < MAX_KVM_CLIENTS; i++) {
            if (kvmClients[i].active) activeCount++;
        }
        if (activeCount < MAX_KVM_CLIENTS) {
            xTaskCreate([](void* param) {
                vTaskDelay(pdMS_TO_TICKS(1500));
                if (NimBLEDevice::getAdvertising()) {
                    Serial.println("[BLE Server] Resuming advertising for additional PC...");
                    NimBLEDevice::getAdvertising()->start();
                }
                vTaskDelete(NULL);
            }, "bgAdvTask", 2048, NULL, 1, NULL);
        }
    }

    void onDisconnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) {
        String peerMac = NimBLEAddress(desc->peer_ota_addr).toString().c_str();
        peerMac.toLowerCase();
        peerMac.trim();
        Serial.printf("[BLE Server] PC Disconnected! MAC: %s (conn_handle: %d)\n",
                      peerMac.c_str(), desc->conn_handle);
        
        for (int i = 0; i < MAX_KVM_CLIENTS; i++) {
            if (kvmClients[i].conn_id == desc->conn_handle || (kvmClients[i].mac.length() > 0 && kvmClients[i].mac.equalsIgnoreCase(peerMac))) {
                kvmClients[i].active = false;
                break;
            }
        }

        xTaskCreate([](void* param) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            if (NimBLEDevice::getAdvertising() && !NimBLEDevice::getAdvertising()->isAdvertising()) {
                Serial.println("[BLE Server] Resuming advertising after PC disconnect...");
                NimBLEDevice::getAdvertising()->start();
            }
            vTaskDelete(NULL);
        }, "bgAdvTask", 2048, NULL, 1, NULL);
    }

    void onAuthenticationComplete(ble_gap_conn_desc* desc) {
        String peerMac = NimBLEAddress(desc->peer_ota_addr).toString().c_str();
        peerMac.toLowerCase();
        peerMac.trim();
        Serial.printf("[BLE Server] Auth Complete for %s | Encrypted: %d | Bonded: %d | KeySize: %d\n",
                      peerMac.c_str(), desc->sec_state.encrypted, desc->sec_state.bonded, desc->sec_state.key_size);
        if (!desc->sec_state.encrypted) {
            Serial.printf("[BLE Server] Pairing/Encryption failed for %s! Deleting stale bond keys...\n", peerMac.c_str());
            NimBLEDevice::deleteBond(desc->peer_ota_addr);
        }
    }

    uint32_t onPassKeyRequest() {
        Serial.println("[BLE Server] PassKey requested by client");
        return 0;
    }

    bool onConfirmPIN(uint32_t pin) {
        Serial.printf("[BLE Server] PIN confirmation requested: %06d\n", pin);
        return true;
    }
};

// --- BLE Host (Central) Variables ---
static NimBLEAdvertisedDevice* advDevice;


static bool doConnect = false;

static NimBLEUUID hidServiceUUID("1812");
static NimBLEUUID reportCharUUID("2a4d");

void updateVirtualCursorAndSend(uint8_t buttons, int16_t dx, int16_t dy, int8_t scroll, int8_t hScroll) {
    if (monitorCount == 0) return;

    virtualX += dx;
    virtualY += dy;

    MonitorConfig& currentMon = monitors[currentMonitorIndex];

    // Find which monitor we are currently in
    int newMonitorIndex = -1;
    for (int i = 0; i < monitorCount; i++) {
        if (virtualX >= monitors[i].x && virtualX < monitors[i].x + monitors[i].width &&
            virtualY >= monitors[i].y && virtualY < monitors[i].y + monitors[i].height) {
            newMonitorIndex = i;
            break;
        }
    }

    static unsigned long lastKvmSwitchTime = 0;

    // Edge Push Detection: When cursor pushes past the edge of current monitor into PC boundary
    if (newMonitorIndex == -1) {
        bool canSwitchPC = (millis() - lastKvmSwitchTime > 300);

        // Pushing UP past top edge of current monitor
        if (dy < 0 && virtualY < currentMon.y) {
            int bestIdx = -1;
            long minScore = 99999999;
            for (int i = 0; i < monitorCount; i++) {
                if (!monitors[i].mac.equalsIgnoreCase(currentMon.mac)) {
                    if (!canSwitchPC) continue;
                    bool xOverlap = (virtualX >= monitors[i].x && virtualX < monitors[i].x + monitors[i].width);
                    long dist = abs((monitors[i].y + monitors[i].height) - currentMon.y);
                    long score = dist + (xOverlap ? 0 : 100000);
                    if (score < minScore) {
                        minScore = score;
                        bestIdx = i;
                    }
                }
            }
            if (bestIdx != -1) {
                newMonitorIndex = bestIdx;
                virtualY = monitors[bestIdx].y + monitors[bestIdx].height - 200;
                virtualX = constrain(virtualX, (long)monitors[bestIdx].x + 50, (long)(monitors[bestIdx].x + monitors[bestIdx].width - 50));
            }
        }
        // Pushing DOWN past bottom edge of current monitor
        else if (dy > 0 && virtualY >= currentMon.y + currentMon.height) {
            int bestIdx = -1;
            long minScore = 99999999;
            for (int i = 0; i < monitorCount; i++) {
                if (!monitors[i].mac.equalsIgnoreCase(currentMon.mac)) {
                    if (!canSwitchPC) continue;
                    bool xOverlap = (virtualX >= monitors[i].x && virtualX < monitors[i].x + monitors[i].width);
                    long dist = abs(monitors[i].y - (currentMon.y + currentMon.height));
                    long score = dist + (xOverlap ? 0 : 100000);
                    if (score < minScore) {
                        minScore = score;
                        bestIdx = i;
                    }
                }
            }
            if (bestIdx != -1) {
                newMonitorIndex = bestIdx;
                virtualY = monitors[bestIdx].y + 200;
                virtualX = constrain(virtualX, (long)monitors[bestIdx].x + 50, (long)(monitors[bestIdx].x + monitors[bestIdx].width - 50));
            }
        }
        // Pushing LEFT past left edge of current monitor
        else if (dx < 0 && virtualX < currentMon.x) {
            int bestIdx = -1;
            long minScore = 99999999;
            for (int i = 0; i < monitorCount; i++) {
                if (!monitors[i].mac.equalsIgnoreCase(currentMon.mac)) {
                    if (!canSwitchPC) continue;
                    bool yOverlap = (virtualY >= monitors[i].y && virtualY < monitors[i].y + monitors[i].height);
                    long dist = abs((monitors[i].x + monitors[i].width) - currentMon.x);
                    long score = dist + (yOverlap ? 0 : 100000);
                    if (score < minScore) {
                        minScore = score;
                        bestIdx = i;
                    }
                }
            }
            if (bestIdx != -1) {
                newMonitorIndex = bestIdx;
                virtualX = monitors[bestIdx].x + monitors[bestIdx].width - 200;
                virtualY = constrain(virtualY, (long)monitors[bestIdx].y + 50, (long)(monitors[bestIdx].y + monitors[bestIdx].height - 50));
            }
        }
        // Pushing RIGHT past right edge of current monitor
        else if (dx > 0 && virtualX >= currentMon.x + currentMon.width) {
            int bestIdx = -1;
            long minScore = 99999999;
            for (int i = 0; i < monitorCount; i++) {
                if (!monitors[i].mac.equalsIgnoreCase(currentMon.mac)) {
                    if (!canSwitchPC) continue;
                    bool yOverlap = (virtualY >= monitors[i].y && virtualY < monitors[i].y + monitors[i].height);
                    long dist = abs(monitors[i].x - (currentMon.x + currentMon.width));
                    long score = dist + (yOverlap ? 0 : 100000);
                    if (score < minScore) {
                        minScore = score;
                        bestIdx = i;
                    }
                }
            }
            if (bestIdx != -1) {
                newMonitorIndex = bestIdx;
                virtualX = monitors[bestIdx].x + 200;
                virtualY = constrain(virtualY, (long)monitors[bestIdx].y + 50, (long)(monitors[bestIdx].y + monitors[bestIdx].height - 50));
            }
        }

        if (newMonitorIndex == -1) {
            newMonitorIndex = currentMonitorIndex;
            virtualX = constrain(virtualX, (long)currentMon.x, (long)(currentMon.x + currentMon.width - 1));
            virtualY = constrain(virtualY, (long)currentMon.y, (long)(currentMon.y + currentMon.height - 1));
        }
    }
    
    String targetMac = monitors[newMonitorIndex].mac;
    targetMac.toLowerCase();
    targetMac.trim();

    uint16_t targetConnHandle = 0;
    for (int i = 0; i < MAX_KVM_CLIENTS; i++) {
        if (kvmClients[i].active && kvmClients[i].mac.equalsIgnoreCase(targetMac)) {
            targetConnHandle = kvmClients[i].conn_id;
            break;
        }
    }

    if (newMonitorIndex != currentMonitorIndex) {
        if (!monitors[newMonitorIndex].mac.equalsIgnoreCase(monitors[currentMonitorIndex].mac)) {
            lastKvmSwitchTime = millis();
        }
        Serial.printf("[KVM SWITCH] Cursor at (%ld, %ld) crossed to Monitor #%d (ID: %s | Bounds X:%d..%d Y:%d..%d) | Target PC: %s | conn_handle: %d\n",
                      virtualX, virtualY, newMonitorIndex + 1, monitors[newMonitorIndex].id.c_str(),
                      monitors[newMonitorIndex].x, monitors[newMonitorIndex].x + monitors[newMonitorIndex].width,
                      monitors[newMonitorIndex].y, monitors[newMonitorIndex].y + monitors[newMonitorIndex].height,
                      targetMac.c_str(), targetConnHandle);
        currentMonitorIndex = newMonitorIndex;
    }

    // Send Standard HID Report (5 bytes: Buttons, dX, dY, VScroll, HScroll)
    uint8_t report[5] = { 
        buttons, 
        (uint8_t)constrain(dx, -127, 127), 
        (uint8_t)constrain(dy, -127, 127), 
        (uint8_t)constrain(scroll, -127, 127),
        (uint8_t)constrain(hScroll, -127, 127) 
    };

    if (targetConnHandle != 0) {
        os_mbuf *om = ble_hs_mbuf_from_flat(report, sizeof(report));
        if (om != NULL) {
            ble_gattc_notify_custom(targetConnHandle, inputChar->getHandle(), om);
        }
    } else {
        inputChar->setValue(report, sizeof(report));
        inputChar->notify();
    }
}

// Callback when HID data is received from the mouse
void notifyCallback(NimBLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    if (length >= 6) {
        // Logitech MX Master 3S HID report button mask can be in byte 0 or byte 1
        uint8_t buttons = pData[0] | pData[1];
        
        // 12-bit X extraction
        int16_t x = pData[2] | ((pData[3] & 0x0F) << 8);
        if (x & 0x800) x |= 0xF000; // Sign extend to 16-bit
        
        // 12-bit Y extraction
        int16_t y = (pData[3] >> 4) | (pData[4] << 4);
        if (y & 0x800) y |= 0xF000; // Sign extend to 16-bit
        
        int8_t scroll = (int8_t)pData[5];
        int8_t hScroll = (length > 6) ? (int8_t)pData[6] : 0;

        Serial.printf("[DECODE] Raw: %02X %02X %02X %02X %02X %02X %02X -> Btn: 0x%02X, dX: %d, dY: %d, VScroll: %d, HScroll: %d\n",
                      pData[0], pData[1], pData[2], pData[3], pData[4], pData[5], (length > 6 ? pData[6] : 0),
                      buttons, x, y, scroll, hScroll);

        updateVirtualCursorAndSend(buttons, x, y, scroll, hScroll);
    }
}

bool connectToServer();

static TaskHandle_t reconnTaskHandle = NULL;

// Callback for BLE Connection Status
class ClientCallbacks : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient* pClient) {
        Serial.println("[BLE Host] Connected to mouse!");
        connected = true;
    }
    void onDisconnect(NimBLEClient* pClient) {
        Serial.println("[BLE Host] Disconnected from mouse!");
        connected = false;
        if (targetMouseMac.length() > 0 && !isScanningForMice) {
            if (reconnTaskHandle != NULL) {
                vTaskDelete(reconnTaskHandle);
                reconnTaskHandle = NULL;
            }
            xTaskCreate([](void* param) {
                vTaskDelay(pdMS_TO_TICKS(2000));
                int retries = 0;
                while (!connected && targetMouseMac.length() > 0 && !isScanningForMice && retries < 60) {
                    Serial.printf("[BLE Host] Auto-reconnecting to bound mouse (%s) [attempt %d]...\n", targetMouseMac.c_str(), retries + 1);
                    if (connectToServer()) {
                        Serial.println("[BLE Host] Reconnected to mouse successfully!");
                        break;
                    }
                    retries++;
                    vTaskDelay(pdMS_TO_TICKS(3000));
                }
                reconnTaskHandle = NULL;
                vTaskDelete(NULL);
            }, "mouseReconnectTask", 4096, NULL, 1, &reconnTaskHandle);
        }
    }
};

bool connectToServer() {
    if (isScanningForMice) return false;
    if (targetMouseMac.length() == 0 && !advDevice) return false;

    if (!pClient) {
        pClient = NimBLEDevice::createClient();
        pClient->setClientCallbacks(new ClientCallbacks());
    }

    if (pClient->isConnected()) {
        connected = true;
        return true;
    }

    bool connRes = false;
    if (advDevice) {
        Serial.printf("[BLE Host] Connecting to advertised device: %s...\n", advDevice->getAddress().toString().c_str());
        connRes = pClient->connect(advDevice);
    } else {
        Serial.printf("[BLE Host] Connecting directly to target MAC (Public): %s...\n", targetMouseMac.c_str());
        NimBLEAddress addrPublic(targetMouseMac.c_str(), BLE_ADDR_PUBLIC);
        connRes = pClient->connect(addrPublic);
        if (!connRes) {
            Serial.printf("[BLE Host] Public connection failed. Trying target MAC (Random): %s...\n", targetMouseMac.c_str());
            NimBLEAddress addrRandom(targetMouseMac.c_str(), BLE_ADDR_RANDOM);
            connRes = pClient->connect(addrRandom);
        }
    }

    if (!connRes) {
        Serial.println("[BLE Host] Connection attempt failed (mouse not advertising or out of range).");
        return false;
    }

    Serial.println("[BLE Host] Connected! Securing connection (Pairing)...");
    if (!pClient->secureConnection()) {
        Serial.println("[BLE Host] Failed to secure connection. Mouse might reject it.");
    } else {
        Serial.println("[BLE Host] Connection secured!");
    }

    NimBLERemoteService* pService = pClient->getService(hidServiceUUID);
    if (pService != nullptr) {
        std::vector<NimBLERemoteCharacteristic*>* pChars = pService->getCharacteristics(true);
        for (auto &pChar : *pChars) {
            if (pChar->getUUID() == reportCharUUID) {
                if(pChar->canNotify()) {
                    pChar->subscribe(true, notifyCallback);
                    Serial.println("[BLE Host] Subscribed to HID report!");
                }
            }
        }
    } else {
        pClient->disconnect();
        return false;
    }
    connected = true;
    return true;
}

#define CONFIG_SERVICE_UUID "12345678-1234-1234-1234-1234567890ab"
#define CONFIG_RX_UUID      "12345678-1234-1234-1234-1234567890ac"
#define CONFIG_TX_UUID      "12345678-1234-1234-1234-1234567890ad"

NimBLECharacteristic* configTxChar = nullptr;
NimBLECharacteristic* configRxChar = nullptr;

static JsonDocument scannedMiceDoc;

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

    Serial.printf("[BLE TX] Sending %d bytes in %d-byte MTU chunks...\n", (int)len, (int)chunkSize);

    for (size_t i = 0; i < len; i += chunkSize) {
      String chunk = fullResp.substring(i, min(i + chunkSize, len));
      configTxChar->setValue((const uint8_t*)chunk.c_str(), chunk.length());
      configTxChar->notify();
      delay(30);
    }
  }
}

class ScanCallbacks : public NimBLEAdvertisedDeviceCallbacks {
    void onResult(NimBLEAdvertisedDevice* advertisedDevice) {
        String devMac = advertisedDevice->getAddress().toString().c_str();
        devMac.toLowerCase();
        devMac.trim();

        String devName = advertisedDevice->getName().c_str();
        int rssi = advertisedDevice->getRSSI();

        if (isScanningForMice) {
            JsonArray arr = scannedMiceDoc.as<JsonArray>();
            bool exists = false;
            for (JsonObject m : arr) {
                if (m["mac"].as<String>() == devMac) {
                    m["rssi"] = rssi;
                    exists = true;
                    break;
                }
            }
            if (!exists) {
                JsonObject obj = arr.add<JsonObject>();
                obj["mac"] = devMac;
                obj["name"] = devName.length() > 0 ? devName : "Bluetooth Device";
                obj["rssi"] = rssi;
            }
            return;
        }

        if (targetMouseMac.length() > 0 && devMac == targetMouseMac) {
            Serial.printf("[BLE Scan] TARGET LOCK MATCH! Connecting to %s (%s)\n", devName.c_str(), devMac.c_str());
            NimBLEDevice::getScan()->stop();
            advDevice = advertisedDevice;
            doConnect = true;
        }
    }
};

void loadConfiguration() {
  preferences.begin(NVS_NAMESPACE, true);
  String json = preferences.getString(NVS_KEY_LAYOUT, "{}");
  targetMouseMac = preferences.getString(NVS_KEY_MOUSE_MAC, "");
  preferences.end();

  targetMouseMac.toLowerCase();
  targetMouseMac.trim();

  if (json.length() > 2 && json != "[]") {
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, json);
    if (!err) {
      // Restore mouseMac if present in saved JSON
      if (doc["mouseMac"].is<String>() && doc["mouseMac"].as<String>().length() > 0) {
        targetMouseMac = doc["mouseMac"].as<String>();
        targetMouseMac.toLowerCase();
        targetMouseMac.trim();
      }

      JsonArray arr;
      if (doc["layouts"].is<JsonArray>() && doc["layouts"].size() > 0) {
        String activeId = doc["activeLayoutId"].as<String>();
        JsonObject activeLayout = doc["layouts"][0].as<JsonObject>();
        for (JsonObject l : doc["layouts"].as<JsonArray>()) {
          if (l["id"].as<String>() == activeId) {
            activeLayout = l;
            break;
          }
        }
        arr = activeLayout["screens"].as<JsonArray>();
      } else if (doc.is<JsonArray>()) {
        arr = doc.as<JsonArray>();
      }

      monitorCount = 0;
      if (arr) {
        for (JsonObject repo : arr) {
          if (monitorCount >= MAX_MONITORS) break;
          monitors[monitorCount].id = repo["id"].as<String>();
          monitors[monitorCount].x = repo["x"];
          monitors[monitorCount].y = repo["y"];
          monitors[monitorCount].width = repo["width"];
          monitors[monitorCount].height = repo["height"];
          monitors[monitorCount].mac = repo["mac"].as<String>();
          monitorCount++;
        }
      }
      Serial.printf("Loaded %d monitors from NVS.\n", monitorCount);

      if (doc["clients"].is<JsonArray>()) {
        int clientCount = 0;
        for (JsonObject c : doc["clients"].as<JsonArray>()) {
          if (clientCount >= MAX_KVM_CLIENTS) break;
          String mac = c["mac"].as<String>();
          if (mac.length() > 0) {
            kvmClients[clientCount].mac = mac;
            kvmClients[clientCount].name = c["name"].as<String>();
            kvmClients[clientCount].active = false; // Live BLE connection state starts as false at boot
            clientCount++;
          }
        }
      }
    }
  }
}

void saveConfiguration(const String& jsonString) {
  preferences.begin(NVS_NAMESPACE, false);
  preferences.putString(NVS_KEY_LAYOUT, jsonString);

  // Extract and persist mouseMac from save payload if present
  JsonDocument doc;
  if (!deserializeJson(doc, jsonString)) {
    if (doc["mouseMac"].is<String>()) {
      String mac = doc["mouseMac"].as<String>();
      mac.toLowerCase();
      mac.trim();
      preferences.putString(NVS_KEY_MOUSE_MAC, mac);
      targetMouseMac = mac;
      Serial.printf("[NVS] Updated targetMouseMac from save payload: %s\n", targetMouseMac.c_str());
    }
  }
  preferences.end();
  Serial.println("Configuration saved to NVS!");
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

void processCommand(String input) {
  input.trim();
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
      Serial.printf("[SAVE CONFIG ERROR] Content-Length mismatch: received %d, expected %d\n", (int)jsonStr.length(), expectedLen);
      sendConfigResponse("ERROR_SAVE Content-Length mismatch");
      return;
    }

    if (jsonStr.startsWith("{") || jsonStr.startsWith("[")) {
      pendingSaveJson = jsonStr;
      doSaveConfig = true;
    }
  } else if (input == "GET_CONFIG") {
    preferences.begin(NVS_NAMESPACE, true);
    String json = preferences.getString(NVS_KEY_LAYOUT, "{}");
    preferences.end();

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, json);

    if (err || !doc.is<JsonObject>()) {
      doc.clear();
      doc["device"] = "ESP32-KVM-Switch";
      doc["activeLayoutId"] = "layout_1";
      doc["totalLayouts"] = 1;
      JsonArray layoutsArr = doc["layouts"].to<JsonArray>();
      JsonObject layout1 = layoutsArr.add<JsonObject>();
      layout1["id"] = "layout_1";
      layout1["name"] = "Default Layout";
      layout1["totalScreens"] = 0;
      layout1["screens"].to<JsonArray>();
      doc["clients"].to<JsonArray>();
    }

    // Include bound mouse MAC in the unified JSON payload for backup & restore
    doc["mouseMac"] = targetMouseMac;

    // Reset connected status for stored clients prior to active connection sync
    if (doc["clients"].is<JsonArray>()) {
      for (JsonObject c : doc["clients"].as<JsonArray>()) {
        c["connected"] = false;
      }
    }

    // Dynamic merge of active connected BLE clients into unified JSON
    JsonArray clientsArr = doc["clients"].is<JsonArray>() ? doc["clients"].as<JsonArray>() : doc["clients"].to<JsonArray>();
    for (int i = 0; i < MAX_KVM_CLIENTS; i++) {
      if (kvmClients[i].active && kvmClients[i].mac.length() > 0) {
        String activeMac = kvmClients[i].mac;
        activeMac.toLowerCase();
        activeMac.trim();

        bool exists = false;
        for (JsonObject c : clientsArr) {
          String cMac = c["mac"].as<String>();
          cMac.toLowerCase();
          cMac.trim();
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
  } else if (input == "SCAN_MICE") {
    scannedMiceDoc.clear();
    scannedMiceDoc.to<JsonArray>();

    isScanningForMice = true;

    // Cancel any background reconnect task and disconnect mouse to free radio
    if (reconnTaskHandle != NULL) {
      Serial.println("[BLE Scan] Cancelling background mouseReconnectTask for discovery scan...");
      vTaskDelete(reconnTaskHandle);
      reconnTaskHandle = NULL;
    }
    if (pClient && pClient->isConnected()) {
      Serial.println("[BLE Scan] Disconnecting from mouse before discovery scan...");
      pClient->disconnect();
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

      Serial.println("[BLE Scan] Starting 5-second active discovery scan for mice...");
      pScan->start(5, false);
      pScan->clearResults();
    }
    isScanningForMice = false;
    Serial.printf("[BLE Scan] Discovery scan complete! Discovered %d BLE devices.\n", (int)scannedMiceDoc.as<JsonArray>().size());

    String jsonStr;
    serializeJson(scannedMiceDoc, jsonStr);
    sendConfigResponse("MICE " + jsonStr);
  } else if (input.startsWith("BIND_MOUSE ")) {
    String mac = input.substring(11);
    mac.toLowerCase();
    mac.trim();
    preferences.begin(NVS_NAMESPACE, false);
    preferences.putString(NVS_KEY_MOUSE_MAC, mac);
    preferences.end();
    targetMouseMac = mac;
    sendConfigResponse("OK_BIND_MOUSE " + targetMouseMac);

    if (reconnTaskHandle != NULL) {
      vTaskDelete(reconnTaskHandle);
      reconnTaskHandle = NULL;
    }
    if (pClient && pClient->isConnected()) {
      pClient->disconnect();
    }
    if (targetMouseMac.length() > 0) {
      doConnect = true;
    }
  } else if (input == "UNBIND_MOUSE") {
    preferences.begin(NVS_NAMESPACE, false);
    preferences.remove(NVS_KEY_MOUSE_MAC);
    preferences.end();
    targetMouseMac = "";
    if (reconnTaskHandle != NULL) {
      vTaskDelete(reconnTaskHandle);
      reconnTaskHandle = NULL;
    }
    if (pClient && pClient->isConnected()) {
      pClient->disconnect();
    }
    sendConfigResponse("OK_UNBIND_MOUSE");
  } else if (input == "GET_TARGET_MOUSE") {
    sendConfigResponse("TARGET_MOUSE " + targetMouseMac);
  } else if (input == "DUMP_FLASH") {
    preferences.begin(NVS_NAMESPACE, true);
    String json = preferences.getString(NVS_KEY_LAYOUT, "{}");
    String mouseMac = preferences.getString(NVS_KEY_MOUSE_MAC, "");
    preferences.end();
    Serial.println("\n--- [NVS FLASH DUMP] ---");
    Serial.printf("Flash layout string length: %d bytes\n", json.length());
    Serial.printf("Flash target mouse MAC: %s\n", mouseMac.c_str());
    Serial.println(json);
    Serial.println("--- [END NVS FLASH DUMP] ---\n");
  }
}

static String bleRxBuffer = "";
static std::vector<String> bleCmdQueue;

class ConfigRxCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pCharacteristic) {
        std::string rxValue = pCharacteristic->getValue();
        Serial.printf("[BLE RX DEBUG]: Received %d bytes: '%s'\n", (int)rxValue.length(), rxValue.c_str());
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
  Serial.setRxBufferSize(2048);
  Serial.begin(115200);
#if CONFIG_IDF_TARGET_ESP32S3
  USBSerial.begin(115200);
#endif
  delay(2000);
  
  Serial.println("\n--- ESP32 KVM Switcher Started ---");
  loadConfiguration();
  
  Serial.println("[BLE] Initializing NimBLE...");
  NimBLEDevice::init("ESP32 KVM Mouse");
  NimBLEDevice::setMTU(512);
  NimBLEDevice::setSecurityAuth(true, false, false); // Compatible Just Works pairing (bonding=true, mitm=false, sc=false for legacy compatibility)
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
  
  // Setup BLE Server (Peripheral)
  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());
  hidDevice = new NimBLEHIDDevice(pServer);
  inputChar = hidDevice->inputReport(1); // Report ID 1
  
  hidDevice->manufacturer()->setValue("Antigravity Labs");
  hidDevice->pnp(0x02, 0x046d, 0x0000, 0x0110);
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
  pAdvertising->setAppearance(0x03C2); // HID Mouse Appearance
  pAdvertising->addServiceUUID(hidDevice->hidService()->getUUID());
  pAdvertising->addServiceUUID(CONFIG_SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->start();
  Serial.println("[BLE Server] Advertising HID Mouse & ESP32 KVM Server Config Service...");

  // If targetMouseMac is bound, attempt direct non-blocking connection at startup
  if (targetMouseMac.length() > 0 && !connected) {
    xTaskCreate([](void* param) {
      vTaskDelay(pdMS_TO_TICKS(2000));
      if (targetMouseMac.length() > 0 && !connected) {
        Serial.printf("[BLE Host] Initiating direct MAC connection for bound mouse (%s)...\n", targetMouseMac.c_str());
        connectToServer();
      }
      vTaskDelete(NULL);
    }, "mouseConnectTask", 4096, NULL, 1, NULL);
  }
}

void loop() {
  if (doSaveConfig) {
    executePendingSave();
  }

  if (!bleCmdQueue.empty()) {
    String cmd = bleCmdQueue.front();
    bleCmdQueue.erase(bleCmdQueue.begin());
    Serial.print("[BLE RX CMD]: ");
    Serial.println(cmd);
    processCommand(cmd);
  }

  if (doConnect) {
    doConnect = false;
    xTaskCreate([](void* param) {
      connectToServer();
      vTaskDelete(NULL);
    }, "mouseConnTask", 4096, NULL, 1, NULL);
  }

  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() > 0) {
      Serial.printf("[UART RX CMD]: %s\n", input.c_str());
      processCommand(input);
    }
  }

#if CONFIG_IDF_TARGET_ESP32S3
  if (USBSerial.available()) {
    String input = USBSerial.readStringUntil('\n');
    input.trim();
    if (input.length() > 0) {
      Serial.printf("[USB CDC RX CMD]: %s\n", input.c_str());
      processCommand(input);
    }
  }
#endif
}
