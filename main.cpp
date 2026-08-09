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

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) {
        String peerMac = NimBLEAddress(desc->peer_ota_addr).toString().c_str();
        Serial.print("[BLE Server] PC Connected! MAC: ");
        Serial.println(peerMac);
        
        // Save connection
        for (int i = 0; i < MAX_KVM_CLIENTS; i++) {
            if (!kvmClients[i].active) {
                kvmClients[i].conn_id = desc->conn_handle;
                kvmClients[i].mac = peerMac;
                kvmClients[i].active = true;
                break;
            }
        }

        // Keep advertising so the second PC can connect
        Serial.println("[BLE Server] Restarting advertising for the second PC...");
        pServer->getAdvertising()->start();
    }

    void onDisconnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) {
        Serial.println("[BLE Server] PC Disconnected.");
        for (int i = 0; i < MAX_KVM_CLIENTS; i++) {
            if (kvmClients[i].conn_id == desc->conn_handle) {
                kvmClients[i].active = false;
                break;
            }
        }

        Serial.println("[BLE Server] Restarting advertising...");
        pServer->getAdvertising()->start();
    }
};

// --- BLE Host (Central) Variables ---
static NimBLEAdvertisedDevice* advDevice;
static bool doConnect = false;
static bool connected = false;

static NimBLEUUID hidServiceUUID("1812");
static NimBLEUUID reportCharUUID("2a4d");

void updateVirtualCursorAndSend(uint8_t buttons, int16_t dx, int16_t dy, int8_t scroll, int8_t hScroll) {
    if (monitorCount == 0) return;

    virtualX += dx;
    virtualY += dy;

    // Boundaries clamping (simple bounding box around all monitors)
    long minX = 0, maxX = 0, minY = 0, maxY = 0;
    for (int i = 0; i < monitorCount; i++) {
        if (monitors[i].x < minX) minX = monitors[i].x;
        if (monitors[i].x + monitors[i].width > maxX) maxX = monitors[i].x + monitors[i].width;
        if (monitors[i].y < minY) minY = monitors[i].y;
        if (monitors[i].y + monitors[i].height > maxY) maxY = monitors[i].y + monitors[i].height;
    }
    if (virtualX < minX) virtualX = minX;
    if (virtualX > maxX) virtualX = maxX;
    if (virtualY < minY) virtualY = minY;
    if (virtualY > maxY) virtualY = maxY;

    // Find which monitor we are currently in
    int newMonitorIndex = currentMonitorIndex;
    for (int i = 0; i < monitorCount; i++) {
        if (virtualX >= monitors[i].x && virtualX <= monitors[i].x + monitors[i].width &&
            virtualY >= monitors[i].y && virtualY <= monitors[i].y + monitors[i].height) {
            newMonitorIndex = i;
            break;
        }
    }
    
    currentMonitorIndex = newMonitorIndex;
    String targetMac = monitors[currentMonitorIndex].mac;
    targetMac.toLowerCase();

    // Send Standard HID Report (5 bytes: Buttons, dX, dY, VScroll, HScroll)
    uint8_t report[5] = { 
        buttons, 
        (uint8_t)constrain(dx, -127, 127), 
        (uint8_t)constrain(dy, -127, 127), 
        (uint8_t)constrain(scroll, -127, 127),
        (uint8_t)constrain(hScroll, -127, 127) 
    };

    inputChar->setValue(report, sizeof(report));
    inputChar->notify();
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
            xTaskCreate([](void* param) {
                delay(500);
                NimBLEScan* pScan = NimBLEDevice::getScan();
                if (pScan && !pScan->isScanning()) {
                    pScan->start(0, false);
                }
                vTaskDelete(NULL);
            }, "mouseReconnectTask", 4096, NULL, 1, NULL);
        }
    }
};

bool connectToServer() {
    Serial.print("[BLE Host] Forming a connection to ");
    Serial.println(advDevice->getAddress().toString().c_str());

    pClient = NimBLEDevice::createClient();
    pClient->setClientCallbacks(new ClientCallbacks());

    if (!pClient->connect(advDevice)) {
        Serial.println("[BLE Host] Failed to connect.");
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
            kvmClients[clientCount].active = c["connected"].as<bool>();
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
      sendConfigResponse("OK_SAVE");
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

    // Dynamic merge of active connected BLE clients into unified JSON
    JsonArray clientsArr;
    if (doc["clients"].is<JsonArray>()) {
      clientsArr = doc["clients"].as<JsonArray>();
    } else {
      clientsArr = doc["clients"].to<JsonArray>();
    }
    for (int i = 0; i < MAX_KVM_CLIENTS; i++) {
      if (kvmClients[i].active) {
        bool exists = false;
        for (JsonObject c : clientsArr) {
          if (c["mac"].as<String>() == kvmClients[i].mac) {
            c["connected"] = true;
            exists = true;
            break;
          }
        }
        if (!exists) {
          JsonObject clientObj = clientsArr.add<JsonObject>();
          clientObj["mac"] = kvmClients[i].mac;
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

    // Disconnect from mouse if currently connected to avoid dual-role GATT conflict
    if (pClient && pClient->isConnected()) {
      Serial.println("[BLE Scan] Disconnecting from mouse before discovery scan...");
      pClient->disconnect();
      delay(300);
    }

    NimBLEScan* pScan = NimBLEDevice::getScan();
    if (pScan && pScan->isScanning()) {
      pScan->stop();
      delay(100);
    }

    Serial.println("[BLE Scan] Starting 5-second active discovery scan for mice...");
    pScan->start(5, false);
    isScanningForMice = false;
    Serial.printf("[BLE Scan] Discovery scan complete! Discovered %d BLE devices.\n", (int)scannedMiceDoc.as<JsonArray>().size());

    String jsonStr;
    serializeJson(scannedMiceDoc, jsonStr);
    sendConfigResponse("MICE " + jsonStr);

    if (targetMouseMac.length() > 0 && !connected) {
      xTaskCreate([](void* param) {
        delay(300);
        NimBLEScan* pScan = NimBLEDevice::getScan();
        if (pScan && !pScan->isScanning()) {
          Serial.println("[BLE Scan] Resuming background scan for target mouse...");
          pScan->start(0, false);
        }
        vTaskDelete(NULL);
      }, "bgScanTask", 4096, NULL, 1, NULL);
    }
  } else if (input.startsWith("BIND_MOUSE ")) {
    String mac = input.substring(11);
    mac.toLowerCase();
    mac.trim();
    preferences.begin(NVS_NAMESPACE, false);
    preferences.putString(NVS_KEY_MOUSE_MAC, mac);
    preferences.end();
    targetMouseMac = mac;
    sendConfigResponse("OK_BIND_MOUSE " + targetMouseMac);

    if (pClient && pClient->isConnected()) {
      pClient->disconnect();
    } else {
      xTaskCreate([](void* param) {
        NimBLEScan* pScan = NimBLEDevice::getScan();
        if (pScan) {
          if (pScan->isScanning()) pScan->stop();
          pScan->start(0, false);
        }
        vTaskDelete(NULL);
      }, "bgScanTask", 4096, NULL, 1, NULL);
    }
  } else if (input == "UNBIND_MOUSE") {
    preferences.begin(NVS_NAMESPACE, false);
    preferences.remove(NVS_KEY_MOUSE_MAC);
    preferences.end();
    targetMouseMac = "";
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
  NimBLEDevice::setSecurityAuth(true, true, true);
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
  pAdvertising->start();
  Serial.println("[BLE Server] Advertising HID Mouse & ESP32 KVM Server Config Service...");

  // Setup BLE Client (Host) in background task to avoid blocking main loop()
  xTaskCreate([](void* param) {
    NimBLEScan* pScan = NimBLEDevice::getScan();
    pScan->setAdvertisedDeviceCallbacks(new ScanCallbacks());
    pScan->setActiveScan(true);
    pScan->setInterval(97);
    pScan->setWindow(37);
    pScan->start(0, false);
    vTaskDelete(NULL);
  }, "bleScanTask", 4096, NULL, 1, NULL);
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
    connectToServer();
    doConnect = false;
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
