#include <Arduino.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <NimBLEDevice.h>
#include <NimBLEHIDDevice.h>

Preferences preferences;

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
static NimBLEClient* pClient = nullptr;

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
    }
    void onDisconnect(NimBLEClient* pClient) {
        Serial.println("[BLE Host] Disconnected from mouse!");
        connected = false;
        NimBLEDevice::getScan()->start(0);
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

class ScanCallbacks : public NimBLEAdvertisedDeviceCallbacks {
    void onResult(NimBLEAdvertisedDevice* advertisedDevice) {
        if (advertisedDevice->haveServiceUUID() && advertisedDevice->isAdvertisingService(hidServiceUUID) || 
            String(advertisedDevice->getName().c_str()).indexOf("MX Master") != -1 ||
            String(advertisedDevice->getName().c_str()).indexOf("Logi") != -1) {
            
            Serial.println("[BLE Scan] MATCH! Found a target HID Mouse.");
            NimBLEDevice::getScan()->stop();
            advDevice = advertisedDevice;
            doConnect = true;
        }
    }
};

#define CONFIG_SERVICE_UUID "12345678-1234-1234-1234-1234567890ab"
#define CONFIG_RX_UUID      "12345678-1234-1234-1234-1234567890ac"
#define CONFIG_TX_UUID      "12345678-1234-1234-1234-1234567890ad"

NimBLECharacteristic* configTxChar = nullptr;
NimBLECharacteristic* configRxChar = nullptr;

void loadConfiguration() {
  preferences.begin("kvm_config", true);
  String json = preferences.getString("layout", "[]");
  preferences.end();

  if (json != "[]") {
    JsonDocument doc;
    deserializeJson(doc, json);
    JsonArray arr = doc.as<JsonArray>();
    monitorCount = 0;
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
    Serial.printf("Loaded %d monitors from NVS.\n", monitorCount);
  }
}

void saveConfiguration(const String& jsonString) {
  preferences.begin("kvm_config", false);
  preferences.putString("layout", jsonString);
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
    String jsonStr = input.substring(12);
    jsonStr.trim();
    if (jsonStr.startsWith("{") || jsonStr.startsWith("[")) {
      pendingSaveJson = jsonStr;
      doSaveConfig = true;
    }
  } else if (input == "GET_CLIENTS") {
    JsonDocument doc;
    JsonArray arr = doc.to<JsonArray>();
    for (int i = 0; i < MAX_KVM_CLIENTS; i++) {
      if (kvmClients[i].active) {
        JsonObject client = arr.add<JsonObject>();
        client["mac"] = kvmClients[i].mac;
        client["connected"] = true;
      }
    }
    String response;
    serializeJson(doc, response);
    String fullResponse = "CLIENTS " + response + "\n";
    Serial.print(fullResponse);
    if (configTxChar) {
      size_t len = fullResponse.length();
      size_t chunkSize = 60;
      for (size_t i = 0; i < len; i += chunkSize) {
        String chunk = fullResponse.substring(i, min(i + chunkSize, len));
        configTxChar->setValue((const uint8_t*)chunk.c_str(), chunk.length());
        configTxChar->notify();
        delay(15);
      }
    }
  } else if (input == "GET_CONFIG") {
    preferences.begin("kvm_config", true);
    String json = preferences.getString("layout", "[]");
    preferences.end();
    String fullResponse = "CONFIG " + json + "\n";
    Serial.print(fullResponse);
    if (configTxChar) {
      size_t len = fullResponse.length();
      size_t chunkSize = 60;
      for (size_t i = 0; i < len; i += chunkSize) {
        String chunk = fullResponse.substring(i, min(i + chunkSize, len));
        configTxChar->setValue((const uint8_t*)chunk.c_str(), chunk.length());
        configTxChar->notify();
        delay(15);
      }
    }
  } else if (input == "DUMP_FLASH") {
    preferences.begin("kvm_config", true);
    String json = preferences.getString("layout", "[]");
    preferences.end();
    Serial.println("\n--- [NVS FLASH DUMP] ---");
    Serial.printf("Flash layout string length: %d bytes\n", json.length());
    Serial.println(json);
    Serial.println("--- [END NVS FLASH DUMP] ---\n");
  }
}

static String bleRxBuffer = "";
static String pendingBleCommand = "";
static bool hasPendingBleCommand = false;

class ConfigRxCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pCharacteristic) {
        std::string rxValue = pCharacteristic->getValue();
        if (rxValue.length() > 0) {
            bleRxBuffer += String(rxValue.c_str());
            while (bleRxBuffer.indexOf('\n') != -1) {
              int lineEnd = bleRxBuffer.indexOf('\n');
              String cmd = bleRxBuffer.substring(0, lineEnd);
              bleRxBuffer = bleRxBuffer.substring(lineEnd + 1);
              cmd.trim();
              if (cmd.length() > 0) {
                pendingBleCommand = cmd;
                hasPendingBleCommand = true;
              }
            }
        }
    }
};

void setup() {
  Serial.setRxBufferSize(2048);
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n--- ESP32 KVM Switcher Started ---");
  loadConfiguration();
  
  Serial.println("[BLE] Initializing NimBLE...");
  NimBLEDevice::init("ESP32 KVM Mouse");
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

  // Setup BLE Client (Host)
  NimBLEScan* pScan = NimBLEDevice::getScan();
  pScan->setAdvertisedDeviceCallbacks(new ScanCallbacks());
  pScan->setActiveScan(true);
  pScan->setInterval(97);
  pScan->setWindow(37);
  pScan->start(0, false);
}

void loop() {
  if (doSaveConfig) {
    executePendingSave();
  }

  if (hasPendingBleCommand) {
    String cmd = pendingBleCommand;
    pendingBleCommand = "";
    hasPendingBleCommand = false;
    processCommand(cmd);
  }

  if (doConnect) {
    connectToServer();
    doConnect = false;
  }

  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    processCommand(input);
  }
}
