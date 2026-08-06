#include <Arduino.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <NimBLEDevice.h>

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

// --- BLE Host (Central) Variables ---
static NimBLEAdvertisedDevice* advDevice;
static bool doConnect = false;
static bool connected = false;
static NimBLEClient* pClient = nullptr;

// HID Service and Characteristics UUIDs
static NimBLEUUID hidServiceUUID("1812");
static NimBLEUUID reportCharUUID("2a4d");

// Callback when HID data is received from the mouse
void notifyCallback(NimBLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    Serial.print("[BLE Mouse] Data Length: ");
    Serial.print(length);
    Serial.print(" - Data: ");
    for(int i = 0; i < length; i++) {
        Serial.printf("%02X ", pData[i]);
    }
    Serial.println();
}

// Callback for BLE Connection Status
class ClientCallbacks : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient* pClient) {
        Serial.println("[BLE] Connected to mouse!");
    }
    void onDisconnect(NimBLEClient* pClient) {
        Serial.println("[BLE] Disconnected from mouse!");
        connected = false;
        NimBLEDevice::getScan()->start(0);
    }
};

bool connectToServer() {
    Serial.print("[BLE] Forming a connection to ");
    Serial.println(advDevice->getAddress().toString().c_str());

    pClient = NimBLEDevice::createClient();
    pClient->setClientCallbacks(new ClientCallbacks());

    if (!pClient->connect(advDevice)) {
        Serial.println("[BLE] Failed to connect.");
        return false;
    }

    Serial.println("[BLE] Connected! Securing connection (Pairing)...");
    
    if (!pClient->secureConnection()) {
        Serial.println("[BLE] Failed to secure connection. Mouse might reject it.");
    } else {
        Serial.println("[BLE] Connection secured!");
    }

    Serial.println("[BLE] Discovering services...");
    
    // Obtain the HID service
    NimBLERemoteService* pService = pClient->getService(hidServiceUUID);
    if (pService != nullptr) {
        // Find all report characteristics
        std::vector<NimBLERemoteCharacteristic*>* pChars = pService->getCharacteristics(true);
        for (auto &pChar : *pChars) {
            if (pChar->getUUID() == reportCharUUID) {
                if(pChar->canNotify()) {
                    pChar->subscribe(true, notifyCallback);
                    Serial.println("[BLE] Subscribed to HID report!");
                }
            }
        }
    } else {
        Serial.println("[BLE] HID Service not found.");
        pClient->disconnect();
        return false;
    }

    connected = true;
    return true;
}

// Callback for BLE Scanning
class ScanCallbacks : public NimBLEAdvertisedDeviceCallbacks {
    void onResult(NimBLEAdvertisedDevice* advertisedDevice) {
        // Print all devices for debugging
        if(String(advertisedDevice->getName().c_str()).length() > 0) {
            Serial.print("[BLE Scan] Found: ");
            Serial.println(advertisedDevice->getName().c_str());
        }

        // Check if the device advertises HID service OR matches Logitech name
        if (advertisedDevice->haveServiceUUID() && advertisedDevice->isAdvertisingService(hidServiceUUID) || 
            String(advertisedDevice->getName().c_str()).indexOf("MX Master 3S") != -1 ||
            String(advertisedDevice->getName().c_str()).indexOf("Logi") != -1) {
            
            Serial.println("[BLE Scan] MATCH! Found a target HID Mouse.");
            NimBLEDevice::getScan()->stop();
            advDevice = advertisedDevice;
            doConnect = true;
        }
    }
};

void loadConfiguration() {
  preferences.begin("kvm_config", true); // true = readonly
  String json = preferences.getString("layout", "[]");
  preferences.end();

  if (json != "[]") {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, json);

    if (!error && doc.is<JsonArray>()) {
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
      Serial.print("Loaded ");
      Serial.print(monitorCount);
      Serial.println(" monitors from NVS.");
    }
  } else {
    Serial.println("No saved configuration found.");
  }
}

void saveConfiguration(const String& jsonString) {
  preferences.begin("kvm_config", false); // false = rw
  preferences.putString("layout", jsonString);
  preferences.end();
  Serial.println("Configuration saved to NVS!");
}

void setup() {
  // Increase RX buffer to prevent truncation of long JSON strings
  Serial.setRxBufferSize(2048);
  Serial.begin(115200);
  delay(2000); // Wait for serial monitor to connect
  
  Serial.println("\n--- ESP32 KVM Switcher Started ---");
  loadConfiguration();
  
  // Initialize NimBLE Host
  Serial.println("[BLE] Initializing NimBLE...");
  NimBLEDevice::init("ESP32_KVM_Hub");
  
  // MUST set security for HID devices (they require bonding/pairing)
  NimBLEDevice::setSecurityAuth(true, true, true);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
  
  NimBLEScan* pScan = NimBLEDevice::getScan();
  pScan->setAdvertisedDeviceCallbacks(new ScanCallbacks());
  pScan->setActiveScan(true);
  pScan->setInterval(97);
  pScan->setWindow(37);
  Serial.println("[BLE] Starting scan for Bluetooth mice...");
  pScan->start(0, false);
}

void loop() {
  // Handle BLE Connection
  if (doConnect) {
    if (connectToServer()) {
      Serial.println("[BLE] Ready to receive mouse data.");
    } else {
      Serial.println("[BLE] Failed to connect, restarting scan.");
      NimBLEDevice::getScan()->start(0);
    }
    doConnect = false;
  }

  // Listen for configuration JSON from Web Serial API
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    // The Web UI sends: SAVE_CONFIG [{"id":...}]
    if (input.startsWith("SAVE_CONFIG ")) {
      Serial.println("\n[DEBUG] Intercepted SAVE_CONFIG command!");
      String jsonStr = input.substring(12); // Remove "SAVE_CONFIG " prefix
      jsonStr.trim();
      
      Serial.print("[DEBUG] JSON Payload length: ");
      Serial.println(jsonStr.length());
      
      if (jsonStr.startsWith("[") && jsonStr.endsWith("]")) {
        Serial.println("[DEBUG] Payload format looks valid (starts with '[' and ends with ']'). Parsing...");
        
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, jsonStr);
        
        if (error) {
          Serial.print("[ERROR] deserializeJson() failed: ");
          Serial.println(error.c_str());
          return;
        }

        // Save raw JSON to NVS
        saveConfiguration(jsonStr);
        
        // Reload into memory
        loadConfiguration();
      } else {
        Serial.println("[ERROR] Invalid JSON payload format! Doesn't start with '[' or end with ']'.");
        Serial.println("[DEBUG] Received Payload: ");
        Serial.println(jsonStr);
      }
    } else if (input.length() > 0) {
      Serial.println("[DEBUG] Received unknown serial data: " + input);
    }
  }
}
