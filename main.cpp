#include <Arduino.h>
#include <ArduinoJson.h>
#include <Preferences.h>

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
  Serial.begin(115200);
  delay(2000); // Wait for serial monitor to connect
  
  Serial.println("\n--- ESP32 KVM Switcher Started ---");
  loadConfiguration();
}

void loop() {
  // Listen for configuration JSON from Web Serial API
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    // The Web UI sends: SAVE_CONFIG [{"id":...}]
    if (input.startsWith("SAVE_CONFIG ")) {
      String jsonStr = input.substring(12); // Remove "SAVE_CONFIG " prefix
      jsonStr.trim();
      
      if (jsonStr.startsWith("[") && jsonStr.endsWith("]")) {
        Serial.println("Received new configuration JSON from Web UI...");
        
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, jsonStr);
        
        if (error) {
          Serial.print("deserializeJson() failed: ");
          Serial.println(error.c_str());
          return;
        }

        // Save raw JSON to NVS
        saveConfiguration(jsonStr);
        
        // Reload into memory
        loadConfiguration();
      }
    }
  }
}
