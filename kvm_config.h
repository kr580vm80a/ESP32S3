#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include "kvm_types.h"

String readNvsBlob(Preferences& pref, const char* key, const String& fallback = "[]");
void loadConfiguration();
void saveConfiguration(const String& jsonString);
String buildConfigJson();
void saveKvmClientsToPreferences();
void saveMouseToNvsLayout(String mac, String name);
void saveKeyboardToNvsLayout(String mac, String name);
void syncOrphanBonds();
void executePendingSave();
void scheduleSaveConfig(const String& json);
