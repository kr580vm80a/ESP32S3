#pragma once

#include <Arduino.h>
#include <NimBLEDevice.h>
#include "kvm_types.h"

uint8_t remapModifiersForTargetOs(uint8_t mods, int targetOs);
void remapKeysForTargetOs(uint8_t* rep8, int targetOs);
void sendGlobePulseToMac(uint16_t connHandle);
void sendMacCalculatorShortcut(uint16_t connHandle);
void sendMacScreenshotShortcut(uint16_t connHandle);
void sendMacLockShortcut(uint16_t connHandle);
void checkCtrlShiftGlobeTrigger(uint8_t rawMods, const uint8_t* rep8, int targetOs, uint16_t targetConn);
void keyboardNotifyCallback(NimBLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify);
void checkAndSyncCapsLock(const uint8_t* rep8);
void syncPhysicalKeyboardLedsForPc(const String& targetMac);
bool isAnyKeyboardKeyPressed();
void resetKeyboardPressedState();
void checkWindowsCtrlShiftDwell();

class KeyboardOutputCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pCharacteristic, ble_gap_conn_desc* desc) override;
};
