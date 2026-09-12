#pragma once

#include <Arduino.h>
#include <NimBLEDevice.h>
#include "kvm_types.h"

extern String firstConnectedPcMac;
extern bool isCalibrated;
extern bool mouseConnected;

void resetSubpixelAccumulators();
MonitorConfig& primaryMonitor(const String& targetMac);
void sendAbsPosWindows(uint16_t connHandle, uint16_t absX, uint16_t absY);
void sendRelative12Bit(uint16_t connHandle, int32_t dx, int32_t dy, uint8_t buttons = 0, int8_t scroll = 0, int8_t hScroll = 0);
void sendAbsoluteCoordinatesWindows(uint16_t connHandle, int monIndex, long targetGlobalX, long targetGlobalY, const char* contextLabel);
void sendAbsoluteCoordinatesMacOs(uint16_t connHandle, int monIndex, long targetGlobalX, long targetGlobalY, const char* contextLabel);
void sendAbsoluteCoordinatesMacOs1(uint16_t connHandle, int monIndex, long targetGlobalX, long targetGlobalY, const char* contextLabel);
void sendAbsoluteCoordinatesAndroid(uint16_t connHandle, int monIndex, long targetGlobalX, long targetGlobalY, const char* contextLabel);
void sendAbsoluteCoordinates(uint16_t connHandle, int monIndex, long targetGlobalX, long targetGlobalY, const char* contextLabel);
void calibrateFirstConnectedPcToCenter(String targetMac);
void scheduleBootCalibration();
bool isPcSwitchAllowed(uint8_t mouseButtons);
void updateVirtualCursorAndSend(uint8_t buttons, int16_t dx, int16_t dy, int8_t scroll = 0, int8_t hScroll = 0);
void notifyCallback(NimBLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify);
