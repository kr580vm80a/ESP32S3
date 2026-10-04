#pragma once

#include "kvm_types.h"

void initBleServer();
void checkAndLogPhyStatus(uint16_t connHandle, const char* deviceLabel);
void activateWebServiceMode();
void deactivateWebServiceMode();
bool isWebServiceModeActive();
void checkWebServiceTimeout();
bool isMacInActiveLayout(const String& mac);
