#pragma once

#include "kvm_types.h"

void initBleServer();
void checkAndLogPhyStatus(uint16_t connHandle, const char* deviceLabel);
