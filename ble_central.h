#pragma once

#include "kvm_types.h"

void startHostReconnectTask();
bool connectToMouse();
bool connectToKeyboard();
void disconnectMouse();
void disconnectKeyboard();
void triggerDeviceDiscoveryScan();
