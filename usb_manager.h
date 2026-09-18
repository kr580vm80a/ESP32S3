#pragma once
#include <Arduino.h>

enum UsbKvmMode {
    USB_KVM_MODE_NONE = 0,     // No USB connection (Pure BLE)
    USB_KVM_MODE_HOST_BOLT,    // Peripheral connected (Logitech Bolt / Mouse) -> USB Host
    USB_KVM_MODE_DEVICE_PC     // PC / MacBook connected (USB Device HID)
};

void usb_manager_init();
void usb_manager_loop();
bool usb_manager_is_pc_connected();
void usb_manager_notify_host_dev_gone();
