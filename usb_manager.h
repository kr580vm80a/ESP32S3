#pragma once
#include <Arduino.h>

enum UsbKvmMode {
    USB_KVM_MODE_NONE = 0,     // No USB connection (Pure BLE)
    USB_KVM_MODE_HOST_BOLT,    // Peripheral connected (Logitech Bolt / Mouse) -> USB Host
    USB_KVM_MODE_DEVICE_PC     // PC / MacBook connected (USB Device HID)
};

void usb_manager_init();
void usb_manager_loop();
UsbKvmMode usb_manager_get_mode();
bool usb_manager_is_pc_connected();
bool usb_manager_is_bolt_connected();
void usb_manager_notify_host_dev_gone();
void usb_manager_set_preferred_mode(const String& mode);
String usb_manager_get_preferred_mode();
