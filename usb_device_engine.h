#pragma once
#include <Arduino.h>

#if CONFIG_IDF_TARGET_ESP32S3

void usb_device_init();
void usb_device_loop();
void usb_device_stop();
bool usb_device_is_connected();
void usb_device_send_mouse(uint8_t buttons, int16_t dx, int16_t dy, int8_t wheel, int8_t pan);
void usb_device_flush_mouse();
void usb_device_send_keyboard(const uint8_t* rep8);
void usb_device_send_consumer(uint16_t usage);
void usb_device_send_mac_abs(const uint8_t* rep5);
void usb_device_send_win_abs(const uint8_t* rep4);
int usb_device_get_detected_os();
String usb_device_get_bound_mac();
void usb_device_set_bound_mac(const String& mac);
void usb_device_clear_bound_mac();
void usb_device_check_detection();
void usb_device_on_ble_connect(const String& mac, int os);
void usb_device_on_ble_led_report(const String& mac, uint8_t leds);
typedef void (*usb_led_cb_t)(uint8_t leds);
void usb_device_set_led_callback(usb_led_cb_t cb);

#else

inline void usb_device_init() {}
inline void usb_device_loop() {}
inline void usb_device_stop() {}
inline bool usb_device_is_connected() { return false; }
inline void usb_device_send_mouse(uint8_t buttons, int16_t dx, int16_t dy, int8_t wheel, int8_t pan) {}
inline void usb_device_flush_mouse() {}
inline void usb_device_send_keyboard(const uint8_t* rep8) {}
inline void usb_device_send_consumer(uint16_t usage) {}
inline void usb_device_send_mac_abs(const uint8_t* rep5) {}
inline void usb_device_send_win_abs(const uint8_t* rep4) {}
typedef void (*usb_led_cb_t)(uint8_t leds);
inline void usb_device_set_led_callback(usb_led_cb_t cb) {}
inline int usb_device_get_detected_os() { return 0; }
inline String usb_device_get_bound_mac() { return ""; }
inline void usb_device_set_bound_mac(const String& mac) {}
inline void usb_device_clear_bound_mac() {}
inline void usb_device_check_detection() {}
inline void usb_device_on_ble_connect(const String& mac, int os) {}
inline void usb_device_on_ble_led_report(const String& mac, uint8_t leds) {}

#endif
