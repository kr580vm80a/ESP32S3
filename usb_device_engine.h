#pragma once
#include <Arduino.h>

#if CONFIG_IDF_TARGET_ESP32S3

void usb_device_init();
void usb_device_loop();
void usb_device_stop();
bool usb_device_is_connected();
void usb_device_send_mouse(uint8_t buttons, int16_t dx, int16_t dy, int8_t wheel, int8_t pan);
void usb_device_send_keyboard(const uint8_t* rep8);
void usb_device_send_consumer(uint16_t usage);
void usb_device_send_mac_abs(const uint8_t* rep5);
void usb_device_send_win_abs(const uint8_t* rep4);
typedef void (*usb_led_cb_t)(uint8_t leds);
void usb_device_set_led_callback(usb_led_cb_t cb);

#else

inline void usb_device_init() {}
inline void usb_device_loop() {}
inline void usb_device_stop() {}
inline bool usb_device_is_connected() { return false; }
inline void usb_device_send_mouse(uint8_t buttons, int16_t dx, int16_t dy, int8_t wheel, int8_t pan) {}
inline void usb_device_send_keyboard(const uint8_t* rep8) {}
inline void usb_device_send_consumer(uint16_t usage) {}
inline void usb_device_send_mac_abs(const uint8_t* rep5) {}
inline void usb_device_send_win_abs(const uint8_t* rep4) {}
typedef void (*usb_led_cb_t)(uint8_t leds);
inline void usb_device_set_led_callback(usb_led_cb_t cb) {}

#endif
