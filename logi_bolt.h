#pragma once
#include <Arduino.h>

#if CONFIG_IDF_TARGET_ESP32S3

void logi_bolt_init();
void logi_bolt_loop();
bool logi_bolt_is_mouse_connected();
bool logi_bolt_is_keyboard_connected();
void logi_bolt_set_keyboard_leds(uint8_t leds);
void scheduleBootCalibration();

#else

inline void logi_bolt_init() {}
inline void logi_bolt_loop() {}
inline bool logi_bolt_is_mouse_connected() { return false; }
inline bool logi_bolt_is_keyboard_connected() { return false; }
inline void logi_bolt_set_keyboard_leds(uint8_t leds) {}

#endif
