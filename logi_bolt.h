#pragma once
#include <Arduino.h>

void logi_bolt_init();
void logi_bolt_deinit();
void logi_bolt_loop();
bool logi_bolt_is_mouse_connected();
bool logi_bolt_is_keyboard_connected();
bool logi_bolt_is_device_attached();
void logi_bolt_pause_host_tasks(bool pause);
void logi_bolt_set_keyboard_leds(uint8_t leds);
void scheduleBootCalibration();
