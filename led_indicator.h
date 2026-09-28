#pragma once
#include <Arduino.h>

void led_indicator_init();
void led_indicator_loop();
void led_indicator_trigger_flash(int targetOs = -1, uint32_t durationMs = 200);
void led_trigger_bright_pulse(int targetOs = -1);
