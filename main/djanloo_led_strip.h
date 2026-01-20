#pragma once
#include "led_strip.h"
#include "driver/rmt_tx.h"

#define BLINK_GPIO 48

void configure_led(void);
void set_led_color(uint8_t red, uint8_t green, uint8_t blue);