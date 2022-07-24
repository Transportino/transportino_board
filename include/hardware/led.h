#pragma once

#include <stdbool.h>
#include <stdio.h>

#include "transportino_error.h"

typedef struct _led {
    uint8_t pin;
    uint8_t value;
    uint8_t pwm_slice;
    bool pwm;
    bool pulse;
} led;

terror led_init(led* led, uint8_t pin, bool pwm);
terror led_pwm(led* led, bool pwm);
terror led_set(led* led, uint8_t value);
terror led_set_pulsating(led* led, bool pulse, float clock_divider);