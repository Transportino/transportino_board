#pragma once

#include <stdbool.h>
#include <stdio.h>
#include "transportino_error.h"

typedef struct _buzzer {
    uint8_t pin;
    uint8_t pwm_slice;

    uint16_t max_frequency;
    uint16_t min_frequency;

    uint16_t frequency;
} buzzer;

terror buzz_init(buzzer* buzz, uint8_t pin, uint16_t min_frequency, uint16_t max_frequency);
terror buzz_set_freq(buzzer* buzz, uint16_t frequency);
terror buzz_play(buzzer* buzz, uint16_t frequency);
terror buzz_mute(buzzer* buzz);