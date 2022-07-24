#include "hardware/buzzer.h"

#include <stdlib.h>

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

// Maximum "top" is set at 65534 to be able to achieve 100% duty with 65535.
#define TOP_MAX 65534

terror buzz_init(buzzer* buzz, uint8_t pin, uint16_t min_frequency, uint16_t max_frequency)
{
    buzz->pin = pin;
    buzz->frequency = 0;
    buzz->min_frequency = min_frequency;
    buzz->max_frequency = max_frequency;

    gpio_set_function(pin, GPIO_FUNC_PWM);
    buzz->pwm_slice = pwm_gpio_to_slice_num(pin);
    pwm_config buzz_config = pwm_get_default_config();
    
    pwm_init(buzz->pwm_slice, &buzz_config, true);

    return NULL_ERROR;
}

terror buzz_set_freq(buzzer* buzz, uint16_t frequency)
{
    // Code got from https://github.com/micropython/micropython/blob/master/ports/rp2/machine_pwm.c
    // Set the frequency, making "top" as large as possible for maximum resolution.

    uint32_t source_hz = clock_get_hz(clk_sys);
    
    // Constrain frequency between max and min
    uint32_t desired_freq;
    
    if(frequency == 0) {
        desired_freq = source_hz;
    } else {
        desired_freq = MAX(MIN(frequency, buzz->max_frequency), buzz->min_frequency);
    }

    uint32_t div16_top = 16 * source_hz / desired_freq;
    uint32_t top = 1;
    for (;;) {
        // Try a few small prime factors to get close to the desired frequency.
        if (div16_top >= 16 * 5 && div16_top % 5 == 0 && top * 5 <= TOP_MAX) {
            div16_top /= 5;
            top *= 5;
        } else if (div16_top >= 16 * 3 && div16_top % 3 == 0 && top * 3 <= TOP_MAX) {
            div16_top /= 3;
            top *= 3;
        } else if (div16_top >= 16 * 2 && top * 2 <= TOP_MAX) {
            div16_top /= 2;
            top *= 2;
        } else {
            break;
        }
    }
    if (div16_top < 16) {
        printf("TransportinoBoard > Buzzer frequency is too large\r\n");
        return terror_make(TRANSPORTINO_BUZZER, TRANSPORTINO_INVALID_CONFIG, false);
    } else if (div16_top >= 256 * 16) {
        printf("TransportinoBoard > Buzzer frequency is too small\r\n");
        return terror_make(TRANSPORTINO_BUZZER, TRANSPORTINO_INVALID_CONFIG, false);
    }

    pwm_hw->slice[buzz->pwm_slice].div = div16_top;
    pwm_hw->slice[buzz->pwm_slice].top = top - 1;

    if(frequency == 0) {
        buzz->frequency = 0;
    } else {
        buzz->frequency = desired_freq;
    }
    
}

terror set_volume(buzzer* buzz, bool mute)
{
    if(mute) {
        pwm_set_gpio_level(buzz->pin, 0);
    } else {
        pwm_set_gpio_level(buzz->pin, UINT16_MAX / 2);
    }

    return NULL_ERROR;
}

terror buzz_play(buzzer* buzz, uint16_t frequency)
{
    buzz_set_freq(buzz, frequency);
    set_volume(buzz, false);

    return NULL_ERROR;
}

terror buzz_mute(buzzer* buzz)
{
    buzz_set_freq(buzz, 0);
    set_volume(buzz, true);

    return NULL_ERROR;
}