#include "hardware/led.h"

#include <stdlib.h>

#include "pico/stdlib.h"
#include "pico/platform.h"
#include "hardware/pwm.h"

static uint8_t led_pin;

// Transportino will only have a single led so I won't need multiple leds.
// However in the future the board may become usable in other contexts
void on_pwm_wrap() 
{
    static uint8_t fade = 0;
    static bool reverting = false;
    
    pwm_clear_irq(pwm_gpio_to_slice_num(PICO_DEFAULT_LED_PIN));

    if(!reverting) {
        fade = MIN(255, fade + 1);
        if(fade == 255) {
            reverting = true;
        }
    } else {
        fade = MAX(0, fade - 1);
        if(fade == 0) {
            reverting = false;
        }
    }

    pwm_set_gpio_level(PICO_DEFAULT_LED_PIN, fade * fade);
}

terror led_init(led* led, uint8_t pin, bool pwm)
{
    led->pin = pin;
    led->value = 0;
    led->pwm = pwm;
    led->pulse = false;

    if(pwm) {
        gpio_set_function(pin, GPIO_FUNC_PWM);
        led->pwm_slice = pwm_gpio_to_slice_num(pin);
        pwm_config led_config = pwm_get_default_config();
        
        pwm_init(led->pwm_slice, &led_config, true);

    } else {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_OUT);
    }

    return NULL_ERROR;
}

terror led_pwm(led* led, bool pwm)
{
    if(led->pwm == pwm) {
        return NULL_ERROR;
    }

    led->pwm = pwm;

    if(pwm) {
        gpio_set_function(led->pin, GPIO_FUNC_PWM);
        led->pwm_slice = pwm_gpio_to_slice_num(led->pin);
        pwm_config led_config = pwm_get_default_config();
        pwm_init(led->pwm_slice, &led_config, true);
    } else {
        pwm_set_irq_enabled(led->pwm_slice, false);
        irq_set_enabled(PWM_IRQ_WRAP, false);
        
        gpio_init(led->pin);
        gpio_set_dir(led->pin, GPIO_OUT);
    }
    
    return NULL_ERROR;
}

terror led_set(led* led, uint8_t value)
{
    if(led->pwm) {
        pwm_set_gpio_level(led->pin, value*value);
    } else {
        gpio_put(led->pin, value != 0);
    }

    led->value = value;

    return NULL_ERROR;
}

terror led_set_pulsating(led* led, bool pulse, float clock_divider)
{

    if(led->pulse) {
        if(pulse) {
            pwm_set_wrap(led->pwm_slice, UINT16_MAX);
            pwm_set_clkdiv(led->pwm_slice, clock_divider);
        } else {
            led->pulse = false;
            pwm_clear_irq(led->pwm_slice);
            pwm_set_irq_enabled(led->pwm_slice, false);
            irq_set_enabled(PWM_IRQ_WRAP, false);
            pwm_set_gpio_level(led->pin, 255 * 255);
            led_pin = 0;
        }
    } else if(pulse) {
        pwm_clear_irq(led->pwm_slice);
        pwm_set_irq_enabled(led->pwm_slice, true);
        irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
        irq_set_enabled(PWM_IRQ_WRAP, true);

        pwm_set_wrap(led->pwm_slice, UINT16_MAX);
        pwm_set_clkdiv(led->pwm_slice, clock_divider);

        led->pulse = true;
        led_pin = led->pin;
    }

    return NULL_ERROR;
}
