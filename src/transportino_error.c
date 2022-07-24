#include "transportino_error.h"

#include <stdbool.h>

#include "hardware/led.h"
#include "transportino_board.h"

#include <pico/stdlib.h>
#include <hardware/watchdog.h>

bool transportino_is_error_null(terror error)
{
    return error.category == 0 && error.status == 0;
}

void blink_error(led* led, terror error)
{
    for (uint8_t i = 0; i < error.category; i++)
    {
        led_set(led, 1);
        sleep_ms(300);
        led_set(led, 0);
        sleep_ms(300);
    }

    sleep_ms(1000);

    for (uint8_t i = 0; i < error.status; i++)
    {
        led_set(led, 1);
        sleep_ms(300);
        led_set(led, 0);
        sleep_ms(300);
    }

    sleep_ms(3000);
}

terror terror_make(tcategory category, tstatus status, bool crash) 
{
    terror _err = {
        .category = category,
        .status = status,
        .crash = crash
    };
    return _err;
}

void transportino_error(void* _board, terror error)
{
    tboard* board = (tboard*) _board;
    led_pwm(board->led, false);
    led_set(board->led, 0);
    blink_error(board->led, error);
}

void transportino_panic(void* _board, terror error)
{
    tboard* board = (tboard*) _board;
    led_pwm(board->led, false);
    led_set(board->led, 0);
    while (1) {
            blink_error(board->led, error);
            watchdog_update();
    }
}

