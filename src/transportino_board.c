#include <stdlib.h>
#include <pico/stdlib.h>
#include <pico/bootrom.h>


#include <hardware/i2c.h>
#include <hardware/pwm.h>
#include <hardware/watchdog.h>

#include "transportino_board.h"
#include "transportino_config.h"


#include <inttypes.h>

// MicroRos
#include "transports/transportino_uart_transports.h"

#include "misc/nokia.h"

void transportino_init(tboard* tboard)
{
    tboard->restart = false;
    // Setup status led PWM
    tboard->led = (led*) malloc(sizeof(led));
    led_init(tboard->led, LED_PIN, true);

    // Setup buzzer using frequencies from notes.h
    tboard->buzz = (buzzer*) malloc(sizeof(buzzer));
    buzz_init(tboard->buzz, BUZZER_PIN, NOTE_B0, NOTE_DS8); 
    
    // Setup motor driver and check motors
    
    // This must be set based on your specific configuration
    motorcfg motors[] = {
        {   // Motor A
            .pin_a = MOTOR_A0,
            .pin_b = MOTOR_A1,
            .pwm_pin = MOTOR_A_PWM,
            .enc_pin_a = MOTOR_A_ENC0, 
            .enc_pin_b = MOTOR_A_ENC1, 
            .max_rpm = MOTOR_MAX_RPM,
            .kp = 0.2,
            .ki = 2.4,
            .kd = 0.015,
        }, 
        {   // Motor B
            .pin_a = MOTOR_B0,
            .pin_b = MOTOR_B1,
            .pwm_pin = MOTOR_B_PWM,
            .enc_pin_a = MOTOR_B_ENC0, 
            .enc_pin_b = MOTOR_B_ENC1, 
            .max_rpm = MOTOR_MAX_RPM,
            .kp = 0.2,
            .ki = 2.8,
            .kd = 0.015,
        }
    };

    led_set_pulsating(tboard->led, true, 1.5f);
    
    tboard->motordrv = (motordrv*) malloc(sizeof(motordrv));
    motordrv_init(tboard->motordrv, MOTOR_DRIVER_ENABLE_PIN, motors, tboard);

    // Initialize imu module
    led_set_pulsating(tboard->led, true, 2.0f);
    
    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 400*1000);
    
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Wait some time so the module can startup
    sleep_ms(50);

    // Initialise icm20689 module
    tboard->icm20689 = (icm20689*) malloc(sizeof(icm20689));
    
    uint8_t status = icm20689_init(tboard->icm20689);

    if(status != 0) {
        terror error = terror_make(TRANSPORTINO_ICM20689, TRANSPORTINO_INIT_ERROR, true);
        transportino_panic(tboard, error);
        return;
    }
    
    led_set_pulsating(tboard->led, false, 0.0f);
    
    // Cool buzzer startup sound :3
    nokia_play(tboard->buzz);
    
    // Led and Buzzer can't work simultaneously unfortunately because both buzzer and led pins are on the same pwm channel.
    // In the near future I will make use of different pins on a new board and 
    // hopefully the problem will be solved.
    
    led_set_pulsating(tboard->led, true, 3.0f);
    
    // Hardware initialisation completed..
    
    // Start micro_ros backend
    tboard->micro_ros = (micro_ros*) malloc(sizeof(micro_ros));
    micro_ros_setup(tboard->micro_ros, tboard);
    micro_ros_init(tboard->micro_ros);

    // Ros agent connected!

    led_set_pulsating(tboard->led, false, 0.0f);

    buzz_play(tboard->buzz, NOTE_A5);
    sleep_ms(100);
    buzz_mute(tboard->buzz);
    sleep_ms(20);
    buzz_play(tboard->buzz, NOTE_D4);
    sleep_ms(100);
    buzz_mute(tboard->buzz);
    sleep_ms(20);
    buzz_play(tboard->buzz, NOTE_A5);
    sleep_ms(100);

    buzz_mute(tboard->buzz);

    led_set_pulsating(tboard->led, true, 5.0f);
    
    // Setting up watchdog so the board will auto reboot after 5s of inactivity
    watchdog_enable(WATCHDOG_TIMEOUT_MS, false);
}

void transportino_restart(tboard* tboard, bool prog_mode) {
    tboard->restart = true;
    tboard->prog_mode = prog_mode;
}

void transportino_update(tboard* tboard)
{ 
    watchdog_update(); // Resetting watchdog timer.
    
    micro_ros_update(tboard->micro_ros);

    if(tboard->restart) {
        transportino_free(tboard);

        if(tboard->prog_mode) {
            reset_usb_boot(0, 0);
        } else {
            watchdog_reboot(0, SRAM_END, 0);
        }
    }
}

void transportino_free(tboard* tboard)
{
    micro_ros_free(tboard->micro_ros);
    free(tboard->micro_ros);
}

