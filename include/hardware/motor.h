#pragma once

#include <stdio.h>
#include <stdbool.h>
#include "software/motor_pid.h"

#include "transportino_error.h"

#define CLAMP(value, min, max) MAX(MIN(value, max), min)

#define MOTOR_CLOCKWISE 0
#define MOTOR_COUNTERCLOCKWISE 1

typedef struct _motorcfg {
    uint8_t pin_a;
    uint8_t pin_b;

    uint8_t pwm_pin;

    uint8_t enc_pin_a;
    uint8_t enc_pin_b;

    uint16_t min_rpm;
    uint16_t max_rpm;

    double kp, ki , kd;

} motorcfg;
typedef struct _motor {
    uint8_t pin_a;
    uint8_t pin_b;

    uint8_t pwm_pin;
    uint8_t pwm_slice;
    uint16_t pwm_value;

    uint8_t enc_pin_a;
    uint8_t enc_pin_b;

    volatile double current_rpm;
    volatile double desired_rpm;

    uint16_t min_rpm;
    uint16_t max_rpm;
    uint16_t pwm_per_rpm;

    motor_pid* pid;
    volatile double pid_rpm;

    void* encoder_timer;

    volatile int32_t pulses;
    volatile bool dir;
} motor;

terror motor_init(motor* motor, motorcfg* motor_cfg, void* tboard);
terror motor_update(motor* motor);
terror motor_set_speed(motor* motor, double rpm);
terror motor_set_dir(motor* motor, bool dir);
terror motor_move(motor* motor, double rpm);
terror motor_stop(motor* motor);
