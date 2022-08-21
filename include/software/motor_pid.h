#pragma once

#include <stdio.h>
#include <stdbool.h>

typedef struct _pid {
    double kp, ki, kd;

    double proportional;
    double integral;
    double derivate;

    double delta_time;

    double error;
    double last_error;
    double* goal;
    
    double* current_measure;

    double* output;
    int32_t max_value, min_value;
} motor_pid;

void pid_init(motor_pid* pid, double kp, double ki, double kd, double delta_time, int32_t min_value, int32_t max_value, double* current_measure, double* goal, double* output);
void pid_reset(motor_pid* pid);
void pid_compute(motor_pid* pid);