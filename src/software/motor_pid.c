#include "software/motor_pid.h"
#include <pico/platform.h>
#include "math.h"

#define CLAMP(value, min, max) MAX(MIN(value, max), min)


void pid_init(motor_pid* pid, double kp, double ki, double kd, double delta_time, int32_t min_value, int32_t max_value, double* current_measure, double* goal, double* output)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    
    pid->proportional = 0.0;
    pid->integral = 0.0;
    pid->derivate = 0.0;

    pid->last_error = 0.0;
    pid->error = 0.0;
    pid->delta_time = delta_time;
    pid->min_value = min_value;
    pid->max_value = max_value;
    pid->current_measure = current_measure;
    pid->output = output;
    pid->goal = goal;
}

void pid_reset(motor_pid* pid)
{
    pid->proportional = 0.0;
    pid->integral = 0.0;
    pid->derivate = 0.0;
    pid->last_error = 0.0;
    pid->error = 0.0;
    *pid->output = 0.0;
    *pid->goal = 0.0;
}

void pid_compute(motor_pid* pid)
{  
    pid->error = *pid->goal - *pid->current_measure;

    pid->proportional = pid->kp * pid->error;

    pid->integral = pid->integral + pid->ki * pid->delta_time * pid->error;
    
    pid->derivate = pid->kd * ((pid->error - pid->last_error)/pid->delta_time);

    *pid->output = CLAMP(pid->proportional + pid->integral + pid->derivate, pid->min_value, pid->max_value);

    pid->last_error = pid->error;

}