#pragma once

#include <stdbool.h>
#include "transportino_error.h"

#define RPM_TO_RADS 0.10472

typedef struct _odometry {
    double x; // m
    double y; // m
    double yaw; // rad
    double orientation_quat[4];

    double linear_vel; // m/s
    double angular_vel; // rad/s

    double wheel_yaws[2]; // rad
    double wheel_vel[2]; // rad/s
} odometry;

terror odom_init(odometry* odom);
terror odom_update(odometry* odom, double left_motor_rpm, double right_motor_rpm, double delta_time);

