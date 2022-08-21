#include "software/odometry.h"
#include "transportino_config.h"
#include <math.h>

terror odom_init(odometry* odom)
{
    odom->x = 0.0;
    odom->y = 0.0;
    odom->yaw = 0.0;
    odom->linear_vel = 0.0;
    odom->angular_vel = 0.0;
    odom->orientation_quat[0] = 0.0;
    odom->orientation_quat[1] = 0.0;
    odom->orientation_quat[2] = 0.0;
    odom->orientation_quat[3] = 1.0;

    odom->wheel_yaws[0] = 0.0;
    odom->wheel_yaws[1] = 0.0;
    odom->wheel_vel[0] = 0.0;
    odom->wheel_vel[1] = 0.0;
    return NULL_ERROR;
}

terror odom_update(odometry* odom, double left_motor_rpm, double right_motor_rpm, double delta_time)
{
    odom->linear_vel = 
        (((- left_motor_rpm + right_motor_rpm) / 2.0) * MOTORS_WHEEL_RADIUS) * RPM_TO_RADS;
    odom->angular_vel =
        ((left_motor_rpm + right_motor_rpm) / MOTORS_WHEEL_SEPARATION) * MOTORS_WHEEL_RADIUS * RPM_TO_RADS;
    
    
    odom->yaw += odom->angular_vel * delta_time;    
    odom->x += (odom->linear_vel * cos(odom->yaw)) * delta_time;
    odom->y += (odom->linear_vel * sin(odom->yaw)) * delta_time;

    // Orientation quaternion from yaw angle calculation
    // See https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    odom->orientation_quat[2] = sin(odom->yaw/2);
    odom->orientation_quat[3] = cos(odom->yaw/2);

    // Wheel yaws calculation
    odom->wheel_vel[0] = -left_motor_rpm * RPM_TO_RADS;
    odom->wheel_vel[1] = -right_motor_rpm * RPM_TO_RADS;

    odom->wheel_yaws[0] += odom->wheel_vel[0] * delta_time;
    odom->wheel_yaws[1] += odom->wheel_vel[1] * delta_time;

    return NULL_ERROR;
}
