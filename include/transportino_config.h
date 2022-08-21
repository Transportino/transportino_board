#pragma once

#define I2C_PORT i2c0
#define I2C_SDA 0
#define I2C_SCL 1

#define WATCHDOG_TIMEOUT_MS 5 * 1000

#define MICRO_ROS_PING_DELAY_US 5 * 1000 * 1000
#define MICRO_ROS_PING_TIMEOUT_MS 5 * 1000

#define BUZZER_PIN 6
#define LED_PIN 7

#define MOTORS_NUM 2
#define MOTORS_WHEEL_SEPARATION 0.243 // meters
#define MOTORS_WHEEL_RADIUS 0.0335 // meters

#define MOTOR_MIN_RPM 10
#define MOTOR_MAX_RPM 178
#define MOTOR_PULSES_PER_REVOLUTIONS 11
#define MOTOR_REDUCTION_RATE 56

#define MOTOR_DRIVER_ENABLE_PIN 15

#define MOTOR_A0 18
#define MOTOR_A1 17
#define MOTOR_A_PWM 16
#define MOTOR_A_ENC0 20
#define MOTOR_A_ENC1 19

#define MOTOR_B0 23
#define MOTOR_B1 22
#define MOTOR_B_PWM 21
#define MOTOR_B_ENC0 25
#define MOTOR_B_ENC1 24

// Ros related params

#define ODOM_FRAME "odom"
#define BASE_FRAME "base_link"
#define IMU_FRAME "imu_link"

#define LEFT_WHEEL_JOINT "left_wheel_joint"
#define RIGHT_WHEEL_JOINT "right_wheel_joint"

#define DATA_FREQUENCY 30.0

