#pragma once

#include "transportino_config.h"
#include "transportino_error.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

typedef struct _microros {
    void* tboard;
    terror error;
    uint32_t last_ping_time;

    rcl_allocator_t allocator;
    rclc_executor_t executor;
    rclc_support_t support;

    rcl_node_t main_node;

    rcl_publisher_t imu_publisher;


    rcl_publisher_t motors_publisher;
    
    rcl_timer_t timer;

    // Services
    rcl_service_t cmd_service;

} micro_ros;

terror micro_ros_setup(micro_ros* micro_ros, void* tboard);
terror micro_ros_init(micro_ros* micro_ros);
terror micro_ros_update(micro_ros* micro_ros);