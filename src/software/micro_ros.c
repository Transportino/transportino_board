#include "software/micro_ros.h"

#include "transports/transportino_uart_transports.h"

#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/joint_state.h>
#include <sensor_msgs/msg/temperature.h>


#include <rmw_microros/rmw_microros.h>
#include <micro_ros_utilities/string_utilities.h>
#include <micro_ros_utilities/type_utilities.h>

#include "transportino_interfaces/srv/board_cmd.h"
#include "transportino_interfaces/srv/board_reset.h"
#include "transportino_interfaces/msg/board_motors.h"

#include <hardware/watchdog.h>
#include <pico/stdlib.h>
#include <math.h>

#include "transportino_board.h"

#define TIMER_DELTA_TIME 1.0 / DATA_FREQUENCY

micro_ros* _microros;
// Msgs
sensor_msgs__msg__Imu imu_data;
transportino_interfaces__msg__BoardMotors motors_data;
geometry_msgs__msg__Twist cmd_vel_data;
nav_msgs__msg__Odometry odom_data;
sensor_msgs__msg__JointState wheel_joint_states_data;
sensor_msgs__msg__Temperature temperature_data;
builtin_interfaces__msg__Time stamp;
rosidl_runtime_c__String restarting_msg, prog_restarting_msg;
// Service Restart
transportino_interfaces__srv__BoardReset_Request reset_req;
transportino_interfaces__srv__BoardReset_Response reset_res;


// Msg inits
void init_publisher_msgs();
void init_service_msgs();

// Publisher callbacks
void timer_callback(rcl_timer_t * timer, int64_t previous_call);

// Service callbacks
void service_reset(const void * req, void * res, void* context);

// Subscription callbacks
void on_cmd_vel(const void * msg, void* context);


terror micro_ros_setup(micro_ros* micro_ros, void* _tboard)
{
    _microros = micro_ros;
    micro_ros->error = NULL_ERROR;
    micro_ros->tboard = _tboard;

    rmw_uros_set_custom_transport(
		true,
		NULL,
		transportino_serial_transport_open,
		transportino_serial_transport_close,
		transportino_serial_transport_write,
		transportino_serial_transport_read
	);

    micro_ros->allocator = rcl_get_default_allocator();

    // Try connecting to micro ros agent for 2 minutes (one attempt every second)
    rcl_ret_t ros_status = rmw_uros_ping_agent(1000, 120);
    if(ros_status != RCL_RET_OK) {
        return terror_make(TRANSPORTINO_MICRO_ROS, ros_status, true);
    }

    rmw_uros_sync_session(100);
    
    return NULL_ERROR;
}

terror micro_ros_init(micro_ros* micro_ros)
{


    rclc_support_init(&micro_ros->support, 0, NULL, &micro_ros->allocator);

    rclc_node_init_default(&micro_ros->main_node, "board", "transportino", 
        &micro_ros->support);

    // Services
    init_service_msgs();

    // Reset service
    rclc_service_init_best_effort(
        &micro_ros->reset_service, 
        &micro_ros->main_node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(transportino_interfaces, srv, BoardReset),
        "board/reset"
    );

    // Subscriptions
    rclc_subscription_init_best_effort(
        &micro_ros->cmd_vel_subscription,
        &micro_ros->main_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"
    );

    init_publisher_msgs();

    // Imu publisher
    rclc_publisher_init_best_effort(
        &micro_ros->imu_publisher, 
        &micro_ros->main_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "board/imu"
    );

    // Odometry publisher
    micro_ros->odom = (odometry*) malloc(sizeof(odometry));
    odom_init(micro_ros->odom);

    rclc_publisher_init_default(
        &micro_ros->odom_publisher, 
        &micro_ros->main_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "board/wheel/odom"
    );

    // Wheel state publisher
    rclc_publisher_init_best_effort(
        &micro_ros->wheel_states_publisher, 
        &micro_ros->main_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "board/wheel/joint_states"
    );

    // Temperature publisher
    rclc_publisher_init_best_effort(
        &micro_ros->temperature_publisher, 
        &micro_ros->main_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature),
        "board/temperature"
    );

    rclc_timer_init_default(
        &micro_ros->timer,
        &micro_ros->support,
        RCL_MS_TO_NS(TIMER_DELTA_TIME * 1000.0),
        timer_callback
    );
    
    rclc_executor_init(&micro_ros->executor, &micro_ros->support.context, 3, &micro_ros->allocator);

    rclc_executor_add_service_with_context(
        &micro_ros->executor,
        &micro_ros->reset_service,
        &reset_req,
        &reset_res,
        &service_reset,
        micro_ros->tboard
    );

    rclc_executor_add_subscription_with_context(
        &micro_ros->executor,
        &micro_ros->cmd_vel_subscription,
        &cmd_vel_data,
        &on_cmd_vel,
        micro_ros,
        ON_NEW_DATA
    );

    rclc_executor_add_timer(&micro_ros->executor, &micro_ros->timer);

    micro_ros->last_ping_time = time_us_32();

    return NULL_ERROR;
}

void service_reset(const void * req, void * res, void* context)
{
    transportino_interfaces__srv__BoardReset_Request * req_in = 
    (transportino_interfaces__srv__BoardReset_Request *) req;

    transportino_interfaces__srv__BoardReset_Response * res_in = 
        (transportino_interfaces__srv__BoardReset_Response *) res;
    
    tboard* board = (tboard*) context;

    if(req_in->programming_mode) {
        res_in->message = prog_restarting_msg;
    } else {
        res_in->message = restarting_msg;
    }

    transportino_restart(board, req_in->programming_mode);
    res_in->success = true;
}

void ping_callback(micro_ros* micro_ros) 
{
    watchdog_enable(MICRO_ROS_PING_TIMEOUT_MS + 100, false);
    
    led_set_pulsating(((tboard*)micro_ros->tboard)->led, true, 3.0f);

    rcl_ret_t ros_status = rmw_uros_ping_agent(MICRO_ROS_PING_TIMEOUT_MS / 5, 5);
    if(ros_status != RCL_RET_OK) {
        transportino_restart((tboard*)micro_ros->tboard, false);
    }

    // Resetting watchdog timer
    watchdog_enable(WATCHDOG_TIMEOUT_MS, false);

    led_set_pulsating(((tboard*)micro_ros->tboard)->led, true, 5.0f);
}

void on_cmd_vel(const void * msg, void* context)
{
    const geometry_msgs__msg__Twist* twist_msg = (const geometry_msgs__msg__Twist*) msg;
    micro_ros* microros = (micro_ros*) context;

    motor* left_motor = &((tboard*)microros->tboard)->motordrv->motors[0];
    motor* right_motor = &((tboard*)microros->tboard)->motordrv->motors[1];

    double v = twist_msg->linear.x;
    double w = twist_msg->angular.z;

    double left_v = ((2.0 * v) - (w * MOTORS_WHEEL_SEPARATION)) / (2.0 * MOTORS_WHEEL_RADIUS);
    double right_v = ((2.0 * v) + (w * MOTORS_WHEEL_SEPARATION)) / (2.0 * MOTORS_WHEEL_RADIUS);

    motor_move(left_motor, -left_v / RPM_TO_RADS);
    motor_move(right_motor, right_v / RPM_TO_RADS);
}

void init_publisher_msgs() 
{
    motors_data.motors.size = MOTORS_NUM;
    motors_data.motors.capacity = MOTORS_NUM;
    motors_data.motors.data = (transportino_interfaces__msg__BoardMotor *) 
        malloc(MOTORS_NUM * sizeof(transportino_interfaces__msg__BoardMotor));

    imu_data.header.frame_id = micro_ros_string_utilities_init(IMU_FRAME);

    odom_data.header.frame_id = micro_ros_string_utilities_init(ODOM_FRAME);
    odom_data.child_frame_id = micro_ros_string_utilities_init(BASE_FRAME);

    wheel_joint_states_data.header.frame_id = micro_ros_string_utilities_init(BASE_FRAME);
    

    // JointState/Name
    wheel_joint_states_data.name.capacity = MOTORS_NUM;
    wheel_joint_states_data.name.size = MOTORS_NUM;
    wheel_joint_states_data.name.data = 
        (rosidl_runtime_c__String*) malloc(MOTORS_NUM * sizeof(rosidl_runtime_c__String));
    
    wheel_joint_states_data.name.data[0] = micro_ros_string_utilities_init(LEFT_WHEEL_JOINT);
    wheel_joint_states_data.name.data[1] = micro_ros_string_utilities_init(RIGHT_WHEEL_JOINT);
    
    // JointState/Position
    wheel_joint_states_data.position.capacity = MOTORS_NUM;
    wheel_joint_states_data.position.size = MOTORS_NUM;
    wheel_joint_states_data.position.data = 
        (double*) 
            malloc(wheel_joint_states_data.position.capacity * sizeof(double));
    
    // JointState/Velocity
    wheel_joint_states_data.velocity.capacity = MOTORS_NUM;
    wheel_joint_states_data.velocity.size = MOTORS_NUM;
    wheel_joint_states_data.velocity.data = 
        (double*) 
            malloc(wheel_joint_states_data.velocity.capacity * sizeof(double));

    // JointState/Effort
    wheel_joint_states_data.effort.capacity = MOTORS_NUM;
    wheel_joint_states_data.effort.size = MOTORS_NUM;
    wheel_joint_states_data.effort.data = 
        (double*) 
            malloc(wheel_joint_states_data.effort.capacity * sizeof(double));

    for (size_t i = 0; i < MOTORS_NUM; i++)
    {
        wheel_joint_states_data.position.data[i] = 0.0;
        wheel_joint_states_data.velocity.data[i] = 0.0;
        wheel_joint_states_data.effort.data[i] = 0.0;
    }
}

void init_service_msgs()
{
    restarting_msg = micro_ros_string_utilities_init("Restarting...");
    prog_restarting_msg = micro_ros_string_utilities_init("Restarting in programming mode..");
}

double acc[3];
double gyro[3];
uint32_t time_ns;
int32_t time_s;

void timer_callback(rcl_timer_t * timer, int64_t previous_call)
{
    if(timer == NULL) {
        return;
    }

    tboard* board = (tboard*)_microros->tboard;

    // Temperature data
    time_ns = (uint32_t) rmw_uros_epoch_nanos();
    time_s = (int32_t) (rmw_uros_epoch_millis() / 1000);
    stamp.nanosec = time_ns;
    stamp.sec = time_s;

    temperature_data.header.stamp = stamp;

    temp_read(board->temp_sensor);

    temperature_data.temperature = board->temp_sensor->temperature;
    rcl_publish(&(_microros->temperature_publisher), &temperature_data, NULL);

    // Imu data

    uint8_t res = icm20689_read_gyroacc(board->icm20689, acc, gyro);

    if(res != ICM20689_SUCCESS) {
        printf("TransportinoBoard > Could't retrieve data from imu\r\n");
        transportino_error(_microros->tboard, terror_make(TRANSPORTINO_ICM20689, TRANSPORTINO_INTERNAL_ERROR, false));
        return;
    }

    imu_data.header.stamp = stamp;

    imu_data.linear_acceleration.x = acc[0];
    imu_data.linear_acceleration.y = acc[1];
    imu_data.linear_acceleration.z = acc[2];

    imu_data.angular_velocity.x = gyro[0];
    imu_data.angular_velocity.y = gyro[1];
    imu_data.angular_velocity.z = gyro[2];

    rcl_publish(&(_microros->imu_publisher), &imu_data, NULL);

 
    // Odometry

    motor* left_motor =  &board->motordrv->motors[0];
    motor* right_motor =  &board->motordrv->motors[1];

    odom_update(_microros->odom, left_motor->current_rpm, right_motor->current_rpm, TIMER_DELTA_TIME);

    odom_data.header.stamp = stamp;

    odom_data.pose.pose.position.x = _microros->odom->x;
    odom_data.pose.pose.position.y = _microros->odom->y;
    odom_data.pose.pose.position.z = 0.0;

    odom_data.pose.pose.orientation.x = _microros->odom->orientation_quat[0];
    odom_data.pose.pose.orientation.y = _microros->odom->orientation_quat[1];
    odom_data.pose.pose.orientation.z = _microros->odom->orientation_quat[2];
    odom_data.pose.pose.orientation.w = _microros->odom->orientation_quat[3];

    odom_data.twist.twist.linear.x = _microros->odom->linear_vel;
    odom_data.twist.twist.angular.z = _microros->odom->angular_vel;

    rcl_publish(&(_microros->odom_publisher), &odom_data, NULL);
    
    // Wheel joint states
    
    wheel_joint_states_data.header.stamp = stamp;

    for (size_t i = 0; i < MOTORS_NUM; i++)
    {
        wheel_joint_states_data.position.data[i] = _microros->odom->wheel_yaws[i];
        wheel_joint_states_data.velocity.data[i] = _microros->odom->wheel_vel[i];
    }

    rcl_publish(&(_microros->wheel_states_publisher), &wheel_joint_states_data, NULL);

    
}

terror micro_ros_update(micro_ros* micro_ros)
{
    if(time_us_32() - micro_ros->last_ping_time >= MICRO_ROS_PING_DELAY_US) {
        ping_callback(micro_ros);
        micro_ros->last_ping_time = time_us_32();
    }

    rclc_executor_spin_some(&micro_ros->executor, RCL_MS_TO_NS(100));

    return NULL_ERROR;
}

terror micro_ros_free(micro_ros* micro_ros)
{
    rcl_service_fini(&micro_ros->reset_service, &micro_ros->main_node);
    rcl_subscription_fini(&micro_ros->cmd_vel_subscription, &micro_ros->main_node);
    rcl_publisher_fini(&micro_ros->imu_publisher, &micro_ros->main_node);
    rcl_publisher_fini(&micro_ros->motors_publisher, &micro_ros->main_node);
    rcl_timer_fini(&micro_ros->timer);
    rclc_executor_fini(&micro_ros->executor);
    rcl_node_fini(&micro_ros->main_node);
    rclc_support_fini(&micro_ros->support);
}