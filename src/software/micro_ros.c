#include "software/micro_ros.h"

#include "transports/transportino_uart_transports.h"

#include <sensor_msgs/msg/imu.h>
#include <rmw_microros/rmw_microros.h>
#include <micro_ros_utilities/string_utilities.h>
#include <micro_ros_utilities/type_utilities.h>
#include <example_interfaces/srv/trigger.h>

#include "transportino_interfaces/srv/board_reset.h"
#include "transportino_interfaces/srv/board_cmd.h"
#include "transportino_interfaces/msg/board_motors.h"

#include <hardware/watchdog.h>
#include <pico/stdlib.h>

#include "transportino_board.h"

struct timespec ts;

extern int clock_gettime(clockid_t unused, struct timespec *tp);

rosidl_runtime_c__String restarting_msg, prog_restarting_msg, ok_msg, invalid_id_msg;

void setup_response_msg()
{
    restarting_msg = micro_ros_string_utilities_init("Restarting board...");
    prog_restarting_msg = micro_ros_string_utilities_init("Restarting board in programming mode...");
    ok_msg = micro_ros_string_utilities_init("Ok.");
    invalid_id_msg = micro_ros_string_utilities_init("Invalid motor id.");
}

micro_ros* _microros;
// Msg inits
void init_motors_msg(); 

// Publisher callbacks
void timer_callback(rcl_timer_t * timer, int64_t previous_call);

// Service callbacks
void service_cmd(const void * req, void * res, void* context);

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
    
    return NULL_ERROR;
}

terror micro_ros_init(micro_ros* micro_ros)
{

    setup_response_msg();

    rclc_support_init(&micro_ros->support, 0, NULL, &micro_ros->allocator);

    rclc_node_init_default(&micro_ros->main_node, "board", "transportino", 
        &micro_ros->support);

    
    rclc_executor_init(&micro_ros->executor, &micro_ros->support.context, 6, &micro_ros->allocator);

    // Cmd service
    
    /*rclc_service_init_best_effort(
        &micro_ros->cmd_service, 
        &micro_ros->main_node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(transportino_interfaces, srv, BoardCmd),
        "board/cmd"
    );

    transportino_interfaces__srv__BoardCmd_Request cmd_req;
    transportino_interfaces__srv__BoardCmd_Response cmd_res;

    rclc_executor_add_service_with_context(
        &micro_ros->executor,
        &micro_ros->cmd_service,
        &cmd_req,
        &cmd_res,
        service_cmd,
        micro_ros
    );*/

    // Imu publisher
    
    rclc_publisher_init_best_effort(
        &micro_ros->imu_publisher, 
        &micro_ros->main_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "board/imu"
    );

    init_motors_msg();
    rclc_publisher_init_best_effort(
        &micro_ros->motors_publisher, 
        &micro_ros->main_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(transportino_interfaces, msg, BoardMotors),
        "board/motors"
    );

    rclc_timer_init_default(
        &micro_ros->timer,
        &micro_ros->support,
        RCL_MS_TO_NS(10),
        timer_callback
    );
    
    rclc_executor_add_timer(&micro_ros->executor, &micro_ros->timer);

    micro_ros->last_ping_time = time_us_32();

    return NULL_ERROR;
}

void service_cmd(const void * req, void * res, void* context)
{
    transportino_interfaces__srv__BoardCmd_Request * req_in = 
        (transportino_interfaces__srv__BoardCmd_Request *) req;
    
    transportino_interfaces__srv__BoardCmd_Response * res_in = 
        (transportino_interfaces__srv__BoardCmd_Response *) res;
    

    micro_ros* microros = (micro_ros*) context;

    if(req_in->id == 255) {
        transportino_restart(microros->tboard, req_in->dir);
        res_in->success = true;
        if(req_in->dir) {
            res_in->message = restarting_msg;
        } else {
            res_in->message = prog_restarting_msg;
        }
        return;
    }

    if(req_in->id >= MOTORS_NUM) {
        res_in->success = false;
        res_in->message = invalid_id_msg;
        return;
    }

    /*motor* _motor = &((tboard*)microros->tboard)->motordrv->motors[req_in->id];

    if(req_in->speed == 0) {
        motor_stop(_motor);
    } else {
        //motor_move(_motor, req_in->dir, req_in->speed);
    }*/

    res_in->success = true;
    res_in->message = ok_msg;
}

void ping_callback(micro_ros* micro_ros) 
{
    watchdog_enable(MICRO_ROS_PING_TIMEOUT_MS + 100, false);
    
    rcl_ret_t ros_status = rmw_uros_ping_agent(MICRO_ROS_PING_TIMEOUT_MS / 5, 5);
    if(ros_status != RCL_RET_OK) {
        transportino_restart((tboard*)micro_ros->tboard, false);
    }

    // Resetting watchdog timer
    watchdog_enable(WATCHDOG_TIMEOUT_MS, false);
}

sensor_msgs__msg__Imu imu_data;
double acc[3];
double gyro[3];

transportino_interfaces__msg__BoardMotors motors_data;

void init_motors_msg() 
{
    motors_data.motors.size = MOTORS_NUM;
    motors_data.motors.capacity = MOTORS_NUM;
    motors_data.motors.data = (transportino_interfaces__msg__BoardMotor *) 
        malloc(MOTORS_NUM * sizeof(transportino_interfaces__msg__BoardMotor));
}


void timer_callback(rcl_timer_t * timer, int64_t previous_call)
{
    if(timer == NULL) {
        return;
    }

    clock_gettime(1, &ts);
    imu_data.header.stamp.nanosec = ts.tv_nsec;
    imu_data.header.stamp.sec = ts.tv_sec;
    
    uint8_t res = icm20689_read_gyroacc(((tboard*)_microros->tboard)->icm20689, acc, gyro);
    
    if(res != ICM20689_SUCCESS) {
        printf("TransportinoBoard > Could't retrieve data from imu\r\n");
        transportino_error(_microros->tboard, terror_make(TRANSPORTINO_ICM20689, TRANSPORTINO_INTERNAL_ERROR, false));
        return;
    }

    imu_data.linear_acceleration.x = acc[0];
    imu_data.linear_acceleration.y = acc[1];
    imu_data.linear_acceleration.z = acc[2] / 9.8f;

    imu_data.angular_velocity.x = gyro[0];
    imu_data.angular_velocity.y = gyro[1];
    imu_data.angular_velocity.z = gyro[2];

    rcl_ret_t result = rcl_publish(&(_microros->imu_publisher), &imu_data, NULL);

    motors_data.header.stamp.nanosec = ts.tv_nsec;
    motors_data.header.stamp.sec = ts.tv_sec;
    for (uint8_t i = 0; i < MOTORS_NUM; i++)
    {
        motor* _motor =  &((tboard*)_microros->tboard)->motordrv->motors[i];
        motors_data.motors.data[i].id = i;
        motors_data.motors.data[i].current_rpm = _motor->current_rpm;
        motors_data.motors.data[i].desired_rpm = _motor->desired_rpm;
        motors_data.motors.data[i].dir = _motor->dir;
    }
    
    result = rcl_publish(&(_microros->motors_publisher), &motors_data, NULL);
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

