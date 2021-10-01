// Open loop motor control example
#include <Arduino.h>
#include <SimpleFOC.h>

#include <STM32TimerInterrupt.h>
#include <STM32_ISR_Timer.h>

#include <micro_ros_arduino.h>

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include <std_msgs/msg/float32.h>
#include <std_srvs/srv/set_bool.h>

#include <atomic>
#include <string>

#include "config.h"
#include "failsafe.h"

rcl_publisher_t position_pub;
rcl_subscription_t setpoint_sub;
std_msgs__msg__Float32 pub_msg;
std_msgs__msg__Float32 sub_msg;

rcl_service_t free_mode_srv;
std_srvs__srv__SetBool_Response response_msg;
std_srvs__srv__SetBool_Request request_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn)                                                            \
    {                                                                          \
        rcl_ret_t temp_rc = fn;                                                \
        if ((temp_rc != RCL_RET_OK)) {                                         \
            error_loop();                                                      \
        }                                                                      \
    }
#define RCSOFTCHECK(fn)                                                        \
    {                                                                          \
        rcl_ret_t temp_rc = fn;                                                \
        if ((temp_rc != RCL_RET_OK)) {                                         \
        }                                                                      \
    }

void error_loop() {
    while (1) {
        digitalWrite(LED_RED, !digitalRead(LED_RED));
        delay(500);
    }
}

constexpr int MOTOR_ID = 1;
constexpr int FAST_TIMER_INTERVAL_US = 150;
constexpr int SLOW_TIMER_INTERVAL_US = 100 * 1000;
constexpr float TORQUE_LIMIT = 1.0;

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(14);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH,
                                       A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);
InlineCurrentSense currentSense =
    InlineCurrentSense(0.003, -64.0 / 7.0, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

// encoder instance
HardwareEncoder encoder(2048);

// target variable
std::atomic<float> target(0);

void ros_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        pub_msg.data = motor.shaftVelocity();
        RCSOFTCHECK(rcl_publish(&position_pub, &pub_msg, NULL));
    }
}

void subscription_callback(const void *msgin) {
    const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
    target = msg->data;
}

STM32Timer FastTimer(TIM7);
STM32Timer SlowTimer(TIM8);

void disable_motor() { driver.voltage_limit = 0; }
void enable_motor() { driver.voltage_limit = 1.8f; }

void slow_timer_handler() {
    // temperature failsafe
    int32_t temp = static_cast<int32_t>(currentSense.getTemperature());
    if (is_safe(temp)) {
        enable_motor();
    } else {
        disable_motor();
    }
}

void fast_timer_handler() {
    motor.loopFOC();
    motor.move(target);
}

void service_callback(const void *request_msg, void *response_msg) {
    auto req_in =
        static_cast<const std_srvs__srv__SetBool_Request *>(request_msg);
    std_srvs__srv__SetBool_Response *res_in =
        static_cast<std_srvs__srv__SetBool_Response *>(response_msg);

    if (req_in->data) { // true for free-move-mode (zero torque setpoint), false
                        // for normal position control
        motor.controller = MotionControlType::torque;
        target = 0;
    } else {
        motor.controller = MotionControlType::angle;
        motor.sensor_offset =
            motor.shaft_angle; // declare current position as zero
        target = 0;
    }
}

void init_ros() {
    set_microros_transports();

    allocator = rcl_get_default_allocator();

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    // namespace stewart/<ID>
    std::string node_namespace = "stewart/";
    node_namespace += std::to_string(MOTOR_ID);
    RCCHECK(rclc_node_init_default(&node, "motor_control_node",
                                   node_namespace.c_str(), &support));

    // create subscriber
    RCCHECK(rclc_subscription_init_best_effort(
        &setpoint_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "position_target"));

    // create publisher
    RCCHECK(rclc_publisher_init_best_effort(
        &position_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "current_position"));

    // create free-mode service
    RCCHECK(rclc_service_init_default(
        &free_mode_srv, &node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool), "set_free_move"));

    // create timer,
    const unsigned int timer_timeout = 10;
    RCCHECK(rclc_timer_init_default(
        &timer, &support, RCL_MS_TO_NS(timer_timeout), ros_timer_callback));

    // create executor
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 3,
                               &allocator)); // the 3 is the number of handles
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &setpoint_sub, &sub_msg,
                                           &subscription_callback,
                                           ON_NEW_DATA));
}

void setup() {
    pinMode(LED_RED, OUTPUT);
    pinMode(A_POTENTIOMETER, INPUT_ANALOG);

    // wait for ROS initialisation
    init_ros();

    // initialize encoder sensor hardware
    encoder.init();
    motor.linkSensor(&encoder);

    // driver config
    driver.voltage_power_supply = 24;
    driver.pwm_frequency = 30000;
    driver.init();
    motor.linkDriver(&driver);

    // current sensing
    currentSense.init();
    motor.linkCurrentSense(&currentSense);

    motor.velocity_limit = 150;
    motor.PID_velocity.limit = 

    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor.torque_controller = TorqueControlType::foc_current;
    motor.controller = MotionControlType::angle;

    // set PID controller parameters
    config_motor_pid(&motor);
    motor.PID_velocity.limit = TORQUE_LIMIT;

    // init motor hardware
    motor.init();
    // align encoder and start FOC
    motor.initFOC();

    digitalWrite(LED_RED, HIGH); // LED signal for initialisation done

    // attach timer for motor driver
    FastTimer.attachInterruptInterval(FAST_TIMER_INTERVAL_US,
                                      fast_timer_handler);
    SlowTimer.attachInterruptInterval(SLOW_TIMER_INTERVAL_US,
                                      slow_timer_handler);
}

void loop() {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50)));
}
