#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float64_multi_array.h>

rcl_subscription_t subscriber;
std_msgs__msg__Float64MultiArray msg;
rcl_publisher_t publisher;  // Publisher for /state_feedback
std_msgs__msg__Float64MultiArray feedback_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

double desired_ang_vel_r = 0.0;
double desired_ang_vel_l = 0.0;

// State feedback variables
double state1 = 1.0;
double state2 = 2.0;
double state3 = 3.0;
double state4 = 4.0;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while (1) {
    delay(100);
  }
}

void subscription_callback(const void * msgin) {  
  const std_msgs__msg__Float64MultiArray * msg = (const std_msgs__msg__Float64MultiArray *)msgin;

  if (msg->data.size >= 2) {
    desired_ang_vel_r = msg->data.data[0];
    desired_ang_vel_l = msg->data.data[1];

    // Turn LED on/off based on angular velocities
    if (desired_ang_vel_r == 0 && desired_ang_vel_l == 0) {
      digitalWrite(LED_BUILTIN, HIGH); // Turn LED on
    } else {
      digitalWrite(LED_BUILTIN, LOW); // Turn LED off
    }
  }
}

void setup() {
  set_microros_transports();

  // Configure LED_BUILTIN as an output for the LED
  pinMode(LED_BUILTIN, OUTPUT);

  allocator = rcl_get_default_allocator();

  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // Create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
    "wheel_velocities"));

  // Create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
    "state_feedback"));

  // Initialize feedback message
  feedback_msg.data.capacity = 4;
  feedback_msg.data.size = 4;
  feedback_msg.data.data = (double *)malloc(feedback_msg.data.capacity * sizeof(double));

  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  // Update the feedback message with state variables
  feedback_msg.data.data[0] = state1;
  feedback_msg.data.data[1] = state2;
  feedback_msg.data.data[2] = state3;
  feedback_msg.data.data[3] = state4;

  // Publish the state feedback message
  RCSOFTCHECK(rcl_publish(&publisher, &feedback_msg, NULL));

  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
