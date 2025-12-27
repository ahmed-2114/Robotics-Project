#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <ESP32Servo.h>

// Servo Objects
Servo servo_j1;
Servo servo_j2;
Servo servo_j3;
Servo servo_j4;
Servo servo_gripper;

// Pins
#define PIN_J1 13
#define PIN_J2 12
#define PIN_J3 14
#define PIN_J4 27
#define PIN_GRIPPER 26

// ROS Objects
rcl_subscription_t subscriber;
std_msgs__msg__Float32MultiArray msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

// Error handle
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(2, !digitalRead(2)); // Flash onboard LED on error
    delay(100);
  }
}

// Callback Function
void subscription_callback(const void * msgin) {
  const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  
  // We expect 5 values: [J1, J2, J3, J4, J5 (Gripper)]
  if (msg->data.size >= 5) {
    float j1_rad = msg->data.data[0];
    float j2_rad = msg->data.data[1];
    float j3_rad = msg->data.data[2];
    float j4_rad = msg->data.data[3];
    float j5_rad = msg->data.data[4]; 

    // 1. Convert Radians to Degrees (Standard Mapping)
    int j1_deg = (j1_rad * 180.0) / 3.14159;
    int j2_deg = (j2_rad * 180.0) / 3.14159;
    int j3_deg = (j3_rad * 180.0) / 3.14159;
    int j4_deg = (j4_rad * 180.0) / 3.14159;
    
    // 2. Apply Offsets for Real Life Hardware
    // J3: MoveIt 0 -> Real 180. Logic: 180 - calculated_angle (if it moves inversely) or 180 + angle
    // Assuming standard direction: Real = 180 - MoveIt_Angle (to flip 0->180)
    // If it moves the wrong way, change to: j3_deg = 180 + j3_deg;
    j3_deg = abs(180 - j3_deg); 

    // J4: MoveIt 0 -> Real 90. 
    // Real = 90 + MoveIt_Angle
    j2_deg = 90 - j2_deg;

    // 3. Gripper Logic (Map J5 directly)
    int grip_deg = (j5_rad * 180.0) / 3.14159; 
    
    // 4. Clamp values to safe servo range (0-180)
    j1_deg = constrain(j1_deg, 0, 180);
    j2_deg = constrain(j2_deg, 0, 180);
    j3_deg = constrain(j3_deg, 0, 180);
    j4_deg = constrain(j4_deg, 0, 180);
    grip_deg = constrain(grip_deg, 0, 180);

    // 5. Move Servos
    servo_j1.write(j1_deg);
    servo_j2.write(j2_deg);
    servo_j3.write(j3_deg);
    servo_j4.write(j4_deg);
    servo_gripper.write(grip_deg);
  }
}

void setup() {
  set_microros_transports();
  
  servo_j1.attach(PIN_J1);
  servo_j2.attach(PIN_J2);
  servo_j3.attach(PIN_J3);
  servo_j4.attach(PIN_J4);
  servo_gripper.attach(PIN_GRIPPER);
  
  pinMode(2, OUTPUT);
  delay(2000);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/robot_arm_command"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  
  static float memory[10];
  msg.data.capacity = 10;
  msg.data.data = memory;
  msg.data.size = 0;
}

void loop() {
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}