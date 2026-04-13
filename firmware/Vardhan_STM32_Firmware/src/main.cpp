#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include <sensor_msgs/msg/range.h>

// ==========================================
// PINOUT (STM32 L432KC)
// ==========================================
#define PWM_LEFT PA8
#define DIR1_LEFT PA11
#define DIR2_LEFT PA12

#define PWM_RIGHT PB3
#define DIR1_RIGHT PB4
#define DIR2_RIGHT PB5

#define ENCA_FL PA0
#define ENCB_FL PA1
#define ENCA_RL PA3
#define ENCB_RL PA4
#define ENCA_FR PA5
#define ENCB_FR PA6
#define ENCA_RR PA9
#define ENCB_RR PA10

#define TRIG_PIN PB0
#define ECHO_PIN PB1

// ==========================================
// ROS2 VARIABLES
// ==========================================
rcl_subscription_t twist_sub;
rcl_publisher_t range_pub;
rcl_publisher_t encoder_pub;

geometry_msgs__msg__Twist twist_msg;
sensor_msgs__msg__Range range_msg;
geometry_msgs__msg__Vector3 encoder_msg; // x = left ticks, y = right ticks

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t sensor_timer;

// ==========================================
// HARDWARE STATE VARIABLES
// ==========================================
volatile long ticks_FL = 0;
volatile long ticks_RL = 0;
volatile long ticks_FR = 0;
volatile long ticks_RR = 0;

const float TRACK_WIDTH = 0.25; // Distance between left and right wheels (meters)
const float MAX_SPEED_MS = 2.5; // max safer speed

// Error handling macro
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    delay(100);
  }
}

// ==========================================
// INTERRUPT SERVICE ROUTINES (ENCODERS)
// ==========================================
void isr_FL() { if(digitalRead(ENCB_FL) == HIGH) ticks_FL++; else ticks_FL--; }
void isr_RL() { if(digitalRead(ENCB_RL) == HIGH) ticks_RL++; else ticks_RL--; }
void isr_FR() { if(digitalRead(ENCB_FR) == HIGH) ticks_FR++; else ticks_FR--; }
void isr_RR() { if(digitalRead(ENCB_RR) == HIGH) ticks_RR++; else ticks_RR--; }

// ==========================================
// MOTOR CONTROL LOGIC
// ==========================================
void set_motors(float linear_x, float angular_z) {
  // Differential drive kinematics
  float v_left = linear_x - (angular_z * TRACK_WIDTH / 2.0);
  float v_right = linear_x + (angular_z * TRACK_WIDTH / 2.0);

  // Convert m/s to PWM (0-255)
  int pwm_left = (abs(v_left) / MAX_SPEED_MS) * 255;
  int pwm_right = (abs(v_right) / MAX_SPEED_MS) * 255;

  pwm_left = constrain(pwm_left, 0, 255);
  pwm_right = constrain(pwm_right, 0, 255);

  // Left Side Drive
  if (v_left > 0) {
    digitalWrite(DIR1_LEFT, HIGH); digitalWrite(DIR2_LEFT, LOW);
  } else if (v_left < 0) {
    digitalWrite(DIR1_LEFT, LOW); digitalWrite(DIR2_LEFT, HIGH);
  } else {
    digitalWrite(DIR1_LEFT, LOW); digitalWrite(DIR2_LEFT, LOW);
  }
  analogWrite(PWM_LEFT, pwm_left);

  // Right Side Drive
  if (v_right > 0) {
    digitalWrite(DIR1_RIGHT, HIGH); digitalWrite(DIR2_RIGHT, LOW);
  } else if (v_right < 0) {
    digitalWrite(DIR1_RIGHT, LOW); digitalWrite(DIR2_RIGHT, HIGH);
  } else {
    digitalWrite(DIR1_RIGHT, LOW); digitalWrite(DIR2_RIGHT, LOW);
  }
  analogWrite(PWM_RIGHT, pwm_right);
}

// ==========================================
// ROS2 CALLBACKS
// ==========================================
void twist_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  set_motors(msg->linear.x, msg->angular.z);
}

void sensor_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // 1. Read and Publish Ultrasonic
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    // Timeout of 20000us (~3.4 meters) to prevent blocking the micro-ROS executor
    long duration = pulseIn(ECHO_PIN, HIGH, 20000); 
    float distance = (duration == 0) ? 4.0 : (duration * 0.0343) / 2.0;

    range_msg.range = distance;
    RCSOFTCHECK(rcl_publish(&range_pub, &range_msg, NULL));

    // 2. Read and Publish Encoders (Averaged Left and Right)
    encoder_msg.x = (ticks_FL + ticks_RL) / 2.0;
    encoder_msg.y = (ticks_FR + ticks_RR) / 2.0;
    encoder_msg.z = 0.0;
    RCSOFTCHECK(rcl_publish(&encoder_pub, &encoder_msg, NULL));
  }
}

// ==========================================
// MAIN SETUP & LOOP
// ==========================================
void setup() {
  set_microros_transports();

  // Initialize Motor Pins
  pinMode(PWM_LEFT, OUTPUT); pinMode(DIR1_LEFT, OUTPUT); pinMode(DIR2_LEFT, OUTPUT);
  pinMode(PWM_RIGHT, OUTPUT); pinMode(DIR1_RIGHT, OUTPUT); pinMode(DIR2_RIGHT, OUTPUT);

  // Initialize Ultrasonic Pins
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);

  // Initialize Encoder Pins & Interrupts
  pinMode(ENCA_FL, INPUT_PULLUP); pinMode(ENCB_FL, INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(ENCA_FL), isr_FL, RISING);
  pinMode(ENCA_RL, INPUT_PULLUP); pinMode(ENCB_RL, INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(ENCA_RL), isr_RL, RISING);
  pinMode(ENCA_FR, INPUT_PULLUP); pinMode(ENCB_FR, INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(ENCA_FR), isr_FR, RISING);
  pinMode(ENCA_RR, INPUT_PULLUP); pinMode(ENCB_RR, INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(ENCA_RR), isr_RR, RISING);

  delay(2000);

  // Setup micro-ROS
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "stm32_chassis_node", "", &support));

  // Publisher: Ultrasonic
  RCCHECK(rclc_publisher_init_default(
    &range_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range), "/ultrasonic/range"));
  range_msg.header.frame_id.data = (char*)"ultrasonic_link";

  // Publisher: Encoders
  RCCHECK(rclc_publisher_init_default(
    &encoder_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "/wheel_ticks"));

  // Subscriber: cmd_vel
  RCCHECK(rclc_subscription_init_default(
    &twist_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel"));

  // Timer: 20Hz (50ms) for sensors
  RCCHECK(rclc_timer_init_default(&sensor_timer, &support, RCL_MS_TO_NS(50), sensor_timer_callback));

  // Executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &twist_sub, &twist_msg, &twist_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &sensor_timer));
}

void loop() {
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}