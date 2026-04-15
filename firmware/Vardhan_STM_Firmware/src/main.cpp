#include <Arduino.h>
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include <sensor_msgs/msg/range.h>

// ================= PINOUT =================
#define PWM_LEFT   PA8
#define DIR1_LEFT  PA11
#define DIR2_LEFT  PA12

#define PWM_RIGHT  PB3
#define DIR1_RIGHT PB4
#define DIR2_RIGHT PB5

// Front encoders only
#define ENCA_FL PA0
#define ENCB_FL PA1
#define ENCA_FR PA5
#define ENCB_FR PA6

#define TRIG_PIN PB0
#define ECHO_PIN PB1

// ================= ROS =================
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

rcl_subscription_t twist_sub;
rcl_publisher_t range_pub;
rcl_publisher_t encoder_pub;
rcl_timer_t sensor_timer;

geometry_msgs__msg__Twist twist_msg;
sensor_msgs__msg__Range range_msg;
geometry_msgs__msg__Vector3 encoder_msg;

// ================= VARIABLES =================
volatile long ticks_FL = 0;
volatile long ticks_FR = 0;

const float TRACK_WIDTH = 0.25;
const float MAX_SPEED_MS = 2.5;

// ================= ERROR HANDLING =================
#define RCCHECK(fn) { if ((fn) != RCL_RET_OK) error_loop(); }
#define RCSOFTCHECK(fn) { (void)(fn); }

void error_loop() {
  while (1) delay(100);
}

// ================= ENCODER ISR =================
void isr_FL() {
  if (digitalRead(ENCB_FL)) ticks_FL++;
  else ticks_FL--;
}

void isr_FR() {
  if (digitalRead(ENCB_FR)) ticks_FR++;
  else ticks_FR--;
}

// ================= MOTOR CONTROL =================
void set_motors(float linear_x, float angular_z) {
  float v_left  = linear_x - (angular_z * TRACK_WIDTH / 2.0);
  float v_right = linear_x + (angular_z * TRACK_WIDTH / 2.0);

  int pwm_left  = constrain((abs(v_left)  / MAX_SPEED_MS) * 255, 0, 255);
  int pwm_right = constrain((abs(v_right) / MAX_SPEED_MS) * 255, 0, 255);

  // Left motor
  digitalWrite(DIR1_LEFT, v_left > 0);
  digitalWrite(DIR2_LEFT, v_left < 0);
  analogWrite(PWM_LEFT, pwm_left);

  // Right motor
  digitalWrite(DIR1_RIGHT, v_right > 0);
  digitalWrite(DIR2_RIGHT, v_right < 0);
  analogWrite(PWM_RIGHT, pwm_right);
}

// ================= CALLBACKS =================
void twist_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  set_motors(msg->linear.x, msg->angular.z);
}

void sensor_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);

  if (!timer) return;

  // Ultrasonic
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 20000);
  float distance = (duration == 0) ? 4.0 : (duration * 0.0343) / 2.0;

  range_msg.range = distance;
  RCSOFTCHECK(rcl_publish(&range_pub, &range_msg, NULL));

  // Encoders
  encoder_msg.x = (float)ticks_FL;
  encoder_msg.y = (float)ticks_FR;
  encoder_msg.z = 0.0;

  RCSOFTCHECK(rcl_publish(&encoder_pub, &encoder_msg, NULL));
}

// ================= SETUP =================
void setup() {
  set_microros_transports();

  // Motor pins
  pinMode(PWM_LEFT, OUTPUT);
  pinMode(DIR1_LEFT, OUTPUT);
  pinMode(DIR2_LEFT, OUTPUT);

  pinMode(PWM_RIGHT, OUTPUT);
  pinMode(DIR1_RIGHT, OUTPUT);
  pinMode(DIR2_RIGHT, OUTPUT);

  // Ultrasonic
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Encoders
  pinMode(ENCA_FL, INPUT_PULLUP);
  pinMode(ENCB_FL, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA_FL), isr_FL, RISING);

  pinMode(ENCA_FR, INPUT_PULLUP);
  pinMode(ENCB_FR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA_FR), isr_FR, RISING);

  delay(2000);

  // micro-ROS init
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "stm32_chassis_node", "", &support));

  // Publishers
  RCCHECK(rclc_publisher_init_default(
    &range_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
    "/ultrasonic/range"));

  range_msg.header.frame_id.data = (char*)"ultrasonic_link";

  RCCHECK(rclc_publisher_init_default(
    &encoder_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "/wheel_ticks"));

  // Subscriber
  RCCHECK(rclc_subscription_init_default(
    &twist_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel"));

  // Timer
  RCCHECK(rclc_timer_init_default(
    &sensor_timer, &support,
    RCL_MS_TO_NS(50),
    sensor_timer_callback));

  // Executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &twist_sub, &twist_msg, &twist_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &sensor_timer));
}

// ================= LOOP =================
void loop() {
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}