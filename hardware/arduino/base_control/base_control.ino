#include <micro_ros_arduino.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>

#include <rmw_microros/rmw_microros.h>

//================pin setup==============

#define LED_PIN 2
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = uxr_millis();} \
    if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
  } while (0)\
    //================PWM SETUP==============
const int freq = 50;
const int m1_pwm_channel = 1;
const int m2_pwm_channel = 2;
const int m3_pwm_channel = 3;
const int m4_pwm_channel = 4;
const int resolution = 8;

//=======================================
#define m1_dir_pin 14     //left top
#define m1_pwm_pin 12

#define m2_dir_pin 27      //right top
#define m2_pwm_pin 26

#define m3_dir_pin 15     //left back
#define m3_pwm_pin 4
  
#define m4_dir_pin 16      //right back
#define m4_pwm_pin 17

#define encoder1_a 32  //fl
#define encoder1_b 33  

#define encoder2_a 35  //fr
#define encoder2_b 34  

#define encoder4_a 19  //rr
#define encoder4_b 18  

#define encoder3_a 36  //rl
#define encoder3_b 39  


////////////////Variables/////////////////////
rcl_allocator_t allocator;
rclc_support_t support;
rcl_timer_t timer;
rcl_node_t node;
rclc_executor_t executor;

rcl_publisher_t m1_tick_pub;
std_msgs__msg__Int32 m1_tick;

rcl_publisher_t m2_tick_pub;
std_msgs__msg__Int32 m2_tick;

rcl_publisher_t m3_tick_pub;
std_msgs__msg__Int32 m3_tick;

rcl_publisher_t m4_tick_pub;
std_msgs__msg__Int32 m4_tick;

rcl_publisher_t feedback_vel_pub;
geometry_msgs__msg__Twist feedback_vel_msg;

rcl_publisher_t feedback_vel_pub2;
geometry_msgs__msg__Twist feedback_vel_msg2;

rcl_subscription_t m1_command_sub;  ////////////motor1
std_msgs__msg__Float32 m1_command;

rcl_subscription_t m2_command_sub;  ////////////motor2
std_msgs__msg__Float32 m2_command;

rcl_subscription_t m3_command_sub;  ////////////motor3
std_msgs__msg__Float32 m3_command;

rcl_subscription_t m4_command_sub;  ////////////motor4
std_msgs__msg__Float32 m4_command;

rcl_subscription_t pid_gain_sub;
geometry_msgs__msg__Vector3 pid_gain;

bool micro_ros_init_successful;

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

unsigned int SPIN_FREQ = 20;
float windup_guard = 20;

// PID control parameters
float kp = 0.02;
float ki = 0.004;
float kd = 0.0;

//Motor pulse parameters
float PPR = 2160; // (From manufacturer)
long p1, p2, p3, p4;

//M1 Wheel control parameter
unsigned long curTime1, prevTime1, dt1;
long curTick1, prevTick1, diffTick1;
double err1, prev_err1, sumErr1, dErr1, setRPM1;
double control_out1;
double measuredRPM1;
double desiredRPM1;

//M2 Wheel control parameter
unsigned long curTime2, prevTime2, dt2;
long curTick2, prevTick2, diffTick2;
double err2, prev_err2, sumErr2, dErr2, setRPM2;
double control_out2;
double measuredRPM2;
double desiredRPM2;

//M3 Wheel control parameter
unsigned long curTime3, prevTime3, dt3;
long curTick3, prevTick3, diffTick3;
double err3, prev_err3, sumErr3, dErr3, setRPM3;
double control_out3;
double measuredRPM3;
double desiredRPM3;

//M4 Wheel control parameter
unsigned long curTime4, prevTime4, dt4;
long curTick4, prevTick4, diffTick4;
double err4, prev_err4, sumErr4, dErr4, setRPM4;
double control_out4;
double measuredRPM4;
double desiredRPM4;

//Find max value function
float max(float num1, float num2)
{
  return (num1 > num2) ? num1 : num2;
}

//Find min value function
float min(float num1, float num2)
{
  return (num1 > num2) ? num2 : num1;
}

// Motor1 Controller Function
void computePID1(double control_cmd,
                 long inTick)
{
  int dirs = 1;  //1 for forward direction
  curTick1 = inTick;
  curTime1 = millis();

  setRPM1 = control_cmd;
  dirs = control_cmd / abs(control_cmd);

  diffTick1 = curTick1 - prevTick1;
  dt1 =  (curTime1 - prevTime1);

  measuredRPM1 = ((diffTick1 / PPR) / (dt1 * 0.001)) * 60;

  err1 = abs(setRPM1) - abs(measuredRPM1);

  sumErr1 += err1 * dt1;

  dErr1 = (err1 - prev_err1) / dt1;

  control_out1 = kp * err1 + ki * sumErr1 + kd * dErr1;

  prev_err1 = err1;
  prevTick1 = curTick1;
  prevTime1 = curTime1;

  if (control_cmd != 0)
  {
    if (dirs > 0.5)
    {
      analogWrite(m1_dir_pin, control_out1);
      analogWrite(m1_pwm_pin, 0);
    }
    else if (dirs < -0.5)
    {
      analogWrite(m1_dir_pin, 0);
      analogWrite(m1_pwm_pin, control_out1);
    }
  }
  else
  {
    analogWrite(m1_dir_pin, 0);
    analogWrite(m1_pwm_pin, 0);
  }

}

// Motor3 Controller Function
void computePID3(double control_cmd,
                 long inTick)
{
  int dirs = 1;  //1 for forward direction
  curTick3 = inTick;
  curTime3 = millis();

  setRPM3 = control_cmd;
  dirs = control_cmd / abs(control_cmd);

  diffTick3 = curTick3 - prevTick3;
  dt3 =  (curTime3 - prevTime3);

  measuredRPM3 = ((diffTick3 / PPR) / (dt3 * 0.001)) * 60;

  err3 = abs(setRPM3) - abs(measuredRPM3);

  sumErr3 += err3 * dt3;

  dErr3 = (err3 - prev_err3) / dt3;

  control_out3 = kp * err3 + ki * sumErr3 + kd * dErr3;

  prev_err3 = err3;
  prevTick3 = curTick3;
  prevTime3 = curTime3;

  if (control_cmd != 0)
  {
    if (dirs > 0.5)
    {
      analogWrite(m3_dir_pin, 0);
      analogWrite(m3_pwm_pin, control_out3);
    }
    else if (dirs < -0.5)
    {
      analogWrite(m3_dir_pin, control_out3);
      analogWrite(m3_pwm_pin, 0);
    }
  }
  else
  {
    analogWrite(m3_dir_pin, 0);
    analogWrite(m3_pwm_pin, 0);
  }

}

// Motor2 Controller Function
void computePID2(double control_cmd,
                 long inTick)
{
  int dirs = 1;  //1 for forward direction
  curTick2 = inTick;
  curTime2 = millis();

  setRPM2 = control_cmd;
  dirs = control_cmd / abs(control_cmd);

  diffTick2 = curTick2 - prevTick2;
  dt2 =  (curTime2 - prevTime2);

  measuredRPM2 = ((diffTick2 / PPR) / (dt2 * 0.001)) * 60;

  err2 = abs(setRPM2) - abs(measuredRPM2);

  sumErr2 += err2 * dt2;

  dErr2 = (err2 - prev_err2) / dt2;

  control_out2 = kp * err2 + ki * sumErr2 + kd * dErr2;

  prev_err2 = err2;
  prevTick2 = curTick2;
  prevTime2 = curTime2;

  if (control_cmd != 0)
  {
    if (dirs > 0.5)
    {
      analogWrite(m2_dir_pin, control_out2);
      analogWrite(m2_pwm_pin, 0);
    }
    else if (dirs < -0.5)
    {
      analogWrite(m2_dir_pin, 0);
      analogWrite(m2_pwm_pin, control_out2);
    }
  }
  else
  {
    analogWrite(m2_dir_pin, 0);
    analogWrite(m2_pwm_pin, 0);
  }

}
// Motor4 Controller Function
void computePID4(double control_cmd,
                 long inTick)
{
  int dirs = 1;  //1 for forward direction
  curTick4 = inTick;
  curTime4 = millis();

  setRPM4 = control_cmd;
  dirs = control_cmd / abs(control_cmd);

  diffTick4 = curTick4 - prevTick4;
  dt4 =  (curTime4 - prevTime4);

  measuredRPM4 = ((diffTick4 / PPR) / (dt4 * 0.001)) * 60;

  err4 = abs(setRPM4) - abs(measuredRPM4);

  sumErr4 += err4 * dt4;

  dErr4 = (err4 - prev_err4) / dt4;

  control_out4 = kp * err4 + ki * sumErr4 + kd * dErr4;

  prev_err4 = err4;
  prevTick4 = curTick4;
  prevTime4 = curTime4;

  if (control_cmd != 0)
  {
    if (dirs > 0.5)
    {
      analogWrite(m4_dir_pin, 0);
      analogWrite(m4_pwm_pin, control_out4);
    }
    else if (dirs < -0.5)
    {
      analogWrite(m4_dir_pin, control_out4);
      analogWrite(m4_pwm_pin, 0);
    }
  }
  else
  {
    analogWrite(m4_dir_pin, 0);
    analogWrite(m4_pwm_pin, 0);
  }

}

void pid_gain_callback(const void * msgin)
{
  const geometry_msgs__msg__Vector3 * gain_value = (const geometry_msgs__msg__Vector3 *)msgin;

  //Uncomment for tuning
  /*kp = gain_value->x;

    ki = gain_value->y;
    kd = gain_value->z;*/
}

// wheel 1 command callback function
void wheel1_callback(const void * msgin)
{
  const std_msgs__msg__Float32 * power = (const std_msgs__msg__Float32 *)msgin;

  desiredRPM1 = power->data;

}
// wheel 2 command callback function
void wheel2_callback(const void * msgin)
{
  const std_msgs__msg__Float32 * power = (const std_msgs__msg__Float32 *)msgin;

  desiredRPM2 = power->data;

}
// wheel 3 command callback function
void wheel3_callback(const void * msgin)
{
  const std_msgs__msg__Float32 * power = (const std_msgs__msg__Float32 *)msgin;

  desiredRPM3 = power->data;

}
// wheel 4 command callback function
void wheel4_callback(const void * msgin)
{
  const std_msgs__msg__Float32 * power = (const std_msgs__msg__Float32 *)msgin;

  desiredRPM4 = power->data;

}

//Timer callback function
void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  (void) last_call_time;
  if (timer != NULL) {
    rcl_publish(&feedback_vel_pub, &feedback_vel_msg, NULL);
    rcl_publish(&feedback_vel_pub2, &feedback_vel_msg2, NULL);

    RCLC_UNUSED(timer);
  }

}

//Create entities function
bool create_entities()
{
  allocator = rcl_get_default_allocator();


  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  RCCHECK(rclc_node_init_default(&node, "base_control_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
            &pid_gain_sub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
            "pid_gain"));

  RCCHECK(rclc_subscription_init_default(
            &m1_command_sub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "wheel1_command"));

  RCCHECK(rclc_subscription_init_default(
            &m2_command_sub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "wheel2_command"));

  RCCHECK(rclc_subscription_init_default(
            &m3_command_sub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "wheel3_command"));

  RCCHECK(rclc_subscription_init_default(
            &m4_command_sub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "wheel4_command"));


  RCCHECK(rclc_publisher_init_default(
            &m1_tick_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "m1_tick"));

  RCCHECK(rclc_publisher_init_default(
            &m2_tick_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "m2_tick"));

  RCCHECK(rclc_publisher_init_default(
            &m3_tick_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "m3_tick"));

  RCCHECK(rclc_publisher_init_default(
            &m4_tick_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "m4_tick"));

  RCCHECK(rclc_publisher_init_default(
            &feedback_vel_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            "feedback_vel"));

  RCCHECK(rclc_publisher_init_default(
            &feedback_vel_pub2,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            "feedback_vel2"));

  RCCHECK(rclc_timer_init_default(
            &timer,
            &support,
            RCL_MS_TO_NS(50),
            timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 6, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &pid_gain_sub, &pid_gain, &pid_gain_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &m1_command_sub, &m1_command, &wheel1_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &m2_command_sub, &m2_command, &wheel2_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &m3_command_sub, &m3_command, &wheel3_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &m4_command_sub, &m4_command, &wheel4_callback, ON_NEW_DATA));

  return true;
}

void destroy_entities()
{
  rcl_publisher_fini(&m1_tick_pub, &node);
  rcl_publisher_fini(&m2_tick_pub, &node);
  rcl_publisher_fini(&m3_tick_pub, &node);
  rcl_publisher_fini(&m4_tick_pub, &node);
  rcl_publisher_fini(&feedback_vel_pub, &node);
  rcl_publisher_fini(&feedback_vel_pub2, &node);
  rcl_subscription_fini(&m1_command_sub, &node);
  rcl_subscription_fini(&m2_command_sub, &node);
  rcl_subscription_fini(&m3_command_sub, &node);
  rcl_subscription_fini(&m4_command_sub, &node);
  rcl_subscription_fini(&pid_gain_sub, &node);

  rcl_node_fini(&node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rclc_support_fini(&support);

  micro_ros_init_successful = false;
}

void setup() {
  set_microros_transports();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  //PWM pin setup
  pinMode(m1_dir_pin,OUTPUT);
  pinMode(m2_dir_pin,OUTPUT);
  pinMode(m3_dir_pin,OUTPUT);
  pinMode(m4_dir_pin,OUTPUT);
  
  ledcSetup(m1_pwm_channel, freq, resolution);
  ledcSetup(m2_pwm_channel, freq, resolution);
  ledcSetup(m3_pwm_channel, freq, resolution);
  ledcSetup(m4_pwm_channel, freq, resolution);

  ledcAttachPin(m1_pwm_pin, m1_pwm_channel);
  ledcAttachPin(m2_pwm_pin, m2_pwm_channel);
  ledcAttachPin(m3_pwm_pin, m3_pwm_channel);
  ledcAttachPin(m4_pwm_pin, m4_pwm_channel);


  pinMode(encoder1_a, INPUT_PULLUP);
  pinMode(encoder1_b, INPUT_PULLUP);
  pinMode(encoder2_a, INPUT_PULLUP);
  pinMode(encoder2_b, INPUT_PULLUP);
  pinMode(encoder3_a, INPUT_PULLUP);
  pinMode(encoder3_b, INPUT_PULLUP);
  pinMode(encoder4_a, INPUT_PULLUP);
  pinMode(encoder4_b, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(encoder1_a), encoder_isr_1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2_a), encoder_isr_2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder3_a), encoder_isr_3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder4_a), encoder_isr_4, CHANGE);

  state = WAITING_AGENT;

}

long curRPM_time;
long prevRPM_time;
long curP1;
long curP2;
long curP3;
long curP4;
long prevP1;
long prevP2;
long prevP3;
long prevP4;

long diffRPM_time;
float RPM_1, RPM_2, RPM_3, RPM_4;

void counter_RPM(long inP1, long inP2, long inP3, long inP4) {

  curP1 = inP1;
  curP2 = inP2;
  curP3 = inP3;
  curP4 = inP4;

  curRPM_time = millis();
  diffRPM_time = curRPM_time - prevRPM_time ;
  RPM_1 = (((curP1 - prevP1) / PPR) / (diffRPM_time * 0.001)) * 60;
  RPM_2 = (((curP2 - prevP2) / PPR) / (diffRPM_time * 0.001)) * 60;
  RPM_3 = (((curP3 - prevP3) / PPR) / (diffRPM_time * 0.001)) * 60;
  RPM_4 = (((curP4 - prevP4) / PPR) / (diffRPM_time * 0.001)) * 60;

  prevP1 = curP1;
  prevP2 = curP2;
  prevP3 = curP3;
  prevP4 = curP4;

  prevRPM_time = curRPM_time;

  feedback_vel_msg.linear.x = RPM_1;
  feedback_vel_msg.linear.y = RPM_2;
  feedback_vel_msg.angular.x = RPM_3;
  feedback_vel_msg.angular.y = RPM_4;
  feedback_vel_msg.angular.z = ki;

  feedback_vel_msg2.linear.x = control_out1; // RPM_l;
  feedback_vel_msg2.linear.y = control_out2;
  feedback_vel_msg2.angular.x = control_out3;
  feedback_vel_msg2.angular.y = control_out4;
  feedback_vel_msg2.angular.z = kp;

}

void encoder_isr_1() {

  int A = digitalRead(encoder1_a);
  int B = digitalRead(encoder1_b);

  if ((A == HIGH) != (B == LOW)) {
    p1--;
  } else {
    p1++;
  }
}

void encoder_isr_2() {

  int A = digitalRead(encoder2_a);
  int B = digitalRead(encoder2_b);

  if ((A == HIGH) != (B == LOW)) {
    p2--;
  } else {
    p2++;
  }
}

void encoder_isr_3() {

  int A = digitalRead(encoder3_a);
  int B = digitalRead(encoder3_b);

  if ((A == HIGH) != (B == LOW)) {
    p3--;
  } else {
    p3++;
  }
}

void encoder_isr_4() {

  int A = digitalRead(encoder4_a);
  int B = digitalRead(encoder4_b);

  if ((A == HIGH) != (B == LOW)) {
    p4--;
  } else {
    p4++;
  }
}

void counter_tick()
{
  m1_tick.data = p1;
  m2_tick.data = p2;
  m3_tick.data = p3;
  m4_tick.data = p4;
  
  counter_RPM(p1, p2, p3, p4);
  rcl_publish(&m1_tick_pub, &m1_tick, NULL);
  rcl_publish(&m2_tick_pub, &m2_tick, NULL);
  rcl_publish(&m3_tick_pub, &m3_tick, NULL);
  rcl_publish(&m4_tick_pub, &m4_tick, NULL);
}



void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        computePID1(desiredRPM1, p1);
        computePID2(desiredRPM2, p2);
        computePID3(desiredRPM3, p3);
        computePID4(desiredRPM4, p4);
        counter_tick();
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }

  if (state == AGENT_CONNECTED) {
    digitalWrite(LED_PIN, 1);
  } else {
    digitalWrite(LED_PIN, 0);
  }
}
