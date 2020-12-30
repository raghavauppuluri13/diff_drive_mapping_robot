#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <math.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <maze_robot/MazeRobotState.h>

#define left_enc_PIN 2
#define right_enc_PIN 3
#define MAX_PWM 125

#define TOTAL_TICKS 28
#define MOTOR_DELAY 10.0 // ms delay for motor to speed up
#define DELTA_T 100.0 // ms between speed measurements

void incLeftTicks();
void incRightTicks();
double calculateAngularSpeed(volatile unsigned long*);
void controlMotor(float, float, uint8_t*, Adafruit_DCMotor*);
void print_fl(float);

// ROS setup
maze_robot::MazeRobotState robot_state;
ros::NodeHandle nh;

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *leftMotor = AFMS.getMotor(2);

Adafruit_DCMotor *rightMotor = AFMS.getMotor(1);

// CONSTANTS

const float target_vel = 45.0; // cm/s

//   PID Controller
const float Kp = 0.15;
const float Ki = 0.02;
const float Kd = 0.02;
const float I_err_range = 5.0;
float err = 0;
float err_sum = 0;

// GLOBALS

  // Right motor
volatile unsigned long rt_tick_ct = 0; // tick count for right wheel
uint8_t rt_dir = FORWARD; // Direction of rotation for right wheel
uint8_t rt_PWM = 0;

  // Left motor
volatile unsigned long lt_tick_ct = 0; // tick count for left wheel
uint8_t lt_dir = FORWARD; // Direction of rotation left wheel
uint8_t lt_PWM = 0;

// Controls motor using PID control
void controlMotor(float target_vel, float curr_speed, uint8_t *PWM, Adafruit_DCMotor *motor)
{
  float P; // Proportional term
  float I; // Integral term
  float D; // Derivative term
  float prev_err;

  if(target_vel > 0)
  {
    motor->run(FORWARD);
  }
  else if(target_vel < 0)
  { 
    motor->run(BACKWARD);
  }
  else{
    motor->run(RELEASE);
  }
  
  prev_err = err;
  err = target_vel - curr_speed;
  
  // Mitigates Integral Windup
  if(err < I_err_range){
    err_sum += err;
  }
  else{
    err_sum = 0;
  }

  P = Kp * err;
  I = Ki * err_sum;
  D = Kd * ((err - prev_err) / (MOTOR_DELAY + DELTA_T));
  
  nh.loginfo("P-term: ");
  print_fl(P);

  nh.loginfo("I-term: ");
  print_fl(I);

  nh.loginfo("D-term: ");
  print_fl(D);
 
  *PWM = *PWM + P + I + D;
  *PWM = *PWM % MAX_PWM;
  
  nh.loginfo("PWM: ");
  print_fl(*PWM);
  
  motor->setSpeed(*PWM);
  delay(MOTOR_DELAY);
}

// Calculates angular speed in rad/s
double calculateAngularSpeed(volatile unsigned long *tick_ct){
  int init_tick_ct;
  int final_tick_ct;
  
  init_tick_ct = *tick_ct;
  delay(DELTA_T);
  final_tick_ct = *tick_ct;

  return ((double) final_tick_ct - init_tick_ct) / TOTAL_TICKS * 2 * M_PI / (DELTA_T / 1000);
}

void print_fl(float fl)
{
  char print_str[20];
  dtostrf(fl, 9, 3, print_str);
  nh.loginfo(print_str);

}

void vel_control_cb(const geometry_msgs::Twist& cmd_vel) {
  nh.loginfo("v_x:");
  print_fl(cmd_vel.linear.x);
  
  nh.loginfo("v_y:");
  print_fl(cmd_vel.linear.y);
  
  nh.loginfo("v_z:");
  print_fl(cmd_vel.linear.z);

  nh.loginfo("w_x:");
  print_fl(cmd_vel.angular.x);

  nh.loginfo("w_y:");
  print_fl(cmd_vel.angular.y);

  nh.loginfo("w_z:");
  print_fl(cmd_vel.angular.z);
}

// increments ticks for left motor
void incLeftTicks() {
  lt_tick_ct++;
}

// increments ticks for right motor
void incRightTicks() { 
  rt_tick_ct++;
}

ros::Publisher pub_maze_robot_state("maze_robot_state", &robot_state); // publishes wheel angular velocities
ros::Subscriber<geometry_msgs::Twist> vel_control("cmd_vel", vel_control_cb);

void setup() {
  nh.initNode();
  nh.advertise(pub_maze_robot_state);
  
  pinMode(left_enc_PIN, INPUT_PULLUP);
  pinMode(right_enc_PIN, INPUT_PULLUP);
  
  AFMS.begin();  // create with the default frequency 1.6KHz

  // Set the speed to start, from 0 (off) to 255 (max speed)
  leftMotor->setSpeed(lt_PWM);
  leftMotor->run(lt_dir);
  // turn on motor
  leftMotor->run(RELEASE);
  
  // Set the speed to start, from 0 (off) to 255 (max speed)
  rightMotor->setSpeed(rt_PWM);
  rightMotor->run(rt_dir);
  // turn on motor
  rightMotor->run(RELEASE);

  attachInterrupt(digitalPinToInterrupt(left_enc_PIN), incLeftTicks, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_enc_PIN), incRightTicks, CHANGE);
}

void loop() {
  robot_state.left_ang_vel = calculateAngularSpeed(&lt_tick_ct);
  robot_state.right_ang_vel = calculateAngularSpeed(&rt_tick_ct);

  pub_maze_robot_state.publish(&robot_state);  
  //controlMotor(target_vel, robot_state.left_ang_vel, &lt_PWM, leftMotor);
  //controlMotor(target_vel, robot_state.right_ang_vel, &rt_PWM, rightMotor);
  
  //pub_maze_robot_state.publish(&robot_state); 
  
  //nh.loginfo("Left Motor Speed: ");
  //print_fl(robot_state.left_ang_vel);
  //nh.loginfo("Right Motor Speed: ");
  //print_fl(robot_state.right_ang_vel);

  nh.spinOnce();
  delay(3);
}


