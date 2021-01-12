//#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <maze_robot/MazeRobotState.h>
#include <PID_v1.h>
#include <Encoder.h>

#define LEFT_PIN 2
#define RIGHT_PIN 3
#define MAX_PWM 125

#define TICKS_PER_METER 263

#define KpR 2.0
#define KiR 0.0
#define KdR 0.0

#define KpL 0.15
#define KiL 0.0
#define KdL 0.0

void incLeftTicks();
void incRightTicks();
void print_fl(float);

// ROS setup
maze_robot::MazeRobotState robot_state;
ros::NodeHandle nh;

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *leftMotor = AFMS.getMotor(2);

Adafruit_DCMotor *rightMotor = AFMS.getMotor(1);


// GLOBALS

  // Right motor
volatile unsigned long right_ticks = 0; // tick count for right wheel

float rt_vel = 0; 
float rt_vel_tar = 0.5; 
uint8_t rt_dir = FORWARD; // Direction of rotation for right wheel
float rt_PWM = 0;

Encoder right_enc = Encoder(&right_ticks, RIGHT_PIN, TICKS_PER_METER, &incRightTicks);
PIDT<float> rightPID(&rt_vel, &rt_PWM, &rt_vel_tar, KpR, KiR, KdR, DIRECT);
  
	// Left motor
volatile unsigned long left_ticks = 0; // tick count for left wheel

float lt_vel = 0;
float lt_vel_tar = 0.5;

uint8_t lt_dir = FORWARD; // Direction of rotation left wheel
float lt_PWM = 0;

Encoder left_enc = Encoder(&left_ticks, LEFT_PIN, TICKS_PER_METER, &incLeftTicks);
PIDT<float> leftPID(&lt_vel, &lt_PWM, &lt_vel_tar, KpL, KiL, KdL, DIRECT);

void print_fl(float fl)
{
  char print_str[5];
  dtostrf(fl, 9, 3, print_str);
  nh.loginfo(print_str);
}

void vel_control_cb(const geometry_msgs::Twist& cmd_vel) {}

// increments ticks for left motor
void incLeftTicks() {
  left_ticks++;
}

// increments ticks for right motor
void incRightTicks() { 
  right_ticks++;
}

ros::Publisher pub_maze_robot_state("maze_robot_state", &robot_state); // publishes wheel angular velocities
ros::Subscriber<geometry_msgs::Twist> vel_control("cmd_vel", vel_control_cb);

void setup() {
  nh.initNode();
  nh.advertise(pub_maze_robot_state);
 	
	left_enc.init();
	right_enc.init();

  leftPID.SetMode(AUTOMATIC);              
  leftPID.SetOutputLimits(0, 1);
  leftPID.SetOutputMapping(0, 255);
 	
	rightPID.SetMode(AUTOMATIC);
	rightPID.SetOutputLimits(0, 1);
	rightPID.SetOutputMapping(0, 255);	

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

}

void setMotorVel(double vel, double PWM, Adafruit_DCMotor *motor) {
	if(vel > 0)
  {
    motor->run(FORWARD);
  }
  else if(vel < 0)
  { 
    motor->run(BACKWARD);
  }
  else{
    motor->run(RELEASE);
  }

	motor->setSpeed(PWM);
}

void loop() {

	left_enc.run();
	right_enc.run();

	// Set target velocities
	
	rightPID.Compute();
	leftPID.Compute();
	
	nh.loginfo("Right PWM:");
	print_fl(rt_PWM);

	nh.loginfo("Left PWM:");
	print_fl(lt_PWM);
	
	setMotorVel(rt_vel_tar, rt_PWM, rightMotor);
	setMotorVel(lt_vel_tar, lt_PWM, leftMotor);
	
	lt_vel = left_enc.getVelocity();
	rt_vel = right_enc.getVelocity();

	robot_state.left_ang_vel = (double) lt_vel;
  robot_state.right_ang_vel = (double) rt_vel;
 
  pub_maze_robot_state.publish(&robot_state);  
  nh.spinOnce();
  
  delay(3);
}
