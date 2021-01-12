#include <ros.h>

#include <maze_robot/MazeRobotState.h>
#include <maze_robot/PWM.h>

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
//maze_robot::PWM pwm;
maze_robot::MazeRobotState robot_state;
ros::NodeHandle nh;

// GLOBALS

  // Right motor
volatile unsigned long right_ticks = 0; // tick count for right wheel

double rt_input_vel = 0;
 
double rt_output = 0;

double rt_vel_tar = 0.5; 
double rt_PWM = 0;

Encoder right_enc = Encoder(&right_ticks, RIGHT_PIN, TICKS_PER_METER, &incRightTicks);
//PID rightPID(&rt_input_vel, &rt_output, &rt_vel_tar, KpR, KiR, KdR, DIRECT);
  
	// Left motor
volatile unsigned long left_ticks = 0; // tick count for left wheel

double lt_input_vel = 0;

double lt_output = 0;

double lt_vel_tar = 0.5;

//double lt_PWM = 0;

Encoder left_enc = Encoder(&left_ticks, LEFT_PIN, TICKS_PER_METER, &incLeftTicks);

//PID leftPID(&lt_input_vel, &lt_output, &lt_vel_tar, KpL, KiL, KdL, DIRECT);

void print_fl(float fl)
{
  char print_str[5];
  dtostrf(fl, 9, 3, print_str);
  nh.loginfo(print_str);
}

// increments ticks for left motor
void incLeftTicks() {
  left_ticks++;
}

// increments ticks for right motor
void incRightTicks() { 
  right_ticks++;
}

ros::Publisher pub_maze_robot_state("maze_robot_state", &robot_state); // publishes wheel angular velocities
//ros::Publisher pub_pwm("pwm", &pwm); // publishes wheel pwm

/*
int map_to_pwm(double vel){
	return (int) map(vel, 0, 1, 0, 255);
}
*/

void setup() {
  nh.initNode();
  nh.advertise(pub_maze_robot_state);
 	nh.advertise(pub_pwm);

  left_enc.init();
  right_enc.init();
/*
  leftPID.SetMode(AUTOMATIC);              
  leftPID.SetOutputLimits(0, 225);
 	
	rightPID.SetMode(AUTOMATIC);
	rightPID.SetOutputLimits(0, 225);
*/
}

void loop() {

	left_enc.run();
	right_enc.run();

	// Set target velocities
	
	//rightPID.Compute();
	//leftPID.Compute();

  print_fl(rt_output);
	
	//pwm.right_motor = map_to_pwm(rt_output);
	
	//pwm.left_motor = map_to_pwm(lt_output);
	
	lt_input_vel = left_enc.getVelocity();
	rt_input_vel = right_enc.getVelocity();

	robot_state.left_ang_vel = lt_input_vel;
  robot_state.right_ang_vel = rt_input_vel;
 
  pub_maze_robot_state.publish(&robot_state);  
  nh.spinOnce();
  
  delay(3);
}
