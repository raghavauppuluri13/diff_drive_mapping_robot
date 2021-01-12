#include <ros.h>

#include <maze_robot/MazeRobotState.h>

#include <Encoder.h>

#define LEFT_PIN 2
#define RIGHT_PIN 3
#define MAX_PWM 125

#define TICKS_PER_METER 263

void incLeftTicks();
void incRightTicks();
void print_fl(float);

maze_robot::MazeRobotState robot_state;
ros::NodeHandle nh;

// GLOBALS

  // Right motor
volatile unsigned long right_ticks = 0; // tick count for right wheel

double rt_vel = 0;

Encoder right_enc = Encoder(&right_ticks, RIGHT_PIN, TICKS_PER_METER, &incRightTicks);
  
	// Left motor
volatile unsigned long left_ticks = 0; // tick count for left wheel

double lt_vel = 0;

Encoder left_enc = Encoder(&left_ticks, LEFT_PIN, TICKS_PER_METER, &incLeftTicks);

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

void setup() {
  nh.initNode();
  nh.advertise(pub_maze_robot_state);
  
  left_enc.init();
  right_enc.init();
}

void loop() {

	left_enc.run();
	right_enc.run();

	// Set target velocities
	
	lt_vel = left_enc.getVelocity();
	rt_vel = right_enc.getVelocity();

	robot_state.left_ang_vel = lt_vel;
  robot_state.right_ang_vel = rt_vel;
 
  pub_maze_robot_state.publish(&robot_state);  
  nh.spinOnce();
  
  delay(3);
}
