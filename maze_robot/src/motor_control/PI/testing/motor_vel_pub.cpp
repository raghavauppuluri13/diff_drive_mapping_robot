#include <ros/ros.h>
#include <ros/console.h>

#include <maze_robot/MazeRobotState.h>
#include <wiringPi.h>

//#include "Encoder.h"

#define LEFT_PIN 2
#define RIGHT_PIN 3
#define TICKS_PER_METER 263

void incLeft(void);
void incRight(void);

static volatile unsigned long left_ticks = 0;
static volatile unsigned long right_ticks = 0;

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "motor_vel_pub");
	
	ros::NodeHandle n;

//	Encoder left_enc = Encoder(&left_ticks, TICKS_PER_METER, &incLeft);

//	Encoder right_enc = Encoder(&right_ticks, TICKS_PER_METER, &incRight);
	
	maze_robot::MazeRobotState state;
	
	if(wiringPiSetup() < 0){
		ROS_ERROR("Unable to initialize wiringPi");
		return 1;
	}		

	pinMode(RIGHT_PIN, INPUT);
	pinMode(LEFT_PIN, INPUT);
	
	pullUpDnControl(RIGHT_PIN, PUD_UP);
	pullUpDnControl(LEFT_PIN, PUD_UP);	
	
	if(wiringPiISR(RIGHT_PIN, INT_EDGE_BOTH, incRight) < 0){
		ROS_ERROR("Unable to init ISR\n");
	}

	if(wiringPiISR(LEFT_PIN, INT_EDGE_BOTH, incLeft) < 0){
    ROS_ERROR("Unable to init ISR\n");
  }

	ROS_ERROR("Here\n");	
	
	ros::Publisher state_pub = n.advertise<maze_robot::MazeRobotState>("maze_robot_state", 1000);
	ros::Rate loop_rate(30);

	state.left_ang_vel = 10.0;
	state.right_ang_vel = 10.0;
	
	while(ros::ok()){
		ros::spinOnce();

		ROS_ERROR_STREAM("Right: "<< analogRead(RIGHT_PIN));
		ROS_ERROR_STREAM("Left: "<< analogRead(LEFT_PIN));

		loop_rate.sleep();
	}

}

void incLeft(void) {
	ROS_ERROR("PRINT LEFT");
	left_ticks++;
}

void incRight(void) {
	ROS_ERROR("PRINT RIGHT");
	right_ticks++;
}
