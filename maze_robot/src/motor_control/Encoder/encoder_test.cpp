#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <iostream>
using namespace std;

//#include "Encoder.h"

#define LEFT_PIN 2
#define RIGHT_PIN 3
#define TICKS_PER_METER 263

void incLeft(void);
void incRight(void);

static volatile unsigned long left_ticks = 0;
static volatile unsigned long right_ticks = 0;

int main(void) 
{

//	Encoder left_enc = Encoder(&left_ticks, TICKS_PER_METER, &incLeft);

//	Encoder right_enc = Encoder(&right_ticks, TICKS_PER_METER, &incRight);
	
	
	if(wiringPiSetup() < 0){
		cout << "Unable to initialize wiringPi";
		return 1;
	}		

	pinMode(RIGHT_PIN, INPUT);
	pinMode(LEFT_PIN, INPUT);
	
	pullUpDnControl(RIGHT_PIN, PUD_UP);
	pullUpDnControl(LEFT_PIN, PUD_UP);	
	
	if(wiringPiISR(RIGHT_PIN, INT_EDGE_BOTH, incRight) < 0){
		cout << "Unable to init ISR\n";
	}

	if(wiringPiISR(LEFT_PIN, INT_EDGE_BOTH, incLeft) < 0){
    cout << "Unable to init ISR\n";
  }

	cout << "Hello";
	
	while(1){
		cout << "Right: \n" << analogRead(RIGHT_PIN);
		cout << "Left: \n" << analogRead(LEFT_PIN);
	}

}

void incLeft(void) {
	left_ticks++;
}

void incRight(void) {
	right_ticks++;
}

