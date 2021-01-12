#include <Encoder.h>
#include <Arduino.h>

#define LEFT 2
#define RIGHT 3
#define TICKS_PER_METER 1000
volatile unsigned long left_ticks = 0;
volatile unsigned long right_ticks = 0; 

void incTicksLeft(){
	left_ticks++;
}
void incTicksRight(){
	right_ticks++;
}

Encoder left = Encoder(&left_ticks, LEFT, TICKS_PER_METER, &incTicksLeft);
Encoder right = Encoder(&right_ticks, RIGHT, TICKS_PER_METER, &incTicksRight);

void setup() {
	Serial.begin(9600);
	left.init();
	right.init();
}

void loop() {
	Serial.println(left_ticks);
	Serial.println(right_ticks);
	
}

