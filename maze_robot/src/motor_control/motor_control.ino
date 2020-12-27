/* 
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control
For use with the Adafruit Motor Shield v2 
---->	http://www.adafruit.com/products/1438
*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *leftMotor = AFMS.getMotor(4);

Adafruit_DCMotor *rightMotor = AFMS.getMotor(1);

int left_enc = A0;
int right_enc = A1;
int val = 0;

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Encoder Test!");
}

void loop() {
  val = analogRead(left_enc);
  Serial.println(val); 
}
