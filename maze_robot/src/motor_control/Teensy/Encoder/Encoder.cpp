#if ARDUINO >= 100
	#include "Arduino.h"
#endif

#include <Encoder.h>

Encoder::Encoder(volatile unsigned long *ticks, int encoder_pin, int ticks_per_meter, void (*adv_ticks)()) {
	_encoder_pin = encoder_pin;
	_ticks = ticks;
	_adv_ticks = adv_ticks;
	
	_ticks_per_meter = ticks_per_meter;
	_last_time = millis() - CALC_TIME;
}

void Encoder::init() {
	pinMode(_encoder_pin, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(_encoder_pin), _adv_ticks, CHANGE);
}

void Encoder::run() {
	float now = millis();
	float time_diff = now - _last_time;
	int ticks_change;
  if(time_diff >= CALC_TIME) {
		ticks_change = *_ticks - _prev_ticks;
		_velocity = ((float) ticks_change / _ticks_per_meter) / (time_diff / 1000.0);
		_last_time = now;
		_prev_ticks = *_ticks;
	}
}
float Encoder::getVelocity() {
	return _velocity;
}

