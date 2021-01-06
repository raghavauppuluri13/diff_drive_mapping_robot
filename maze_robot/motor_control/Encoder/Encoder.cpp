#if ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include <Encoder.h>

Encoder::Encoder(volatile unsigned long *ticks, int encoder_pin, int ticks_per_meter, void (*adv_ticks)()) {
	_encoder_pin = encoder_pin;
	_ticks = ticks;
	_adv_ticks = adv_ticks;
	
	_ticks_per_meter = ticks_per_meter;
	_calc_time = 100.0;
	_last_time = millis() - _calc_time;
}

void Encoder::init() {
	pinMode(_encoder_pin, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(_encoder_pin), _adv_ticks, CHANGE);
}

void Encoder::run() {
	double now = millis();
	double time_diff = now - _last_time;
	int ticks_change;
  if(time_diff >= _calc_time) {
		ticks_change = *_ticks - _prev_ticks;
		_velocity = ((double) ticks_change / _ticks_per_meter) / (time_diff / 1000.0);
		_last_time = now;
		_prev_ticks = *_ticks;
	}
}
double Encoder::getVelocity() {
	return _velocity;
}

void Encoder::setCalcTime(double calc_time) {
	if(calc_time > 0) {
		_calc_time = (double) calc_time;
	}
}

