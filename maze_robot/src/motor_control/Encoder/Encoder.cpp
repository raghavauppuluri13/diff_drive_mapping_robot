#include <Encoder.h>

Encoder::Encoder(volatile unsigned long *ticks, int ticks_per_meter, void (*adv_ticks)()) {
	_ticks = ticks;
	_adv_ticks = adv_ticks;
	
	_ticks_per_meter = ticks_per_meter;
	_last_time = millis() - CALC_TIME;
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

