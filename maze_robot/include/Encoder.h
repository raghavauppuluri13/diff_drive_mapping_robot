#ifndef Encoder_h
#define Encoder_h

#define CALC_TIME 50

class Encoder {
  public:
		Encoder(volatile unsigned long*, int, void (*adv_ticks)());
		float getVelocity(); // In m/s
		void run();
		unsigned long getTickCount();
	
	private:	  
		volatile unsigned long *_ticks;
		void (*_adv_ticks)();
		unsigned long _prev_ticks = 0;
		float _last_time; 
		unsigned long _ticks_per_meter;
		float _velocity;			
};

#endif
