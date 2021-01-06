#ifndef Encoder_h
#define Encoder_h

class Encoder {
  public:
		Encoder(volatile unsigned long*, int, int, void (*adv_ticks)());
		void init();
		
		double getVelocity(); // In m/s
		void run();
		unsigned long getTickCount();
		void setCalcTime(double);
	
	private:	  
		int _encoder_pin;
		volatile unsigned long *_ticks;
		void (*_adv_ticks)();
		unsigned long _prev_ticks = 0;
		double _last_time; 
		unsigned long _ticks_per_meter;
		double _calc_time;
		double _velocity;
			
};

#endif
