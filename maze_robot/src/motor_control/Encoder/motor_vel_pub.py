import rospy
from maze_robot.msg import MazeRobotState

import RPi.GPIO as GPIO
import time

ticks_rt = 0
ticks_lt = 0

class Encoder:
	'Encoder Class'

	CALC_TIME = 20.0 #millis

	def __init__(self,is_right,pin, ticks_per_meter,cb):
		self.ticks_per_meter = ticks_per_meter
		self.is_right = is_right
		GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
		GPIO.add_event_detect(pin, GPIO.BOTH, callback=cb)
		self.last_time = time.time_ns() / (10 ** 3) - self.CALC_TIME	
		self.velocity = 0
		self.prev_ticks = 0
	def get_ticks(self):
		return ticks_rt if self.is_right else ticks_lt
	
	def run(self):
		now = time.time_ns() / (10 ** 3) # millis
		time_diff = now - self.last_time
		if time_diff >= self.CALC_TIME:
			ticks_change = self.get_ticks() - self.prev_ticks
			print(ticks_change)
			self.velocity = (float(ticks_change) / self.ticks_per_meter) / (time_diff / 1000.0);
			self.last_time = now;
			self.prev_ticks = self.get_ticks();

RIGHT_PIN = 2
LEFT_PIN = 3
TICKS_PER_METER = 263
GPIO.setmode(GPIO.BCM)

def inc_tick_rt(channel):
	global ticks_rt
	ticks_rt = ticks_rt + 1

def inc_tick_lt(channel):
  global ticks_lt
  ticks_lt = ticks_lt + 1

left = Encoder(False, LEFT_PIN, TICKS_PER_METER, inc_tick_lt)
right = Encoder(True, RIGHT_PIN, TICKS_PER_METER, inc_tick_rt)

while(1):
	left.run()
	right.run()
	
	print(left.velocity)
	print(right.velocity)
GPIO.cleanup()
