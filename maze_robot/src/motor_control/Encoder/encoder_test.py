import time, sys
import RPi.GPIO as GPIO

Encode = int(sys.argv[1])
GPIO.setmode(GPIO.BCM)

ticks = 0;

def inc(channel):
	global ticks
	ticks = ticks + 1

GPIO.setup(Encode, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(Encode, GPIO.BOTH,callback=inc)
Running = 1

while Running:
	print(ticks)
GPIO.cleanup()
