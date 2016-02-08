import RPi.GPIO as GPIO
import time

TRIG = 18
ECHO = 16

GPIO.setmode(GPIO.BOARD)

GPIO.setup(TRIG, GPIO.OUT)
GPIO.output(TRIG, 0)
GPIO.setup(ECHO, GPIO.IN)


def measureDistance():
	constant = 17000
	GPIO.output(TRIG, 1)
	time.sleep(0.00001)
	GPIO.output(TRIG, 0)
	while GPIO.input(ECHO) == 0:
		pass
	start = time.time()
	while GPIO.input(ECHO) == 1:
		pass
	stop = time.time()
	return ((stop - start) * constant)

#this takes three different outputs and averages them; this makes it more
#accurate

def measureAverage():
	dist1 = measureDistance()
	time.sleep(0.1)
	dist2 = measureDistance()
	time.sleep(0.1)
	dist3 = measureDistance()
	totalDistance = (dist1 + dist2 + dist3) / 3
	return totalDistance

while True:
	print(measureAverage())