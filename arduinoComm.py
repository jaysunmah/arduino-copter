
#uses the RPi.GPIO pin library
#more information on the library can be found here:
#https://pypi.python.org/pypi/RPi.GPIO
import RPi.GPIO as GPIO 
import time


#uses the Raspberry Pi's pin#8 (physical numbering)
LED = 8

GPIO.setmode(GPIO.BOARD)

GPIO.setup(LED, GPIO.OUT)

#configures the LED to be read as a PWM signal
led = GPIO.PWM(LED, 100)
led.start(1)

time.sleep(0.1)


pwm = 0
increasing= True

#continous loop that will raise and lower the PWM
#duty cycle (between 0% to 100%)
while True:
	while pwm < 100:
		led.ChangeDutyCycle(pwm)
		pwm += 1
		time.sleep(0.05)
	
	while pwm > 0:
		led.ChangeDutyCycle(pwm)
		pwm -= 1
		time.sleep(0.05)


