#! /usr/local/bin/python

import RPi.GPIO as GPIO, time

GPIO.setmode(GPIO.BCM)

def RC_count(pin) :
	count = 0

	GPIO.setup(pin, GPIO.OUT)
	GPIO.output(pin, GPIO.LOW)
	time.sleep(0.1)

	GPIO.setup(pin, GPIO.IN)
	while GPIO.input(pin) == GPIO.LOW :
		count += 1

	return count

while True :
	print RC_count(17)
