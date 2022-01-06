#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time, signal

left_output = 33
right_output = 32

GPIO.setmode(GPIO.BOARD)

# set DIR pins to OUT
GPIO.setup(left_output, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(right_output, GPIO.OUT, initial=GPIO.LOW)

# set PWM pins to logic HIGH
GPIO.setup(35, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(36, GPIO.OUT, initial=GPIO.LOW)
GPIO.output(35, GPIO.HIGH)
GPIO.output(36, GPIO.HIGH)

p_l = GPIO.PWM(left_output, 10000)
p_r = GPIO.PWM(right_output, 10000)

def handler(signum, frame):
	
	p_l.stop()
	p_r.stop()
	GPIO.cleanup()

def main():
	
	signal.signal(signal.SIGINT, handler)
	
	p_l.start(50)
	p_r.start(50)

	p_l.ChangeDutyCycle(75)
	p_r.ChangeDutyCycle(75)

	while True:
		print(".")
		# time.sleep(.25)

	
if __name__=='__main__':
	main()
