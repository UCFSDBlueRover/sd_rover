#!/usr/bin/env python3

''' given a Twist message, controls the motors of the rover '''

# system imports
import RPi.GPIO as GPIO
import time, signal
from enum import Enum

# ros imports
import rospy

# ROS messages
from geometry_msgs.msg import Twist

# class Pins(Enum):
# 	LEFT_OUT = 33
# 	RIGHT_OUT = 32

class BaseControllerNode:

	def __init__(self, hw=True, topic="/cmd_vel", cb=self.twist_cb):
		
		twist_sub = rospy.Subscriber(topic, Twist, callback=cb)
		
		self._hw = hw
		# configure for jetson Nano hardware
		if self._hw:
			
			GPIO.setmode(GPIO.BOARD)
			
			self.PIN_L_OUT = 33
			self.PIN_R_OUT = 32
			
			# set DIR pins to OUT
			GPIO.setup(self.PIN_L_OUT, GPIO.OUT, initial=GPIO.LOW)
			GPIO.setup(self.PIN_R_OUT, GPIO.OUT, initial=GPIO.LOW)
				
			# TODO: either get a hardware logic level shifter, or get rid of this block entirely 
			# set PWM pins to logic HIGH
			# GPIO.setup(35, GPIO.OUT, initial=GPIO.LOW)
			# GPIO.setup(36, GPIO.OUT, initial=GPIO.LOW)
			# GPIO.output(35, GPIO.HIGH)
			# GPIO.output(36, GPIO.HIGH)
				
			# configure PWM
			self.PWM_L = GPIO.PWM(self.PIN_L_OUT, 10000)
			self.PWM_R = GPIO.PWM(self.PIN_R_OUT, 10000)

			# start PWM for motors at stop (50)
			self.PWM_L.start(50)
			self.PWM_R.start(50)
	
	def run(self):
	
		rospy.spin()
						
	def twist_cb(self, data):
		print(data)
		pass

	def ctrl_c_handler(self, signum, frame):
		
		self.PWM_L.stop()
		self.PWM_R.stop()
		GPIO.cleanup()
		
		
def main():
	
	rospy.init_node('base_controller', anonymous=True)
	
	bc = BaseControllerNode(hw=True)
	
	signal.signal(signal.SIGINT, bc.ctrl_c_handler)
	
	bc.run()
	
	# p_l.ChangeDutyCycle(75)
	# p_r.ChangeDutyCycle(75)
	
if __name__=='__main__':
	main()
