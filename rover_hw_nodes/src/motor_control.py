#!/usr/bin/env python3
from distutils.debug import DEBUG
import math

import rospy
import geometry_msgs.msg as geom
import nav_msgs.msg as nav
import std_msgs.msg as std

PWM_INCREMENT = 1
TICKS_PER_REV = 48

# Wheel radius (meters)
WHEEL_RADIUS = .06 

# Distance from center of left tire to center of right tire (meters)
WHEEL_BASE = .205

METERS_PER_REV = WHEEL_RADIUS * math.pi * 2
REVS_PER_METER = 1 / METERS_PER_REV
# could also be measured manually
TICKS_PER_METER = TICKS_PER_REV * REVS_PER_METER

# Proportional constant of PWM-Linear Velocity relationship
# measured, not derived
K_P = 10   # TODO

# Y-intercept for the PWM-Linear Velocity relationship for the robot
b = 0
 
# Correction multiplier for drift. Chosen through experimentation.
DRIFT_MULTIPLIER = 120
 
# Turning PWM output (0 = min, 255 = max for PWM values)
PWM_TURN = 60

# Set maximum and minimum limits for the PWM values
PWM_MIN = 30     # TODO
PWM_MAX = 100   # TODO

RPM = 120
MAX_LINEAR_X = METERS_PER_REV / (RPM / 60)
#######################################################################

pwmLeftReq = 0
pwmRightReq = 0
pwm_update_time = None
odom = None

def calc_pwm_values(cmd_vel: geom.Twist) -> None:
   
    global pwmLeftReq
    global pwmRightReq
    global pwm_update_time

    pwm_update_time = rospy.Time.now()

    # don't go faster than we can go
    linear_x = min(cmd_vel.linear.x, MAX_LINEAR_X)

    pwmLeftReq = K_P * cmd_vel.linear.x + b
    pwmRightReq = K_P * cmd_vel.linear.x + b

    # checks if we need to turn
    if cmd_vel.angular.z != 0.0:
        # if yes, get direction
        if cmd_vel.angular.z > 0: # turn left
            pwmLeftReq = -PWM_TURN
            pwmRightReq = PWM_TURN
        else:                     # turn right
            pwmLeftReq = PWM_TURN
            pwmRightReq = -PWM_TURN
    
    # handle (too) low values
    if abs(pwmLeftReq) < PWM_MIN:
        pwmLeftReq = 0
    if abs(pwmRightReq) < PWM_MIN:
        pwmRightReq = 0
        
    # handle values above the max
    pwmLeftReq = min(PWM_MAX, int(pwmLeftReq))
    pwmRightReq = min(PWM_MAX, int(pwmRightReq))

    print("LEFT: {}\n".format(pwmLeftReq))
    print("RIGHT: {}\n".format(pwmRightReq))
    
def odom_callback(odom_msg: nav.Odometry) -> None:

    odom = odom_msg

def main():

    global pwm_update_time, pwmLeftReq, pwmRightReq

    rospy.init_node('motor_control', anonymous=True, log_level=rospy.DEBUG)

    rospy.Subscriber('/cmd_vel', geom.Twist, callback=calc_pwm_values)
    pwm_update_time = rospy.Time.now()

    left_pub = rospy.Publisher('/left_pwm', std.Int8, queue_size=1)
    right_pub = rospy.Publisher('/right_pwm', std.Int8, queue_size=1)

    rate = rospy.Rate(10)   # loop rate: 10Hz
    while not rospy.is_shutdown():

        if (rospy.Time.now() - pwm_update_time) > rospy.Duration(2.5):
            pwmLeftReq = 0
            pwmRightReq = 0

        msg = std.Int8()
        msg.data = pwmLeftReq
        left_pub.publish(msg)

        msg.data = pwmRightReq
        right_pub.publish(msg)

        rate.sleep()

if __name__=='__main__':
    main()