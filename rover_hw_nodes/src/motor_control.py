#!/usr/bin/env python3
import math
from typing import Tuple

import rospy
import geometry_msgs.msg as geom
import nav_msgs.msg as nav
import std_msgs.msg as std
import rover_msg.msg as rov

#################################################
############### CONSTANTS #######################
#################################################

PWM_INCREMENT = 1
# number of encoder ticks per revolution of the wheels
TICKS_PER_REV = 48

# Wheel radius (meters)
WHEEL_RADIUS = .06 
# Distance from center of left tire to center of right tire (meters)
WHEEL_BASE = .190
METERS_PER_REV = WHEEL_RADIUS * math.pi * 2
REVS_PER_METER = 1 / METERS_PER_REV
# could also be measured manually
TICKS_PER_METER = TICKS_PER_REV * REVS_PER_METER

# Proportional constant of PWM-Linear Velocity relationship
# measured, not derived
# K_P = 530   # TODO
K_P = 183.33

# Y-intercept for the PWM-Linear Velocity relationship for the robot
b = 8 # TODO
 
# Correction multiplier for drift. Chosen through experimentation.
DRIFT_MULTIPLIER = 0 # TODO
 
# Turning PWM output (0 = min, 100 = max for PWM values)
PWM_TURN = 30

# Set maximum and minimum limits for the PWM values
PWM_MIN_LEFT = 20
PWM_MAX_LEFT = 50      
PWM_MIN_RIGHT = 20      # TODO: the minimum value required to make the rover move
PWM_MAX_RIGHT = 50

RPM = 250
MAX_LINEAR_X = METERS_PER_REV / (RPM / 60)

FORWARD = 0
REVERSE = 1
##################################################
############## END OF CONSTANTS ##################
##################################################

class encoderState():
    def __init__(self):
        self.prevTime = rospy.Time.now()
        self.prevCount = 0
        self.velocity = 0

def tick_cb(tick_msg: std.Int64, enc: encoderState) -> None:

    """
    When we receive ticks, use the encoderState provided to calculate velocity.
    """

    # get ticks since last callback
    ticks = tick_msg.data - enc.prevCount
    # get speed from ticks
    enc.velocity = (ticks / TICKS_PER_METER) / ((rospy.Time.now() - enc.prevTime).to_sec())
    # rospy.logdebug('vel: {}'.format(enc.velocity))
    # update timestamp and prevCount
    enc.prevTime = rospy.Time.now()
    enc.prevCount = tick_msg.data

pwm_update_time = None
motor_msg = rov.Motors()

prevDiff = 0
prevPrevDiff = 0
ready_flag = False

def calc_pwm_values(cmd_vel: geom.Twist, encoders: Tuple) -> None:

    """
    When we receive a Twist message on motor command topic, calculate and update pwm values to be published in main()
    """

    # use (hideous) globals
    global motor_msg
    global pwm_update_time
    global prevDiff
    global prevPrevDiff
    global ready_flag

    ready_flag = False

    # unpack encoders
    enc1 = encoders[0]
    enc2 = encoders[1]

    # get current stamp
    pwm_update_time = rospy.Time.now()

    # don't go faster than we can go
    if cmd_vel.linear.x > MAX_LINEAR_X:
        linear_x = MAX_LINEAR_X
    elif cmd_vel.linear.x < -MAX_LINEAR_X:
        linear_x = -MAX_LINEAR_X
    else:
        linear_x = cmd_vel.linear.x

    # set speeds
    pwmLeftReq = math.ceil(K_P * abs(linear_x) + b)
    pwmRightReq = math.ceil(K_P * abs(linear_x) + b)

    # checks if we need to turn
    if cmd_vel.angular.z != 0.0:
        # if yes, get direction
        if cmd_vel.angular.z < 0: # turn left
            pwmLeftReq = PWM_TURN
            left_dir = REVERSE    # back
            pwmRightReq = PWM_TURN
            right_dir = FORWARD   # forward
        else:                     # turn right
            pwmLeftReq = PWM_TURN
            left_dir = FORWARD    # forward
            pwmRightReq = PWM_TURN
            right_dir = REVERSE   # back
    # if we don't need to turn, just go straight
    else:
        # average out differences in wheel velocities
        diff = enc1.velocity - enc2.velocity
        
        avgDiff = (diff + prevDiff + prevPrevDiff) / 3  
        # print("avgDiff: {}".format(avgDiff))
        prevPrevDiff = prevDiff
        prevDiff = diff

        # correct PWM values to make vehicle go (mostly) straight
        pwmLeftReq -= math.ceil(avgDiff * DRIFT_MULTIPLIER)
        pwmRightReq += math.ceil(avgDiff * DRIFT_MULTIPLIER)

        # set directions
        left_dir = FORWARD if (linear_x > 0) else REVERSE
        right_dir = left_dir

    # handle (too) low values
    if abs(pwmLeftReq) < PWM_MIN_LEFT:
        pwmLeftReq = 0
    if abs(pwmRightReq) < PWM_MIN_RIGHT:
        pwmRightReq = 0
        
    # handle values above the max
    pwmLeftReq = min(PWM_MAX_LEFT, pwmLeftReq)
    pwmRightReq = min(PWM_MAX_RIGHT, pwmRightReq)

    # update the Motors message
    motor_msg.dir2.data = left_dir
    motor_msg.pwm2.data = pwmLeftReq
    motor_msg.dir1.data = right_dir
    motor_msg.pwm1.data = pwmRightReq

    # print("LEFT: {}\t RIGHT: {}".format(pwmLeftReq, pwmRightReq))
    # print("RIGHT: {}\n".format(pwmRightReq))

    ready_flag = True

def main():

    global motor_msg
    global pwm_update_time
    global ready_flag

    rospy.init_node('motor_control', anonymous=True, log_level=rospy.DEBUG)

    motor_pub = rospy.Publisher('/motors', rov.Motors, queue_size=1)

    # listen for ticks (for wheel velocities)
    enc1 = encoderState()
    rospy.Subscriber('/left_ticks', std.Int64, callback=tick_cb, callback_args=(enc1))
    enc2 = encoderState()
    rospy.Subscriber('/right_ticks', std.Int64, callback=tick_cb, callback_args=(enc2))
    # listen for motor commands
    rospy.Subscriber('/cmd_vel', geom.Twist, callback=calc_pwm_values, callback_args=(enc1, enc2))

    # initialize pwm_update_time
    pwm_update_time = rospy.Time.now()

    clear_flag = False

    rate = rospy.Rate(10)   # loop rate: 10Hz
    while not rospy.is_shutdown():

        # if we haven't updated pwm in a while, stop the motors
        if (rospy.Time.now() - pwm_update_time) > rospy.Duration(.5):
            if not clear_flag:
                rospy.logdebug("Reached PWM timeout; clearing PWM")
            clear_flag = True
            motor_msg.pwm1.data = 0
            motor_msg.pwm2.data = 0
        else:
            clear_flag = False

        if ready_flag:

            # last ditch effort to stop out-of-bounds values
            if not 0 <= motor_msg.pwm1.data <= 100:
                continue
            elif not 0 <= motor_msg.pwm2.data <= 100:
                continue

            try:
                motor_pub.publish(motor_msg)
            except Exception as e:
                rospy.logdebug("Threw exception, not publishing motor message.")
            ready_flag = False

        rate.sleep()

if __name__=='__main__':
    main()