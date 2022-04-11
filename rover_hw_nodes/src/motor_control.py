#!/usr/bin/env python3
import math

import rospy
import geometry_msgs.msg as geom
import nav_msgs.msg as nav
import std_msgs.msg as std
import rover_msg.msg as rov

#################################################
############### CONSTANTS #######################
#################################################

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
K_P = 530   # TODO

# Y-intercept for the PWM-Linear Velocity relationship for the robot
b = 0 # TODO
 
# Correction multiplier for drift. Chosen through experimentation.
DRIFT_MULTIPLIER = 120
 
# Turning PWM output (0 = min, 100 = max for PWM values)
PWM_TURN = 60

# Set maximum and minimum limits for the PWM values
PWM_MIN_LEFT = 30
PWM_MAX_LEFT = 100      
PWM_MIN_RIGHT = 30      # TODO: the minimum value required to make the rover move
PWM_MAX_RIGHT = 100

RPM = 120
MAX_LINEAR_X = METERS_PER_REV / (RPM / 60)

############################################################
############## END OF CONSTANTS ############################
############################################################

pwm_update_time = None
motor_msg = rov.Motors()

def calc_pwm_values(cmd_vel: geom.Twist) -> None:

    global motor_msg
    global pwm_update_time

    pwm_update_time = rospy.Time.now()

    # don't go faster than we can go
    if cmd_vel.linear.x > MAX_LINEAR_X:
        linear_x = MAX_LINEAR_X
    elif cmd_vel.linear.x < -MAX_LINEAR_X:
        linear_x = -MAX_LINEAR_X
    else:
        linear_x = cmd_vel.linear.x

    # checks if we need to turn
    if cmd_vel.angular.z != 0.0:
        # if yes, get direction
        if cmd_vel.angular.z > 0: # turn left
            pwmLeftReq = PWM_TURN
            left_dir = 0    # back
            pwmRightReq = PWM_TURN
            right_dir = 1   # forward
        else:                     # turn right
            pwmLeftReq = PWM_TURN
            left_dir = 1    # forward
            pwmRightReq = PWM_TURN
            right_dir = 0   # back
    # if we don't need to turn, just go straight
    else:
        # set directions
        left_dir = 1 if (linear_x > 0) else 0
        right_dir = 1 if (linear_x > 0) else 0
        # set speeds
        pwmLeftReq = math.ceil(K_P * abs(linear_x) + b)
        pwmRightReq = math.ceil(K_P * abs(linear_x) + b)

    # handle (too) low values
    if abs(pwmLeftReq) < PWM_MIN_LEFT:
        pwmLeftReq = 0
    if abs(pwmRightReq) < PWM_MIN_RIGHT:
        pwmRightReq = 0
        
    # handle values above the max
    pwmLeftReq = min(PWM_MAX_LEFT, pwmLeftReq)
    pwmRightReq = min(PWM_MAX_RIGHT, pwmRightReq)

    # update the Motors message
    motor_msg.dir1.data = left_dir
    motor_msg.pwm1.data = pwmLeftReq
    motor_msg.dir2.data = right_dir
    motor_msg.pwm2.data = pwmRightReq

    print("LEFT: {}\n".format(pwmLeftReq))
    print("RIGHT: {}\n".format(pwmRightReq))

def main():

    global pwm_update_time

    rospy.init_node('motor_control', anonymous=True, log_level=rospy.DEBUG)

    motor_pub = rospy.Publisher('/motors', rov.Motors, queue_size=1)
    # listen for motor commands
    rospy.Subscriber('/cmd_vel', geom.Twist, callback=calc_pwm_values)

    # initialize pwm_update_time
    pwm_update_time = rospy.Time.now()

    clear_flag = False

    rate = rospy.Rate(10)   # loop rate: 10Hz
    while not rospy.is_shutdown():

        # if we haven't updated pwm in a while, stop the motors
        if (rospy.Time.now() - pwm_update_time) > rospy.Duration(2.5):
            if not clear_flag:
                rospy.logdebug("Reached PWM timeout; clearing PWM")
            clear_flag = True
            motor_msg.pwm1.data = 0
            motor_msg.pwm2.data = 0
        else:
            clear_flag = False

        motor_pub.publish(motor_msg)

        rate.sleep()

if __name__=='__main__':
    main()