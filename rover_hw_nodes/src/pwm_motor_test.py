#!/usr/bin/env python3

''' Acts as a serial bridge to the RPi Pico
'''
# ros imports
import rospy

from pynput import keyboard

import std_msgs.msg as std
import geometry_msgs.msg as geom
import nmea_msgs.msg as nmea
import rover_msg.msg as rov

pwm_pub = rospy.Publisher('/motors', rov.Motors, queue_size=1)

# default PWM message, modified by on_press and on_release
motors = rov.Motors()
motors.dir1.data = 1
motors.pwm1.data = 0
motors.dir2.data = 1
motors.pwm2.data = 0

updated = False
motor = "left"

def on_press(key):

    global updated, motor

    try:
        k = key.char
    except AttributeError as e:
        k = key.name

    # modify the pwm message
    if k == 'up':
        if motor == 'left':
            motors.pwm1.data = min(100, motors.pwm1.data + 1)
        elif motor == 'right':
            motors.pwm2.data = min(100, motors.pwm2.data + 1)
        updated = True  
    if k == 'down':
        if motor == 'left':
            motors.pwm1.data = max(0, motors.pwm1.data - 1)
        elif motor == 'right':
            motors.pwm2.data = max(0, motors.pwm2.data - 1)
        updated = True
    if k == 'left':
        motor = "left"
        updated = True
    if k == 'right':
        motor = "right"
        updated = True

    update()

def on_release(key):

    global updated

    try:
        k = key.char
    except AttributeError:
        k = key.name

    # if Escape is pressed, exit keyboard listener
    if key == keyboard.Key.esc:
        # Stop listener
        return False    

    # update published twiest message
    update()

def update():

    global updated

    if updated:
        pwm_pub.publish(motors)
        change_flag = False

def main():

    # TODO: add CTRL+C signal handler. 

    rospy.init_node('pwm_motor_test', anonymous=True, log_level=rospy.DEBUG) # node that will handle sending commands
    listener = keyboard.Listener()  # pynput keyboard listener for catching arrow key input

    # when key is pressed, run on_press(), when a key is released, run on_release()
    with keyboard.Listener(
        on_press=on_press,  
        on_release=on_release) as listener: 
        listener.join()

    rospy.spin()    # keeps this ros node from closing


if __name__=='__main__':
    main()
