#!/usr/bin/env python3

''' Acts as a serial bridge to the RPi Pico
'''
# ros imports
import rospy
from rospy_message_converter import json_message_converter
# python imports
import serial
import rospy_message_converter

import std_msgs.msg as std
import nmea_msgs.msg as nmea
import rover_msg.msg as rov

class PicoBridge():

    def __init__():
        pass

    def run():
        pass


def str_to_sentence(input: str) -> nmea.Sentence:

    ''' Converts a given input string to an nmea_msgs/Sentence message
        NOTE: doesn't verify that given input string is a valid NMEA sentence
    '''

    # get rid of any newlines
    input = input.strip()

    # create nmea_msgs/Sentence message
    sentence = nmea.Sentence()

    # populate the header
    sentence.header = std.Header()
    sentence.header.stamp = rospy.Time.now()
    sentence.header.frame_id = 'gps_link'   # the physical location of the gps on the robot, relative to origin

    # populate the string with given (stripped) input string
    sentence.sentence = input
    
    return sentence

def loopback_cmd_cb(command: rov.Cmd, ser: serial.Serial) -> None:

    ''' send received CMD messages down the pipe to the Pico
    '''

    # convert ROS message to JSON string
    cmd_json = json_message_converter.convert_ros_message_to_json(command)
    # get rid of all whitespaces in JSON string
    cmd_json = cmd_json.replace(" ", "")
    # build the CMD string
    cmd_string = "$CMD " + cmd_json + "\n"
    encoded = cmd_string.encode('utf-8')
    ser.write(encoded)

def main():

    rospy.init_node('pico_bridge', anonymous=True)

    # publishers for data streams FROM the pico
    nmea_pub = rospy.Publisher('nmea_sentence', nmea.Sentence, queue_size=1)
    cmd_pub = rospy.Publisher('/cmd', rov.Cmd, queue_size=1)

    ser = serial.Serial(
        port='/dev/ttyACM0',
        baudrate=115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS
    )

    cmd_sub = rospy.Subscriber('/loopback_cmd', rov.Cmd, callback=loopback_cmd_cb, callback_args=(ser))

    if not ser.is_open:
        rospy.logerr("Couldn't open serial port")
        return

    while not rospy.is_shutdown():

        # read input lines, decode bytes -> string
        input = ser.readline().decode()

        # 'tokenize' by spaces
        input_split = input.split(" ")

        # identify NMEA strings, publish them
        if input_split[0][0:4] == '$GPS':
            nmea_sentence = str_to_sentence(input_split[1])
            nmea_pub.publish(nmea_sentence)
        elif input_split[0][0:4] == '$CMD':
            cmd = json_message_converter.convert_json_to_ros_message('rover_msg/Cmd', input_split[1])
            cmd_pub.publish(cmd)
        else:
            print(input)


if __name__=='__main__':
    main()

