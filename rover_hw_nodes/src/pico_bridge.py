#!/usr/bin/env python3

''' Acts as a serial bridge to the RPi Pico
'''
# ros imports
import rospy
from rospy_message_converter import json_message_converter

# python imports
import serial
from typing import Tuple

# message imports
import std_msgs.msg as std
import geometry_msgs.msg as geom
import nmea_msgs.msg as nmea
import rover_msg.msg as rov

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

def str_to_command(input: list) -> rov.Cmd:

    """
    Input format: X Y Z start cancel shutdown rc_preempt pose_preempt
    """

    cmd = rov.Cmd()

    cmd.target = geom.Point()
    try:
        cmd.target.x = float(input[0])
        cmd.target.y = float(input[1])
        cmd.target.z = float(input[2])
    except ValueError as e:
        rospy.logerr(e)
        return None

    # flags (TODO: need to be cast?)
    cmd.start.data          = (input[3] == 'True')
    cmd.cancel.data         = (input[4] == 'True')
    cmd.shutdown.data       = (input[5] == 'True')
    cmd.rc_preempt.data     = (input[6] == 'True')
    cmd.pose_preempt.data   = (input[7] == 'True')

    return cmd

def str_to_rc_command(input: list) -> rov.Cmd:

    """
    Converts "rc burst" style strings into Cmd messages
    """

    cmd = rov.Cmd()

    # rc fields
    cmd.rc.forward  = int(input[0])
    cmd.rc.reverse  = int(input[1])
    cmd.rc.left     = int(input[2])
    cmd.rc.right    = int(input[3])

    # "burst" messages always throw this flag to set state machine to MANUAL
    cmd.rc_preempt.data = True
    # all other flags are false
    cmd.start.data          = False
    cmd.cancel.data         = False
    cmd.shutdown.data       = False
    cmd.pose_preempt.data   = False
    
    return cmd    

def ticks_to_message(input: list) -> Tuple[std.Int64, std.Int64]:

    l_ticks = std.Int64()
    r_ticks = std.Int64()

    # ticks are reported strangely
    try:
        l_ticks.data = int(input[3])    # encoder 2 on PICO
        r_ticks.data = int(input[1])    # encoder 1 on PICO
    except ValueError as e:
        rospy.log_err('Invalid ticks data')
        return None

    return (l_ticks, r_ticks)

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

def tlm_cb(tlm: rov.Telemetry, ser: serial.Serial) -> None:

    ''' when we receive telemetry messages, pass them through to the pico to be transmitted '''

    tlm_dict = [tlm.state.data,
                "{:.2f}".format(tlm.pose.pose.position.x),
                "{:.2f}".format(tlm.pose.pose.position.y),
                "{:.2f}".format(tlm.pose.pose.position.z),
                "{:.6f}".format(tlm.fix.latitude),
                "{:.6f}".format(tlm.fix.longitude),
                "{:.3f}".format(tlm.fix.altitude)]

    # cast all items to str and join
    msg_string = ''.join((str(e) + " ") for e in tlm_dict)
    # create $MTR string and send
    tlm_string = "$TLM " + msg_string + '\n'

    rospy.logdebug(tlm_string)
    encoded = tlm_string.encode('utf-8')
    ser.write(encoded)

def pwm_cb(msg: rov.Motors, ser: serial.Serial) -> None:

    # unpack msg to dictionary
    pwm = [msg.dir1.data, 
           msg.pwm1.data, 
           msg.dir2.data, 
           msg.pwm2.data]
    # cast all items to str and join
    msg_string = ''.join((str(e) + " ") for e in pwm)
    # create $MTR string and send
    pwm_string = "$MTR " + msg_string + '\n'
    rospy.logdebug(pwm_string)
    encoded = pwm_string.encode('utf-8')
    ser.write(encoded)

def main():

    rospy.init_node('pico_bridge', anonymous=True, log_level=rospy.DEBUG)

    # publishers for data streams FROM the pico
    nmea_pub = rospy.Publisher('nmea_sentence', nmea.Sentence, queue_size=1)
    cmd_pub = rospy.Publisher('/cmd', rov.Cmd, queue_size=1)
    l_tick_pub = rospy.Publisher('/left_ticks', std.Int64, queue_size=1)
    r_tick_pub = rospy.Publisher('/right_ticks', std.Int64, queue_size=1)
    err_pub = rospy.Publisher('/pico_errors', std.String, queue_size=1)

    # attempt to establish serial connection with the pico
    _port = '/dev/ttyACM0'
    ser = serial.Serial(
        port=_port,
        baudrate=115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS
    )
    rospy.logdebug("Serial Connection established on {}".format(_port))

    # cmd_sub = rospy.Subscriber('/loopback_cmd', rov.Cmd, callback=loopback_cmd_cb, callback_args=(ser))
    tlm_sub = rospy.Subscriber('/telemetry', rov.Telemetry, callback=tlm_cb, callback_args=(ser))
    pwm_sub = rospy.Subscriber('/motors', rov.Motors, callback=pwm_cb, callback_args=(ser))

    if not ser.is_open:
        rospy.logerr("Couldn't open serial port.")
        return

    while not rospy.is_shutdown():

        # read input lines
        try:
            input = ser.readline()
        except serial.SerialException as e:
            rospy.logerr_once("Issue with serial read: {}".format(e))
            continue
        
        # attempt to decode input, strip whitespace
        try:
            input = input.decode().strip()
        except UnicodeDecodeError as e:
            rospy.logerr("Decoding error: {}".format(e))
            continue

        # 'tokenize' by spaces
        input_split = input.split(" ")

        prefix = input_split[0][0:4]    # pull prefix from messages to switch

        # identify NMEA strings, publish them
        if prefix == '$GPS':
            nmea_sentence = str_to_sentence(input_split[1])
            nmea_pub.publish(nmea_sentence)
        # commands from ground station
        elif prefix == '$CMD':
            # switch on cmd message type
            cmd_type = input_split[1]
            if cmd_type == 'MAN1':
                cmd_msg = str_to_command(input_split[2:])
            elif cmd_type == 'MAN2':
                cmd_msg = str_to_rc_command(input_split[2:])

            # cmd = json_message_converter.convert_json_to_ros_message('rover_msg/Cmd', input_split[1])
            if cmd_msg is not None:
                cmd_pub.publish(cmd_msg)
        # encoder ticks to be published
        elif prefix == '$ENC':
            # get ticks messages from string
            left_tick, right_tick = ticks_to_message(input_split[1:])
            # publish tick messages
            if left_tick is not None:
                l_tick_pub.publish(left_tick)
            if right_tick is not None:
                r_tick_pub.publish(right_tick)
        # publish error messages
        elif prefix == '$ERR':
            msg = std.String()
            msg.data = "".join(input_split[1])
            err_pub.publish(msg)
        else:
            rospy.logdebug("Unhandled prefix: {}".format(input))


if __name__=='__main__':
    main()

