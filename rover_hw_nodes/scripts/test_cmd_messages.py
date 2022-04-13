#!/usr/bin/env python3

# ros imports
import rospy

# message imports
import std_msgs.msg as std
import geometry_msgs.msg as geom
import rover_msg.msg as rov

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
    # cmd.rc_preempt.data     = (input[6] == 'True')
    cmd.rc_preempt.data     = False     # rc_preempt handled by different cmd string now
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

def main():

    rospy.init_node('test_cmd_messages', anonymous=True, log_level=rospy.DEBUG)

    cmd_pub = rospy.Publisher('/cmd', rov.Cmd, queue_size=1)

    input_str = None

    while not rospy.is_shutdown():

        input_str = input("MSG: ")

        # 'tokenize' by spaces
        input_split = input_str.split(" ")

        prefix = input_split[0][0:4]    # pull prefix from messages to switch

        if prefix == '$CMD':
            # switch on cmd message type
            cmd_type = input_split[1]
            if cmd_type == 'MAN1':
                cmd_msg = str_to_command(input_split[2:])
            elif cmd_type == 'MAN2':
                cmd_msg = str_to_rc_command(input_split[2:])

            # cmd_msg = str_to_command(input_split[1:])
            # cmd = json_message_converter.convert_json_to_ros_message('rover_msg/Cmd', input_split[1])
            if cmd_msg is not None:
                cmd_pub.publish(cmd_msg)
        else:
            rospy.logdebug("The fuck is this?: {}".format(input))
            # print(input)

if __name__=='__main__':
    main()