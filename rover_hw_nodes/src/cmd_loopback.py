#!/usr/bin/env python3

''' A test script that generates a rover_msgs/Cmd message and sends it over USB to the Pico'''

import rospy

import std_msgs.msg as std
import rover_msg.msg as rov
import geometry_msgs.msg as geom

def main():

    rospy.init_node('cmd_loopback', anonymous=True)

    cmd_pub = rospy.Publisher('/loopback_cmd', rov.Cmd, queue_size=1)

    cmd = rov.Cmd()
    cmd.target = geom.Point
    cmd.target.x = 1.0
    cmd.target.y = 1.0
    cmd.target.z = 1.0
    
    # cmd.header = std.Header()
    # cmd.header.stamp = rospy.Time.now()
    # cmd.header.frame_id = '---'

    loop_rate = rospy.Rate(1) # 1 Hz

    while not rospy.is_shutdown():

        cmd_pub.publish(cmd)

        loop_rate.sleep()

if __name__=='__main__':
    main()
