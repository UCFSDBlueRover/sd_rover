#!/usr/bin/env python3

import cmd
import rospy

# ROS messages
import std_msgs.msg as std
import nav_msgs.msg as nav
import sensor_msgs.msg as sensor
import geometry_msgs.msg as geom
import rover_msg.msg as rov
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def main():

    # initialize ROS node
    rospy.init_node('bypass_boot', anonymous=True, log_level=rospy.DEBUG)

    hb_pub = rospy.Publisher('/heartbeat', rov.Heartbeat, queue_size=1)
    cmd_pub = rospy.Publisher('/cmd', rov.Cmd, queue_size=1)
    gps_pub = rospy.Publisher('/fix', sensor.NavSatFix, queue_size=1)
    imu_pub = rospy.Publisher('/imu', sensor.Imu, queue_size=1)
    odom_pub = rospy.Publisher('/odom', nav.Odometry, queue_size=1)

    loop_rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        # publish a heartbeat message
        hb = rov.Heartbeat()
        hb.time = rospy.Time.now()
        hb_pub.publish(hb)

        # publish a cmd message (with start flag true)
        cmd = rov.Cmd()
        cmd.start.data = True
        cmd_pub.publish(cmd)

        # publish an (empty) GPS message
        fix = sensor.NavSatFix()
        fix.status.status = 0
        gps_pub.publish(fix)

        # publish the (empty) IMU message
        imu = sensor.Imu()
        imu.header = std.Header()
        imu.header.stamp = rospy.Time.now()
        imu_pub.publish(imu)

        # publish an (empty) Odom message
        odom = nav.Odometry()
        odom.header = std.Header()
        odom.header.stamp = rospy.Time.now()
        odom_pub.publish(odom)

        loop_rate.sleep()


if __name__ == '__main__':
    main()