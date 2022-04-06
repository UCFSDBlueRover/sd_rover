#!/usr/bin/env python3

# ROS system imports
import rospy

# ROS messages
import std_msgs.msg as std
import nav_msgs.msg as nav
import sensor_msgs.msg as sensor
import geometry_msgs.msg as geom
import rover_msg.msg as rov

def gps_callback(fix: sensor.NavSatFix, tlm) -> None:

    tlm.fix = fix

def pose_callback(odom: nav.Odometry, tlm) -> None:

    tlm.pose = geom.PoseStamped()
    tlm.pose.header = odom.header
    tlm.pose.pose = odom.pose.pose

def state_callback(state: std.String, tlm) -> None:

    tlm.state = state

def main():

    # initialize ROS node
    rospy.init_node('telemetry_pub', anonymous=True, log_level=rospy.DEBUG)

    tlm = rov.Telemetry()

    # configure subscriber callbacks
    rospy.Subscriber('/fix', sensor.NavSatFix, callback=gps_callback, callback_args=(tlm))  # GPS
    rospy.Subscriber('/pose', nav.Odometry, callback=pose_callback, callback_args=(tlm))
    rospy.Subscriber('/states/state', std.String, callback=state_callback, callback_args=(tlm))

    # configure tlm publisher
    tlm_pub = rospy.Publisher('/telemetry', rov.Telemetry, queue_size=1)

    rate = rospy.Rate(.1)   # every ten seconds

    while not rospy.is_shutdown():

        tlm_pub.publish(tlm)
        
        rate.sleep()




