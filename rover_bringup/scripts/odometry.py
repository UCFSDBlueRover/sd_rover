#!/usr/bin/env python3

# http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom

import math

import rospy
import tf_conversions
import tf2_ros

# messages
import geometry_msgs.msg as geom
import nav_msgs.msg as nav
import std_msgs.msg as std

#################################################
############### CONSTANTS #######################
#################################################

TICKS_PER_REV = 48
# Wheel radius (meters)
WHEEL_RADIUS = .06 
# Distance from center of left tire to center of right tire (meters)
WHEEL_BASE = .205
METERS_PER_REV = WHEEL_RADIUS * math.pi * 2
REVS_PER_METER = 1 / METERS_PER_REV
# could also be measured manually
TICKS_PER_METER = TICKS_PER_REV * REVS_PER_METER

K_P = 1

##################################################
############## END OF CONSTANTS ##################
##################################################

# track encoder values in class
# since static vars aren't a thing in python
class encoderState():
    def __init__(self):
        self.prevTime= rospy.Time.now()
        self.prevCount = 0
        self.velocity = 0

def tick_cb(ticks: std.Int64, enc: encoderState) -> None:

    """
    When we receive ticks, use the encoderState provided to calculate velocity.
    """

    # ticks since last callback
    ticks = ticks.data - enc.prevCount
    # speed from ticks
    enc.velocity = ticks / TICKS_PER_METER / (rospy.Time.now() - enc.prevTime).to_sec()
    # update timestamp and prevCount
    enc.prevTime = rospy.Time.now()
    enc.prevCount = ticks.data

def main():

    rospy.init_node('motor_control', anonymous=True, log_level=rospy.DEBUG)

    # publishers
    tf_br = tf2_ros.TransformBroadcaster()  # for odom transform
    odom_pub = rospy.Publisher('/rover/odom', nav.Odometry, queue_size=1)   # for odom message

    # listen for ticks
    left_enc = encoderState()
    right_enc = encoderState()
    rospy.Subscriber('/left_ticks', std.Int64, callback=tick_cb, callback_args=(left_enc))
    rospy.Subscriber('/right_ticks', std.Int64, callback=tick_cb, callback_args=(right_enc))

    prevTime = rospy.Time.now()
    
    # start at origin
    x = 0
    y = 0
    theta = 0

    rate = rospy.Rate(10)   # loop rate: 10Hz
    while not rospy.is_shutdown():

        timestamp = rospy.Time.now()

        # calculate velocities in x, y, theta from wheel velocities
        v_left = left_enc.velocity
        v_right = right_enc.velocity

        v_x = ((v_left + v_right) / 2) * K_P
        v_y = 0     # robot can't move sideways
        v_theta = ((v_right - v_left) / WHEEL_BASE) * K_P   # unicycle model

        # predict change in position
        dt = (timestamp - prevTime).to_sec()
        delta_x = (v_x * math.cos(theta)) * dt
        delta_y = (v_x * math.sin(theta)) * dt
        delta_theta = v_theta * dt

        # update position
        x += delta_x
        y += delta_y
        theta += delta_theta

        # broadcast tf transform
        odom_trans = geom.TransformStamped()
        odom_trans.header.stamp = timestamp
        odom_trans.header.frame_id = "odom"
        odom_trans.child_frame_id = "base_link"
        
        odom_trans.transform.translation.x = x
        odom_trans.transform.translation.y = y
        odom_trans.transform.translation.z = 0.0    # we don't move up and down
        odom_quat = tf_conversions.transformations.quaternion_from_euler(0, 0, theta)
        odom_trans.transform.rotation = odom_quat

        tf_br.sendTransform(odom_trans)

        # create Odometry message
        odom = nav.Odometry()
        odom.header.stamp = timestamp
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0     # we "never" move in Z
        odom.pose.pose.orientation = odom_quat

        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = v_x
        odom.twist.twist.linear.y = v_y
        odom.twist.twist.angular.z = v_theta

        odom_pub.publish(odom)

        prevTime = timestamp

        rate.sleep()

if __name__=='__main__':
    main()
