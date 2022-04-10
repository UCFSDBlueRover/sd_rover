#!/usr/bin/env python3

# ROS system imports
import rospy

# ROS messages
import std_msgs.msg as std
import nav_msgs.msg as nav
import sensor_msgs.msg as sensor
import geometry_msgs.msg as geom
import rover_msg.msg as rov

def gps_callback(fix: sensor.NavSatFix, tlm: rov.Telemetry) -> None:

    tlm.fix = fix

def pose_callback(odom: nav.Odometry, tlm: rov.Telemetry) -> None:

    tlm.pose = geom.PoseStamped()
    tlm.pose.header = odom.header
    tlm.pose.pose = odom.pose.pose

def state_callback(state: std.String, tlm: rov.Telemetry) -> None:

    tlm.state = state

def main():

    # initialize ROS node
    rospy.init_node('telemetry_pub', anonymous=True, log_level=rospy.DEBUG)

    tlm = rov.Telemetry()
    tlm.fix = sensor.NavSatFix()
    tlm.pose = geom.PoseStamped()
    tlm.state.data = "-"

    # configure subscriber callbacks
    rospy.Subscriber('/fix', sensor.NavSatFix, callback=gps_callback, callback_args=(tlm))  # GPS
    rospy.Subscriber('/odom', nav.Odometry, callback=pose_callback, callback_args=(tlm))
    rospy.Subscriber('/states/state', std.String, callback=state_callback, callback_args=(tlm))

    # configure tlm publisher
    tlm_pub = rospy.Publisher('/telemetry', rov.Telemetry, queue_size=1)

    rate = rospy.Rate(.2)   # every ten seconds

    while not rospy.is_shutdown():

        tlm_pub.publish(tlm)
        
        rate.sleep()

if __name__=='__main__':
    main()



