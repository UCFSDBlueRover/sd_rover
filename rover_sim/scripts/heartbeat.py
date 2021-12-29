#!/usr/bin/env python3

''' Publishes the Heartbeat message type for simulation (since there's nothing transmitting it)
'''

import rospy
from rover_msg.msg import Heartbeat

def main():

    rospy.init_node('sim_heartbeat', anonymous=True)

    heart_pub = rospy.Publisher('/heartbeat', Heartbeat, queue_size=10)

    rate = rospy.Rate(.2) # every five seconds

    while not rospy.is_shutdown():

        hb = Heartbeat()
        hb.time = rospy.Time.now()

        heart_pub.publish(hb)

        rate.sleep()


if __name__=='__main__':
    main()

