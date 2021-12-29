#!/usr/bin/env python

# system imports
import unittest
import random

# ROS imports
import rospy
import smach, smach_ros

import std_msgs.msg as std
import nav_msgs.msg as nav
import geometry_msgs.msg as geom
import rover_msg.msg as rov

from rover_states import Boot

class BootTest(unittest.TestCase):

    def setUp(self):

        rospy.init_node("test_states", anonymous=True, log_level=rospy.DEBUG)

        self.outcomes = {'error', 'boot_success'}   # this is a set

        self.boot = Boot()

    def test_boot_init(self):

        # make sure our outcome sets match
        self.assertSetEqual(self.boot._outcomes, self.outcomes)

    def test_boot_hb_callback(self):

        pass

if __name__=='__main__':
    unittest.main()