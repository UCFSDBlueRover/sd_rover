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

from rover_states import Boot, Standby

class BootTest(unittest.TestCase):

    def setUp(self):

        rospy.init_node("test_states", anonymous=True, log_level=rospy.DEBUG)

        self.outcomes = {'error', 'boot_success'}   # this is a set

        self.boot = Boot()

    def tearDown(self):
        # kill the ROS node
        # http://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown
        rospy.signal_shutdown('')

    def test_boot_init(self):

        # make sure our outcome sets match
        self.assertSetEqual(self.boot._outcomes, self.outcomes)

    def test_boot_hb_callback(self):

        pass

class StandbyTest(unittest.TestCase):

    def setUp(self):

        rospy.init_node("test_states", anonymous=True, log_level=rospy.DEBUG)

        # publisher will be used to transition the sm with Cmd messages
        self.cmd_pub = rospy.Publisher('/command', rov.Cmd, queue_size=10)

        self.outcomes = ['got_pose', 'rc_preempt', 'error', 'end']

        # create state machine with outcomes
        # state machine will contain ONLY standby state
        self.sm = smach.StateMachine(outcomes=self.outcomes)

        with self.sm:
            smach.StateMachine.add('STANDBY',
                                    Standby(),
                                    # transitions are identical to outcomes, since we only have one state in this container
                                    transitions={'end':'end', 'got_pose':'got_pose', 'rc_preempt':'rc_preempt', 'error':'error'},
                                    remapping={'pose_target','pose_target'})

        self.outcome = self.sm.execute()

    def tearDown(self):
        # kill the ROS node
        # http://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown
        rospy.signal_shutdown('')

    # def test_standby_init(self):

    #     # make sure our outcome sets match
    #     self.assertSetEqual(self.standby._outcomes, self.outcomes)

    def test_standby_shutdown_preempt(self):

        ''' verify that we can throw the shutdown transition with a command message flag '''

        # create the command message
        cmd = rov.Cmd()
        cmd.shutdown.data = True

        # publish command message
        self.cmd_pub.publish(cmd)

        # wait for state machine to transition
        while not rospy.is_shutdown():
            if self.outcome is not None:
                self.assertEqual(self.outcome, 'end')

    def test_standby_rc_preempt(self):

        ''' verify that we can throw the rc_preempt transition with a command message flag'''

        # create the command message
        cmd = rov.Cmd()
        cmd.rc_preempt.data = True        

        # publish command message
        self.cmd_pub.publish(cmd)

        # wait for state machine to transition
        while not rospy.is_shutdown():
            if self.outcome is not None:
                self.assertEqual(self.outcome, 'rc_preempt')

    def test_standby_pose_target_preempt(self):

        ''' verify that we can throw the pose_target transition with a command message '''

        # create the command message
        cmd = rov.Cmd()
        
        # create the twist message, actual fields don't matter
        pose = geom.PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = 1.0
        pose.pose.position.y = 1.0
        pose.pose.position.z = 1.0

        cmd.pose_target = pose

        # publish command message
        self.cmd_pub.publish(cmd)

        # wait for state machine to transition
        while not rospy.is_shutdown():
            if self.outcome is not None:
                self.assertEqual(self.outcome, 'pose_target')
                break

        # make sure the pose target that gets propagated matches what we sent
        self.assertEqual(self.sm.userdata.pose_target, pose)


if __name__=='__main__':
    unittest.main()