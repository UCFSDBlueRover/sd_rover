#!/usr/bin/env python3

# ROS system imports
import rospy
import actionlib
import smach, smach_ros

from rospy_message_converter import json_message_converter

# ROS messages
import std_msgs.msg as std
import nav_msgs.msg as nav
import sensor_msgs.msg as sensor
import geometry_msgs.msg as geom
import rover_msg.msg as rov
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class Boot(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['error', 'boot_success'])

        # TODO: define message callbacks for topics to watch and throw flags
        # what needs to be verified before we can begin?

        self.config = self.read_params() # TODO

        # checking that all input sources are up
        # # RX HEARTBEAT
        # rospy.Subscriber('/heartbeat', rov.Heartbeat, callback=self.hb_callback)
        # INIT message from GS
        rospy.Subscriber('/cmd', rov.Cmd, callback=self.cmd_callback)
        # GPS data 
        rospy.Subscriber('/fix', sensor.NavSatFix, callback=self.GPS_callback)
        # IMU data
        rospy.Subscriber('/imu', sensor.Imu, callback=self.IMU_callback)
        # odom
        rospy.Subscriber('/odom', nav.Odometry, callback=self.odom_callback)

        # self._hb_flag = False
        self._cmd_flag = False
        self._gps_flag = False
        self._imu_flag = False
        self._odom_flag = False

        # received ACK from all software modules (define list in XML/YAML format?)

    def execute(self, userdata):

        rate = rospy.Rate(20)

        # configure timer to output status of subscribers every 5 secs
        status_timer = rospy.Timer(rospy.Duration(5), self.timer_status_callback)

        # configure timeout for switching to WARN state
        # TODO

        while not rospy.is_shutdown():

            # if self._hb_flag and self._cmd_flag and self._gps_flag and self._imu_flag:
            if self._cmd_flag and self._gps_flag and self._imu_flag and self._odom_flag:
                rospy.logdebug("All sources up. Transitioning to STANDBY.")
                # end the status timer
                status_timer.shutdown()
                return 'boot_success'
            
            rate.sleep()

    def timer_status_callback(self, event):

        rospy.logdebug(
            # "\nHEARTBEAT: \t{}\n".format(self._hb_flag) + \
            "\nCMD: \t\t{}\n".format(self._cmd_flag) + \
            "GPS: \t\t{}\n".format(self._gps_flag) + \
            "IMU: \t\t{}\n".format(self._imu_flag) + \
            "ODOM: \t\t{}\n".format(self._odom_flag)
        )

    # def hb_callback(self, msg):

    #     # make sure it's a valid message
    #     if msg.time is not None:
    #         self._hb_flag = True

    def cmd_callback(self, msg):

        try:
            if msg.start.data is True:
                self._cmd_flag = True
        except AttributeError as e:
            pass
    
    def GPS_callback(self, msg):

        try:
            if msg.status.status != -1:
                self._gps_flag = True
        except AttributeError as e:
            pass

    def IMU_callback(self, msg):

        if msg.header is not None:
            self._imu_flag = True

    def odom_callback(self, msg):

        if msg.header is not None:
            self._odom_flag = True

    # TODO
    def read_params(self):
        ''' Reads parameters from loaded config file to set up this states topic subscribers/publishers, etc. '''
        pass

class Standby(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['got_pose', 'rc_preempt', 'error', 'end'],
                                   output_keys=['pose_target', 'rc_msg', 'end_status', 'end_reason'] )

        # flags
        self._rc_preempt = False
        self._pose_preempt = False
        self._end = False

        # userdata
        self._pose_target = None    # _pose_target is filled when we receive a pose target from Cmd
        self._rc_msg = None         # _rc_msg holds RC commands from Cmd

    def execute(self, userdata):

        rate = rospy.Rate(20)

        # wait until execute to initialize subscribers so that multiple states can listen to same topic names without clashing
        rospy.Subscriber('/cmd', rov.Cmd, callback=self.cmd_callback)

        while not rospy.is_shutdown():

            # if we received motor commands, 
            if self._rc_preempt:
                rospy.logdebug("Standby preempted by RC command.")
                userdata.rc_msg = json_message_converter.convert_ros_message_to_json(self._rc_msg)  # add userdata for MANUAL
                return 'rc_preempt'
            # if we received a pose in Command msg, pass that as output of state
            if self._pose_preempt:
                # userdata cannot hold arbitrary objects like ROS messages
                # so, convert ROS message to json (string), then convert again at destination
                userdata.pose_target = json_message_converter.convert_ros_message_to_json(self._pose_target)
                rospy.logdebug("Standby preempted by pose target.")
                return 'got_pose'

            # check for errors
            # if err:
            #   userdata.end_reason = 'Fatal Error'
            #   userdata.end_status = 'err'
            #   return 'end'

            # check for user-initiated end
            if self._end:
                rospy.logdebug('Standby preempted by SHUTDOWN signal.')
                userdata.end_reason = 'User initiated end state.'
                userdata.end_status = 'success'
                return 'end'
            
            rate.sleep()


    def cmd_callback(self, data):

        ''' TODO: check for override flag in RC message, throw rc_preempt '''

        # assumes that pose_target field of Command msg is empty if we don't want waypoint navigation
        if data.pose_preempt is not None:
            if data.pose_preempt.data:
                self._pose_target = data.target
                self._pose_preempt = True

        if data.rc_preempt is not None:
            if data.rc_preempt.data:
                self._rc_msg = data.rc
                self._rc_preempt = True
            
        # check for the shutdown flag    
        if data.shutdown is not None:
            if data.shutdown.data:
                self._end = True


class Waypoint(smach.State):

    ''' pose_target contains the pose target that initiated the switch to waypoint 
        NOT guaranteed to remain the pose_target during operation '''

    def __init__(self):
        smach.State.__init__(self, outcomes=['nav_finish', 'rc_preempt', 'error'],
                                   input_keys=['pose_target'],
                                   output_keys=['status', 'rc_msg'])

        # create the client that will connect to move_base
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        
        self._target_update = False    # flag is TRUE when a pose update is received (a pose different from the one we were previously given is received)
        self._rc_preempt = False

        # the pose target that WAYPOINT will use for navigation
        self._pose_target = None
        self.goal = None

        self._rc_msg = None

    def execute(self, userdata):

        ''' manages the lifecycle of calls to the autonomy stack '''

        # listen for command updates that (might) change our pose_target
        cmd_sub = rospy.Subscriber('/cmd', rov.Cmd, callback=self.cmd_callback)

        # get input pose target (pose that caused us to transition to Waypoint)
        # needs to be converted from JSON to ROS message
        self._pose_target = json_message_converter.convert_json_to_ros_message('geometry_msgs/Point', userdata.pose_target)

        # make sure we have connection to client server before continuing, timeout after 10s
        self.client.wait_for_server(timeout=rospy.Duration(10))
        
        # send the initial goal based on the pose target that brought us to this state
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "odom"   # MUST be in odom frame
        self.goal.target_pose.header.stamp = rospy.Time.now()

        # read our pose_target from userdata
        self.goal.target_pose.pose.position = self._pose_target
        # use arbitrary orientation (we don't care right now)
        self.goal.target_pose.pose.orientation.x = 0
        self.goal.target_pose.pose.orientation.y = 0
        self.goal.target_pose.pose.orientation.z = 0
        self.goal.target_pose.pose.orientation.w = 1

        self.client.send_goal(self.goal)

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():

            if self._rc_preempt:
                self._rc_preempt = False
                userdata.rc_msg = json_message_converter.convert_ros_message_to_json(self._rc_msg)
                rospy.logdebug("Preempted by RC.")
                return 'rc_preempt'

            if self._target_update:
                rospy.logdebug("Updating goal to \n\t{}".format(self.goal.serialize))
                self.client.send_goal(self.goal)
                self._target_update = False

            current_state = self.client.get_state()
            if current_state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal Succeeded.\n {}".format(self.client.get_goal_status_text()))
                return 'nav_finish'
            elif current_state == actionlib.GoalStatus.REJECTED or \
                 current_state == actionlib.GoalStatus.ABORTED:
                rospy.logerr("Goal returned an error:\n\t {}".format(self.client.get_goal_status_text))
                return 'error'
            elif current_state == actionlib.GoalStatus.LOST:
                rospy.logerr("Goal state returned LOST. Are we sure we sent a goal?")

            rate.sleep()

    def cmd_callback(self, msg):

        if msg.target is not None and self.goal is not None:
            # if we received a target goal that doesn't match what we already have
            if msg.target != self.goal.target_pose.pose.position:
                # cancel all goals
                self.client.cancel_all_goals()
                # set our goal to the new target
                self.goal.target_pose.pose.position = msg.target
                # set the flag to notify our main loop
                self._target_update = True
        
        if msg.rc_preempt is not None:
            if msg.rc_preempt:
                self._rc_msg = msg.rc
                self._rc_preempt = True
    
class Manual(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['resume_standby', 'resume_waypoint', 'error'],
                                   input_keys=['rc_msg'],
                                   output_keys=[])

        # flags
        self._rc_un_preempt = False # if true, we return to previous state

        # publish motor commands for the base_controller to actuate
        self.motor_pub = rospy.Publisher('/cmd_vel', geom.Twist, queue_size=10)

        self._fwd_period    = rospy.Duration(2.5)    # how long each forward RC burst lasts
        self._rev_period    = rospy.Duration(2.5)    # how long each revers RC burst lasts
        self._left_period   = rospy.Duration(2.5)    # time to turn roughly -90 degrees
        self._right_period  = rospy.Duration(2.5)    # time to turn roughly +90 degrees

        self._rc = rov.RC()
        self._rc_update = True # update flag

    def execute(self, userdata):

        # subscribe to commands
        rospy.Subscriber('/cmd', rov.Cmd, callback=self.cmd_callback)

        # read the userdata to find the RC message that got us here
        self._rc = json_message_converter.convert_json_to_ros_message("rover_msg/RC", userdata.rc_msg)
        self._rc_update = True

        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            
            if self._rc_update:
                
                # get burst period based on which field is filled
                # making the assumption here that only one RC field is used at a time
                period = None
                dir = 1
                if self._rc.forward > 0:
                    dir = 1
                    period = self._fwd_period
                elif self._rc.reverse > 0:
                    dir = -1
                    period = self._rev_period
                elif self._rc.left > 0:
                    period = self._left_period
                elif self._rc.right > 0:
                    period = self._right_period

                # create message to publish
                # values assigned don't matter much, since they'll be truncated by motor_control
                twist = geom.Twist()
                twist.linear.x = (1.0 * dir) if (self._rc.forward > 0) else 0.0
                twist.linear.y = (1.0 * dir) if (self._rc.reverse > 0) else 0.0
                twist.linear.z = 0.0
                twist.angular.x = 0
                twist.angular.y = 0
                twist.angular.z = (1.0 * dir) if (self._rc.left > 0 or self._rc.right > 0) else 0.0

                time = rospy.Time.now()
                loop_rate = rospy.Rate(5)   # Hz
                # set /cmd_vel message for given duration
                while rospy.Time.now() < (time + period):
                    
                    self.motor_pub.publish(twist)

                    loop_rate.sleep()
                
                self._rc_update = False

            if self._rc_un_preempt:
                return 'rc_un_preempt'

            rate.sleep()

    def cmd_callback(self, data):

        if data.rc_preempt is not None:

            if data.rc_preempt:
                self._rc = data.rc
                self._rc_update = True
            else:
                self._rc_un_preempt = True

class Warn(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['reset', 'standby', 'end'],
                                   output_keys=['end_status', 'end_reason'])

    def execute(self, userdata):

        pass

class End(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['end_success', 'end_err'],
                                   input_keys=['end_status', 'end_reason'])

    def execute(self, userdata):

        # kill the ROS node
        # http://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown
        rospy.signal_shutdown(userdata.end_reason)

        if userdata.end_status == 'success':
            return 'end_success'
        elif userdata.end_status == 'err':
            return 'end_err'

def main():

    # initialize ROS node
    rospy.init_node('rover_sm', anonymous=True, log_level=rospy.DEBUG)

    # TODO: publisher for Telemetry message

    # create state machine with outcomes
    sm = smach.StateMachine(outcomes=['success', 'err'])

    # declare userdata
    sm.userdata.pose_target = ""
    sm.userdata.rc_msg = ""
    sm.userdata.end_status = ""
    sm.userdata.end_reason = ""

    # define states within sm
    with sm:
        smach.StateMachine.add('BOOT', 
            Boot(), 
            transitions={'error':'WARN', 'boot_success':'STANDBY'}, 
            remapping={})
        smach.StateMachine.add('STANDBY',
            Standby(),
            transitions={'got_pose':'WAYPOINT', 'rc_preempt':'MANUAL', 'error':'WARN', 'end':'END'},
            remapping={'pose_target':'pose_target', 'end_status':'end_status', 'end_reason':'end_reason'})
        smach.StateMachine.add('WAYPOINT',
            Waypoint(),
            transitions={'nav_finish':'STANDBY', 'rc_preempt':'MANUAL', 'error':'WARN'},
            remapping={'pose_target':'pose_target', 'status':'waypoint_status'})
        smach.StateMachine.add('MANUAL',
            Manual(),
            transitions={'resume_standby':'STANDBY', 'resume_waypoint':'WAYPOINT', 'error':'WARN'},
            remapping={})
        smach.StateMachine.add('WARN',
            Warn(),
            transitions={'reset':'BOOT', 'standby':'STANDBY', 'end':'END'},
            remapping={})
        smach.StateMachine.add('END',
            End(),
            transitions={'end_success':'success', 'end_err':'err'},
            remapping={'end_status':'end_status', 'end_reason':'end_reason'})


    # create an introspection server for debugging transitions
    introspect = smach_ros.IntrospectionServer('rover_sm_info', sm, '/SM_ROOT')
    introspect.start()

    outcome = sm.execute()

    rospy.spin()
    introspect.stop()

if __name__ == '__main__':
    main()