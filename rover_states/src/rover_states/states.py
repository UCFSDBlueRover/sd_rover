#!/usr/bin/env python3

# ROS system imports
from xml.dom.minidom import Attr
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

        # RX HEARTBEAT
        rospy.Subscriber('/heartbeat', rov.Heartbeat, callback=self.hb_callback)
        # INIT message from GS
        rospy.Subscriber('/cmd', rov.Cmd, callback=self.cmd_callback)
        # GPS data 
        rospy.Subscriber('/fix', sensor.NavSatFix, callback=self.GPS_callback)
        # IMU data
        rospy.Subscriber('/imu', sensor.Imu, callback=self.IMU_callback)
        # Camera data
        # 
        # odom?

        self._hb_flag = False
        self._cmd_flag = False
        self._gps_flag = False
        self._imu_flag = False

        # RX data from all sensor stream topics

        # received ACK from all software modules (define list in XML/YAML format?)

    def execute(self, userdata):

        rate = rospy.Rate(20)

        # configure timer to output status of subscribers every 5 secs
        status_timer = rospy.Timer(rospy.Duration(5), self.timer_status_callback)

        # configure timeout for switching to WARN state
        # TODO

        while not rospy.is_shutdown():

            if self._hb_flag and self._cmd_flag and self._gps_flag and self._imu_flag:
                rospy.logdebug("All sources up. Transitioning to STANDBY.")
                # end the status timer
                status_timer.shutdown()

                return 'boot_success'

            # what constitutes an error?
            rate.sleep()

    def timer_status_callback(self, event):

        rospy.logdebug(
            "\nHEARTBEAT: \t{}\n".format(self._hb_flag) + \
            "CMD: \t\t{}\n".format(self._cmd_flag) + \
            "GPS: \t\t{}\n".format(self._gps_flag) + \
            "IMU: \t\t{}\n".format(self._imu_flag)
        )

    def hb_callback(self, msg):

        # make sure it's a valid message
        if msg.time is not None:
            self._hb_flag = True

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

    # TODO
    def read_params(self):
        ''' Reads parameters from loaded config file to set up this states topic subscribers/publishers, etc. '''
        pass

class Standby(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['got_pose', 'rc_preempt', 'error', 'end'],
                                   output_keys=['pose_target', 'end_status', 'end_reason'] )

        # flags
        self._rc_preempt = False
        self._pose_preempt = False
        self._end = False

        # _pose_target is filled when we receive a pose target from Cmd
        self._pose_target = None

    def execute(self, userdata):

        rate = rospy.Rate(20)

        # wait until execute to initialize subscribers so that multiple states can listen to same topic names without clashing
        rospy.Subscriber('/cmd', rov.Cmd, callback=self.cmd_callback)

        while not rospy.is_shutdown():

            # if we received motor commands, 
            if self._rc_preempt:
                rospy.logdebug("Standby preempted by RC command.")
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
                rospy.logdebug('Standby preempted by end signal.')
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
                                   output_keys=['status'])

        # create the client that will connect to move_base
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        
        
        self.pose_update = False    # flag is TRUE when a pose update is received (a pose different from the one we were previously given is received)

        # the pose target that WAYPOINT will use for navigation
        self._pose_target = None

    def execute(self, userdata):

        ''' manages the lifecycle of calls to the autonomy stack '''

        # listen for pose updates that will change our pose_target
        pose_sub = rospy.Subscriber('/pose_updates', geom.Point, callback=self.pose_callback)

        # get input pose target (pose that caused us to transition to Waypoint)
        # needs to be converted from JSON to ROS message
        self._pose_target = json_message_converter.convert_json_to_ros_message('geometry_msgs/Point', userdata.pose_target)
        self._pose_update = True

        # make sure we have connection to client server before continuing
        self.client.wait_for_server()

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():

            # will need to use an actionlib connection to move_base, like below:
            # https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/
            # this approach may require multithreading to not block the main loop

            # for a non-blocking method: https://answers.ros.org/question/347823/action-client-wait_for_result-in-python/
            # 1. send goal with send_goal()
            # 2. query action server for state with getState()
            # 3. if we're in the appropriate state, getResult()
            # api docs: http://docs.ros.org/melodic/api/actionlib/html/classactionlib_1_1simple__action__client_1_1SimpleActionClient.html

            # if we received a pose update, create a new goal
            if self._pose_update:
                
                # goal = MoveBaseGoal()

                # goal.target_pose.header.frame_id = "map"
                # goal.target_pose.header.stamp = rospy.Time.now()
                # goal.target_pose.pose.position.x = 0.5
                # goal.target_pose.pose.orientation.w = 1.0

                # client.send_goal(goal)
                # wait = client.wait_for_result()
                # if not wait:
                #     rospy.logerr("Action server not available!")
                #     rospy.signal_shutdown("Action server not available!")
                # else:
                #     return client.get_result()

                # reset the flag
                self._pose_update = False

            rate.sleep()

    def pose_callback(self, data):
        # update internal pose target
        self._pose_target = data.pose
        # set flag to true
        self._pose_update = True
    
class Manual(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['resume_standby', 'resume_waypoint', 'error'])
        # TODO: add input_key prev_state

        # flags
        self._rc_un_preempt = False # if true, we return to previous state

        # publish motor commands for the base_controller to actuate
        self.motor_pub = rospy.Publisher('/cmd_vel', geom.Twist, queue_size=10)

        # subscribe to commands
        rospy.Subscriber('/command', rov.Cmd, callback=self.cmd_callback)

    def execute(self, userdata):

        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
        
            if self._rc_un_preempt:
                return 'rc_un_preempt'

            rate.sleep()

    def cmd_callback(self, data):

        if data.rc_preempt is not None:

            # if our RC preempt is active, pass motor commands to base_controller via /cmd_vel
            if data.rc_preempt.data:
                self.motor_pub.publish(data.motors)
            elif not data.rc_preempt.data:

                # zero out a Twist message to stop motors
                twist = geom.Twist()
                twist.linear.x = 0
                twist.linear.y = 0
                twist.linear.z = 0
                twist.angular.x = 0
                twist.angular.y = 0
                twist.angular.z = 0

                self.motor_pub.publish(twist)

                # set rc_un_preempt to true to return to previous state
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