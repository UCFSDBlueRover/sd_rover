#!/usr/bin/env python3

# ROS system imports
import rospy
import actionlib

# ROS messages
import std_msgs.msg as std
import nav_msgs.msg as nav
import sensor_msgs.msg as sensor
import geometry_msgs.msg as geom
import rover_msg.msg as rov
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import GoalStatus as status

def main():

    rospy.init_node('movebase_client', anonymous=True, log_level=rospy.DEBUG)
    
    # create server connection
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    # block until we connect to server
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 1.0
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)

    rate = rospy.Rate(1)
    # wait until we reach a terminal state
    # Possible States Are: PENDING, ACTIVE, RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, LOST.
    while (client.get_state() == status.PENDING) or (client.get_state() == status.ACTIVE):

        rospy.logdebug("Current status: {}".format(client.get_goal_status_text()))

        rate.sleep()
    
    if client.get_result() != status.SUCCEEDED:
        rospy.logerr("Goal failed. Returned: {}".format(client.get_result()))
    else:
        rospy.loginfo("Goal succeeded. Returned: {}". client.get_result())

if __name__ == '__main__':
    main()