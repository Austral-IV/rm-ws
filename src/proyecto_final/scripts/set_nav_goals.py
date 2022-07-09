#!/usr/bin/env python3

from logging import shutdown
import rospy
import actionlib
from geometry_msgs.msg import Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

#  creo que la orientación se da en quaternions
GOALS = {
    "pos" : [[11.5, 4.2], [20, 17], [30.38, 6.2]], 
    "orient" : [[0, 0, -0.6, 0.8], [0, 0, -0.05, 1], [0, 0, 0.2, 1]]
    }

def goal_done( status, result ):
    global goal
    rospy.loginfo( 'DONE status: %s (%s)', move_base_client.get_goal_status_text(), str( status ) )
    if goal >= 2: 
        Done = True
    else: goal += 1
    


if __name__ == '__main__':
    rospy.init_node( 'nav_stack_example' )

    Done = False

    move_base_client = actionlib.SimpleActionClient( '/move_base', MoveBaseAction )
    move_base_client.wait_for_server()
    
    goal = 0
    while not rospy.is_shutdown() and not Done:
        
        position= GOALS["pos"][goal]
        orientation= GOALS["orient"][goal]

        goal_pose = Pose()
        
        goal_pose.position.x = position[0]
        goal_pose.position.y = position[1]
        goal_pose.orientation.x = orientation[0]
        goal_pose.orientation.y = orientation[1]
        goal_pose.orientation.z = orientation[2]
        goal_pose.orientation.w = orientation[3]

        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose.header.frame_id = 'map'
        move_base_goal.target_pose.header.stamp = rospy.Time.now()
        move_base_goal.target_pose.pose = goal_pose
        move_base_client.send_goal( move_base_goal, done_cb = goal_done )

    rospy.spin()