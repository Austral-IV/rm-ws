#!/usr/bin/env python3

from logging import shutdown
import rospy
import actionlib
from geometry_msgs.msg import Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal



    
class GoalSender():
    def __init__(self):

        #  creo que la orientación se da en quaternions
        self.GOALS = {
            "pos" : [[11.5, 4.2], [20, 17], [30.38, 6.2]], 
            "orient" : [[0, 0, -0.6, 0.8], [0, 0, -0.05, 1], [0, 0, 0.2, 1]]
            }

        rospy.init_node( 'nav_stack_example' )
        self.move_base_client = actionlib.SimpleActionClient( '/move_base', MoveBaseAction )
        self.move_base_client.wait_for_server()
        self.move_base_goal = MoveBaseGoal()
        self.move_base_goal.target_pose.header.frame_id = 'map'
        self.move_base_goal.target_pose.header.stamp = rospy.Time.now()
    
    def run(self):
        self.goal = 0
        self.set_goal()
    
    def goal_done(self, status, result):
        # rospy.loginfo( 'DONE status: %s (%s)', self.move_base_client.get_goal_status_text(), str( status ) )
        goal_coord = self.GOALS["pos"][self.goal]
        rospy.loginfo( f"Reached Goal: {goal_coord}")
        if self.goal >= 2: pass
        self.goal += 1
        self.set_goal()

    def set_goal(self):
        goal = self.goal
        position= self.GOALS["pos"][goal]
        orientation= self.GOALS["orient"][goal]
        goal_pose = Pose()

        goal_pose.position.x = position[0]
        goal_pose.position.y = position[1]
        goal_pose.orientation.x = orientation[0]
        goal_pose.orientation.y = orientation[1]
        goal_pose.orientation.z = orientation[2]
        goal_pose.orientation.w = orientation[3]

        self.move_base_goal.target_pose.pose = goal_pose
        self.move_base_client.send_goal( self.move_base_goal, done_cb = self.goal_done )


if __name__ == '__main__':
    rospy.init_node( 'nav_stack_example' )
    
    Done = False
    reached_goal = False

    goalsender = GoalSender()
    goalsender.run()

    rospy.spin()