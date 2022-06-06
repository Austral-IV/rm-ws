#!/usr/bin/env python3

from asyncore import read
from hashlib import new
import rospy
from geometry_msgs.msg import PoseArray, Pose
import json
from getpass import getuser

user = getuser()
pose_file = f"/home/{user}/rm-ws/src/lab-2/scripts/poses.txt"

rospy.init_node("read_poses", anonymous = True)
goal_pub = rospy.Publisher("/goal_list", PoseArray, queue_size = 10)

with open(pose_file) as f:
    pose_list = json.load(f)

read_poses = PoseArray()

for pose in pose_list:
    new_pose = Pose()
    new_pose.position.x = pose[0][0]
    new_pose.position.y = pose[0][1]
    new_pose.position.z = pose[0][2]

    new_pose.orientation.x = pose[1][0]
    new_pose.orientation.y = pose[1][1]
    new_pose.orientation.z = pose[1][2]
    new_pose.orientation.w = pose[1][3]

    read_poses.poses.append(new_pose)

while not rospy.is_shutdown() and goal_pub.get_num_connections() == 0:
    continue

goal_pub.publish(read_poses)
