#!/usr/bin/env python3

from asyncore import read
from hashlib import new
import rospy
from geometry_msgs.msg import PoseArray, Pose
import json
from getpass import getuser

user = getuser()
pose_file = f"/home/{user}/rm-ws/src/lab-2/scripts/path_sin.txt"

rospy.init_node("read_poses", anonymous = True)
goal_pub = rospy.Publisher("/goal_list", PoseArray, queue_size = 10)

with open(pose_file) as f:
    lines = f.readlines()
    pose_list = [pose.strip().split(",") for pose in lines]
# for pose in pose_list: print(pose)

read_poses = PoseArray()

for pose in pose_list:
    new_pose = Pose()
    new_pose.position.x = float(pose[0])
    new_pose.position.y = float(pose[1])
    new_pose.position.z = 0

    new_pose.orientation.x = 0
    new_pose.orientation.y = 0
    new_pose.orientation.z = 0
    new_pose.orientation.w = 0

    read_poses.poses.append(new_pose)

while not rospy.is_shutdown() and goal_pub.get_num_connections() == 0:
    continue

goal_pub.publish(read_poses)
