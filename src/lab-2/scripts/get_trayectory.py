#!/usr/bin/env python3

import rospy
import tf
from getpass import getuser
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


rospy.init_node("get_nav_path", anonymous = True)
path_pub = rospy.Publisher("/nav_plan", Path, queue_size = 100)
print("getting trayectory")
user = getuser()
rec_path = "sin"
pose_file = f"/home/{user}/rm-ws/src/lab-2/scripts/path_{rec_path}.txt"

#Creamos el objeto de tipo Path para el mensaje
msg = Path()
with open(pose_file) as f:
    lines = f.readlines()
    pose_list = [pose.strip().split(",") for pose in lines]
    for pose in pose_list:
        pose_class = PoseStamped()
        #designamos los valores de x e y, el resto no importa.
        # Trasladamos en -1, -1 pq de otro modo sale del mapa.
        pose_class.pose.position.x = float(pose[0]) - 1
        pose_class.pose.position.y = float(pose[1]) - 1

        msg.poses.append(pose_class)

while not rospy.is_shutdown() and path_pub.get_num_connections() == 0: continue

path_pub.publish(msg)
# print(msg.data)
