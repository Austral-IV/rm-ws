#!/usr/bin/env python3

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import PointCloud, Image
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from std_msgs.msg import Empty

from cv_bridge import CvBridge

class DisplayMap():
  
  def __init__(self):
    rospy.init_node( 'diplay_map' )
    self.variables_init()
    self.connections_init()
    rospy.spin()
  

  def variables_init(self):
    self.draw_map = DrawMap()
    self.bridge = CvBridge()
    self.map = None
    self.point_cloud = None


  def connections_init(self):
    self.pub_map =rospy.Publisher('/img_map', Image, queue_size=10)
    rospy.Subscriber( '/map', OccupancyGrid, self.set_map)
    rospy.Subscriber( '/lidar_points', PointCloud, self.show_pointcloud)
    rospy.Subscriber( '/show_map', Empty, self.show_map)
  

  def set_map(self, map):
    width = map.info.width
    height = map.info.height
    np_map = np.array(map.data)

    #rospy.loginfo(np_map.shape)

    np_map = np_map.reshape( (height, width) )
    #rospy.loginfo(np_map.shape)

    mapimg = 100 - np_map
    mapimg = ( mapimg * (255/100.0) ).astype( np.uint8 )
    self.map = cv2.cvtColor( mapimg, cv2.COLOR_GRAY2RGB )


  def show_pointcloud(self, pointcloud):
    points = pointcloud.points

    map_copy = self.map.copy()

    self.draw_map.update_odom_pix()
    self.draw_map.draw_robot(map_copy)
    self.draw_map.draw_point_cloud(map_copy, points)

    img_msg = self.bridge.cv2_to_imgmsg(map_copy,"bgr8")
    self.pub_map.publish(img_msg)
  

  def show_map(self, empty):
    map_copy = self.map.copy()

    self.draw_map.update_odom_pix()
    self.draw_map.draw_robot(map_copy)

    img_msg = self.bridge.cv2_to_imgmsg(map_copy,"bgr8")
    self.pub_map.publish(img_msg)


class DrawMap():
   def __init__(self):
     # map features
     self.robot_color = (0,0,255)    # red
     self.points_color = (0,255,0)    # green
     self.line_color = (255,255,255) # white

     self.robot_radio = 0.18         # meters
     self.points_radio = 0.02
     self.resolution = 0.01

     # robot pose
     self.robot_x_pix = 0
     self.robot_y_pix = 0
     self.robot_ang = 0


   def update_odom_pix(self):
      odom_pix_data = rospy.wait_for_message('/odom_pix', Pose, timeout = 3)
      
      self.robot_x_pix = int(odom_pix_data.position.x)
      self.robot_y_pix = int(odom_pix_data.position.y)
      self.robot_ang = odom_pix_data.orientation.z


   def draw_robot(self, map): 
     robot_radio_pix = int(self.robot_radio / self.resolution)

     #get head point
     head_x = int(robot_radio_pix * np.cos(-self.robot_ang) + self.robot_x_pix)
     head_y = int(robot_radio_pix * np.sin(-self.robot_ang) + self.robot_y_pix)

     robot_pose_pix = tuple([self.robot_x_pix, self.robot_y_pix])
     head_pose = tuple([head_x, head_y])

     # Draw map
     cv2.circle(map, robot_pose_pix, robot_radio_pix, self.robot_color , -1)
     cv2.line(map, robot_pose_pix, head_pose,self.line_color ,2)
    

   def draw_point_cloud(self, map, points):
     point_radio_pix = int(self.points_radio/self.resolution)
     
     for point in points:
       x, y = int(point.x), int(point.y)
       cv2.circle(map, (x,y), point_radio_pix, self.points_color, -1)


if __name__ == '__main__':
  display_map = DisplayMap()