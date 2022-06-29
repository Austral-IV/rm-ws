#!/usr/bin/env python3
# de la ayudantía
import rospy
import cv2
import numpy as np
from points2img import get_image2

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
    self.particles = None
    self.readings = None
    rospy.spin()
  

  def variables_init(self):
    self.draw_map = DrawMap()
    self.bridge = CvBridge()
    self.map = None
    self.point_cloud = None


  def connections_init(self):
    self.pub_map = rospy.Publisher('/img_map', Image, queue_size=10)
    rospy.Subscriber( '/map', OccupancyGrid, self.set_map)
    rospy.Subscriber( '/particle_filter', PointCloud, self.show_pointcloud)
    rospy.Subscriber( '/sensor_reading_best', PointCloud, self.show_pointcloud_sens)
    rospy.Subscriber( '/lidar_points_img', PointCloud, get_image2)
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
    self.particles = points
    map_copy = self.map.copy()

    self.draw_map.update_odom_pix()
    self.draw_map.draw_point_cloud(map_copy, points)
    if self.readings is not None:
      self.draw_map.draw_point_cloud_blue(map_copy, self.readings)
    self.draw_map.draw_robot(map_copy)

    img_msg = self.bridge.cv2_to_imgmsg(map_copy,"bgr8")
    self.pub_map.publish(img_msg)

  def show_pointcloud_sens(self, pointcloud):
    points = pointcloud.points
    self.readings = points
    map_copy = self.map.copy()

    self.draw_map.update_odom_pix()
    self.draw_map.draw_point_cloud_blue(map_copy, points)
    if self.particles is not None:
      self.draw_map.draw_point_cloud(map_copy, self.particles)
    self.draw_map.draw_robot(map_copy)

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

     self.robot_radio = 0.05         # meters
     self.points_radio = 0.02
     self.resolution = 0.01

     # robot pose
     self.robot_x_pix = 0
     self.robot_y_pix = 0
     self.robot_ang = 0


   def update_odom_pix(self):
      odom_pix_data = rospy.wait_for_message('/belief_pos', Pose, timeout = 10)
      
      self.robot_x_pix = int(odom_pix_data.position.x)
      self.robot_y_pix = int(odom_pix_data.position.y)
      self.robot_ang = odom_pix_data.orientation.z


   def draw_robot(self, map): 
     robot_radio_pix = int(self.robot_radio / self.resolution)

     robot_pose_pix = tuple([self.robot_x_pix, self.robot_y_pix])

     # Draw map
     cv2.circle(map, robot_pose_pix, robot_radio_pix, self.robot_color , -1)
    

   def draw_point_cloud(self, map, points):
     point_radio_pix = int(self.points_radio/self.resolution)
     
     for point in points:
       x, y = int(point.x), int(point.y)
       cv2.circle(map, (x,y), point_radio_pix, self.points_color, -1)

   def draw_point_cloud_blue(self, map, points):
     point_radio_pix = int(self.points_radio/self.resolution)
     
     for point in points:
       x, y = int(point.x), int(point.y)
       cv2.circle(map, (x,y), point_radio_pix, (255, 0, 0), -1)


if __name__ == '__main__':
  display_map = DisplayMap()