#!/usr/bin/env python3

import rospy
import tf
import cv2
import numpy as np
 
from geometry_msgs.msg import Twist, PoseArray
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float64
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class NavegacionPasillo(object):

  def __init__(self):
    rospy.init_node('navegacion_pasillo', anonymous = True)
    self.bridge = CvBridge()
    self.depth_img_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_image_cb)
    self.fov = 57 # Grados

    self.min_depth = 0.0 # [m]
    self.limit_depth = 10.0 # [m]

    self.running = True

    self.hallway_width = 0.8 # [m]

    self.vel_pub = rospy.Publisher("/yocs_cmd_vel_mux/input/navigation", Twist, queue_size = 10)
    self.rate_pub = rospy.Rate(10) # 10 [Hz]

    self.odom_sub = rospy.Subscriber('/odom', Odometry, self.actualizar_posicion)

    # Control PID
    self.dist_setpoint_pub = rospy.Publisher("robot_ang/setpoint", Float64, queue_size = 10)
    self.dist_pub = rospy.Publisher("robot_ang/state", Float64, queue_size = 10)
    self.ang_control_effort_sub = rospy.Subscriber("robot_ang/control_effort", Float64, self.ang_control_effort_cb)

    self.dist_slack = 0.05 # [m]
    self._yaw = 0
    self.x = 0
    self.y = 0

    self.lwall_dist = 0
    self.rwall_dist = 0

    self.linear_speed = 0.2 # [m/s]
    self.ang_speed = 0

    self.depth_image_np = None

    # Verificamos si se ha actualizado la odometría
    self.updated_odom = False

    self.run()
 
  def ang_control_effort_cb(self, msg):
      """ Callback para el control PID del ángulo. """
      self.ang_speed = msg.data
      # print(f"Ang vel = {self.ang_speed}")
      # print(msg.data)
  
  # Métodos para el control PID setpoint y state
  def pub_dist_setpoint(self, setpoint):
      msg = Float64()
      msg.data = setpoint

      self.dist_setpoint_pub.publish(msg)
      # print("Published setpoint")

  def pub_dist_state(self):
      msg = Float64()
      msg.data = self.rwall_dist - self.lwall_dist

      self.dist_pub.publish(msg)
  
  def actualizar_posicion(self, odom):
      """ Actualiza la posición y orientación actual del Turtlebot dada la odometría odom. """
      self.x = odom.pose.pose.position.x
      self.y = odom.pose.pose.position.y
      self.z = odom.pose.pose.position.z
      self.roll, self.pitch, self.yaw = tf.transformations.euler_from_quaternion((odom.pose.pose.orientation.x,
                                                                                  odom.pose.pose.orientation.y,
                                                                                  odom.pose.pose.orientation.z,
                                                                                  odom.pose.pose.orientation.w))
      
      self.updated_odom = True
    
  def depth_image_cb( self, msg ):
    try:
        self.depth_image_np = self.bridge.imgmsg_to_cv2( msg )
        # Transformamos los nan a ceros (directamente frente al robot)
        self.find_wall_dist(np.nan_to_num(self.depth_image_np))
        
        img2display = self.depth_image_np - self.depth_image_np.min()
        img2display = ( img2display * (255.0/img2display.max())).astype(np.uint8)
        img2display = 255 - img2display
        img2display = cv2.flip(img2display, 0)
        cv2.imshow('Depth Sensor', img2display)
        cv2.waitKey( 1 )
    
    except CvBridgeError as e:
      rospy.logerr(e)

  def find_wall_dist(self, depth_image):
    if depth_image is not None:

      height, width = depth_image.shape

      wall_x = 4
      wall_skip = width//5 #recortando
      wall_y = int(height/2)
      # width -> self.fov
      # recortado width-ws -> x
      # x = (w-ws)*self.fov / w
      # a[s:s+w]
      # a[-w-s:-s]
      ang = (width-2*wall_skip)*self.fov / width

      lreading = sum(depth_image[wall_y, wall_skip:wall_skip+wall_x])/wall_x

      rreading = sum(depth_image[wall_y, -wall_x-wall_skip:-wall_skip])/wall_x

      if lreading < rreading:
        self.lwall_dist = min(lreading * np.cos(np.deg2rad(ang/2)), self.hallway_width)
        self.rwall_dist = max(self.hallway_width - self.lwall_dist, 0)
      
      else:
        self.rwall_dist = min(rreading * np.cos(np.deg2rad(ang/2)), self.hallway_width)
        self.lwall_dist = max(self.hallway_width - self.rwall_dist, 0)

      if self.running and (lreading == rreading == self.limit_depth or lreading == rreading == self.min_depth):
        self.running = False

      if not self.running and not(lreading == rreading == self.limit_depth or lreading == rreading == self.min_depth):
        self.running = True

      if self.running:
        self.pub_dist_setpoint(0)
        self.pub_dist_state()

      # print(f"La pared izquierda se encuentra a {self.lwall_dist} metros de distancia")
      # print(f"La pared derecha se encuentra a {self.rwall_dist} metros de distancia")

      # print(lreading)
      # print(rreading)

  def run(self):
    while not rospy.is_shutdown():
      if self.running:
        speed = Twist()
        speed.linear.x = self.linear_speed
        speed.angular.z = self.ang_speed
        
        # Publicamos la velocidad deseada y esperamos a que pase el rate
        self.vel_pub.publish(speed)
        self.rate_pub.sleep()

a = [1, 2, 3, 4, 5,6,7,8,9,10]
w = 2
s = 3
a[s:s+w]
a[-w-s:-s]

if __name__ == "__main__":
  nav = NavegacionPasillo()
  rospy.spin()
