#!/usr/bin/env python3
# de la ayudantía
import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class OdomPixel():
  
  def __init__(self):
    rospy.init_node( 'odom_pix' )
    self.variables_init()
    self.connections_init()
    rospy.spin()
  

  def variables_init(self):
    self.resolution = 0.01
    self.map_height = 270
     
    # init pose
    self.init_x = 0.5
    self.init_y = 0.5


  def connections_init(self):
    self.pub_odom_pix = rospy.Publisher('/odom_pix', Pose, queue_size=1)
    rospy.Subscriber('/odom', Odometry, self.odom_pix)


  def odom_pix(self, odom_data):
      pose_data = odom_data.pose.pose
      
      robot_x_pix = int((pose_data.position.x + self.init_x) / self.resolution)
      
      robot_y_pix = self.map_height - int((pose_data.position.y + self.init_y)/self.resolution)
      
      quaternion = (pose_data.orientation.x,
                    pose_data.orientation.y,
                    pose_data.orientation.z,
                    pose_data.orientation.w)

      row, pitch, yaw = euler_from_quaternion(quaternion)

      robot_ang = yaw

      pose_pix = Pose()

      pose_pix.position.x = robot_x_pix
      pose_pix.position.y = robot_y_pix
      pose_pix.orientation.z = robot_ang

      self.pub_odom_pix.publish(pose_pix)


if __name__ == '__main__':
  odom_pixel = OdomPixel()