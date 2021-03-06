#!/usr/bin/env python3
# de la ayudantía
# fuente: https://omes-va.com/funciones-dibujo/
# descargar opencv: pip3 install opencv-python

import rospy
import numpy as np
from points2img import get_image
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32, Pose


class Scan2PointCloud():

    def __init__(self):
        rospy.init_node('scan2pointcloud')
        self.variables_init()
        self.connections_init()
        rospy.spin()
    
    def variables_init(self):
         # map features
        self.map_height = 270
        self.map_width = 270
        self.resolution = 0.01

        # robot pose
        self.robot_x_pix = 0.0
        self.robot_y_pix = 0.0
        self.robot_ang = 0.0


    def connections_init(self):
        self.pub_pcl_img = rospy.Publisher('/lidar_points_img', PointCloud, queue_size=5)
        rospy.Subscriber('/scan', LaserScan, self.laser_scan_hd, queue_size=1)


    def laser_scan_hd(self, scan):
        
        self.update_odom_pix()
        
        num_angles = int((scan.angle_max-scan.angle_min)/scan.angle_increment)
        angles = np.linspace(scan.angle_min, scan.angle_max, num_angles)

        point_clould = PointCloud()
        point_clould_img = PointCloud()

        for z_ang, zk in zip(angles, scan.ranges):
            if zk >= 4: continue

            zx_pix = zk/self.resolution
            global_ang = -(z_ang)
            limited = True
            px_pix = int( zx_pix * np.cos(global_ang))
            py_pix = int( zx_pix * np.sin(global_ang))

            px_in_range = 0 < py_pix < self.map_height 
            py_in_range = 0 < px_pix < self.map_width

            if (px_in_range and py_in_range and zk < scan.range_max) or limited:
                point = Point32()
                point.x, point.y = px_pix, py_pix 
                point_clould_img.points.append(point)

        self.pub_pcl_img.publish(point_clould_img)
        

    def update_odom_pix(self):
      odom_pix_data = rospy.wait_for_message('/odom_pix', Pose, timeout = 3)
      
      self.robot_x_pix = int(odom_pix_data.position.x)
      self.robot_y_pix = int(odom_pix_data.position.y)
      self.robot_ang = odom_pix_data.orientation.z

if __name__ == '__main__':
    scan2cloud = Scan2PointCloud()