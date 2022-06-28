#!/usr/bin/env python3

import rospy
import tf
import sys
import time
import numpy as np
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud, Image
from geometry_msgs.msg import Twist, Pose, Point32
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float64
import cv2
from scipy import spatial, stats
from cv_bridge import CvBridge
from getpass import getuser


zhit = 1/19.947114020071634
zrandom = 0
zmax = 1
dist_zhit = stats.norm(loc = 0, scale = 0.02)

def view1D(a, b): # a, b are arrays
    a = np.ascontiguousarray(a)
    b = np.ascontiguousarray(b)
    void_dt = np.dtype((np.void, a.dtype.itemsize * a.shape[1]))
    A, B = a.view(void_dt).ravel(),  b.view(void_dt).ravel()
    return np.isin(A,B)

class Localizacion(object):

    def __init__(self, fov = 57):
        rospy.init_node('likelihood_fields', anonymous = True)
        self.bridge = CvBridge()
        rospy.Subscriber( '/map', OccupancyGrid, self.set_map)
        rospy.Subscriber('/odom', Pose, self.update_odometry)

        self.vel_pub = rospy.Publisher("/yocs_cmd_vel_mux/input/navigation", Twist, queue_size = 10)
        self.pub_pcl = rospy.Publisher('/lidar_points', PointCloud, queue_size=5)

        self.fov = fov # Grados

        self.min_depth = 0.0 # [m]
        self.limit_depth = 4.0 # [m]
        self.rate_pub = rospy.Rate(5) # 5 [Hz]

        self.x = 0
        self.y = 0
        self.yaw = 0

        self.linear_speed = 0.2 # [m/s]
        self.ang_speed = 0

        self.map = None
        # self.map = cv2.imread('rm-ws/src/lab-3/mapas/mapa.pgm', cv2.IMREAD_GRAYSCALE)

        # Verificamos si se ha actualizado la odometría
        self.updated_odom = False

        self.available = []
        self.occupied = []

        self.sample_pool = []
        self.sampled_indexes = []
        self.weights = []

        self.sampled_particles = []

        if self.map is not None:
            self.available_particles()

        self.run()
    
    def publish_point_cloud(self, points):
        point_cloud = PointCloud()
        for p in points:
            point = Point32()
            point.x = p[0]
            point.y = p[1]
            point_cloud.points.append(point)
            
        self.pub_pcl(point_cloud)

    def available_particles(self):
        if self.map is None:
            self.available = []
            self.weights = []
            self.occupied = []
            self.sample_pool = np.array(self.available)
            self.weights = np.array(self.weights)
        
        else:
            self.available = []
            self.weights = []
            self.occupied = []

            self.occupied = np.argwhere(self.map == 0)
            self.available = np.argwhere(self.map >= 254).astype(float)

            available_copy = self.available
            self.available = np.insert(self.available, 2, 0, axis=1)

            for i in [np.pi/2, np.pi, 3*np.pi/2]:
            # for i in [np.pi/4, np.pi/2, 3*np.pi/4, np.pi, 5*np.pi/4, 3*np.pi/2, 7*np.pi/4]:
                angle_i = np.insert(available_copy, 2, i, axis=1)
                self.available = np.vstack((self.available, angle_i))
            
            self.sample_pool = self.available
            self.weights = np.zeros(self.sample_pool.shape[0])
            self.weights[:] = 1/len(self.weights)
        return

    def update_particle_pool(self, dx, dy, dyaw):
        if len(self.sample_pool):
            self.sample_pool += np.array([dx, dy, dyaw])
            valid_indexes = view1D(self.sample_pool, self.available)

            self.weights = self.weights[valid_indexes]
            self.sample_pool = self.sample_pool[valid_indexes]
            return

    def generate_particles(self, n_particles = 400):
        self.weights = self.weights/self.weights.sum()
        self.sampled_indexes = np.random.choice((range(len(self.sample_pool))), n_particles, p = self.weights)
        self.sampled_particles = [self.available[s] for s in self.sampled_indexes]
        return

    def set_map(self, map):
        width = map.info.width
        height = map.info.height
        np_map = np.array(map.data)

        np_map = np_map.reshape( (height, width) )

        mapimg = 100 - np_map
        mapimg = ( mapimg * (255/100.0) ).astype(int)
        self.map = mapimg
        self.available_particles()
        print("MAP IS SET")

    def kd_tree(self, point):
        tree = spatial.KDTree(self.occupied)
        dist, _ = tree.query([point])
        return dist[0]

    def likelihood_fields_model(self, z, x):
        """
        Retorna la probabilidad de que la medición sea correcta con un valor entre [0, 1]

        z = Mediciones del sensor laser, dado un FOV para su rango, formato np.array()

        x = Lista que representa un vector desde el punto que fue
        lanzado el laser con dirección de lanzamiento, formato [x, y, yaw]
        """

        q = 1
        for k in range(len(z)):
            if z[k] < self.limit_depth:
                angulo_medicion = (self.fov/2 - k) * 180/np.pi
                x_medicion = x[0] + z[k] * np.cos(x[2] + angulo_medicion)
                y_medicion = x[1] + z[k] * np.sin(x[2] + angulo_medicion)

                x_medicion *= 100
                y_medicion *= 100

                dist = self.kd_tree([x_medicion, y_medicion])
                dist = dist/100
                q = q * (zhit * dist_zhit.pdf(dist) + zrandom/zmax)
        return q

    def update_odometry(self, odom):
      """ Actualiza la posición y orientación actual del Turtlebot dada la pose odom. """
      self.x = odom.position.x
      self.y = odom.position.y
      self.z = odom.position.z
      self.roll, self.pitch, self.yaw = tf.transformations.euler_from_quaternion((odom.orientation.x,
                                                                                  odom.orientation.y,
                                                                                  odom.orientation.z,
                                                                                  odom.orientation.w))
      
      self.updated_odom = True

    def run(self):
        movement_x = 0
        movement_y = 0
        movement_yaw = 0

        while not rospy.is_shutdown():
            if self.map is not None and len(self.sample_pool) > 0:
                prev_x = self.x
                prev_y = self.y
                prev_yaw = self.yaw

                speed = Twist()
                speed.linear.x = self.linear_speed
                speed.angular.z = self.ang_speed
                
                # Publicamos la velocidad deseada y esperamos a que pase el rate
                self.vel_pub.publish(speed)

                rospy.sleep(1)

                movement_x += self.x - prev_x
                movement_y += self.y - prev_y
                movement_yaw += self.yaw - prev_yaw

                dx = movement_x
                dy = movement_y
                dyaw = (movement_yaw // np.pi/4) * np.pi/4
                
                movement_x -= dx
                movement_y -= dy
                movement_yaw -= dyaw

                self.update_particle_pool(dx, dy, dyaw)
                self.generate_particles()

            z = [0.521]

            self.weights = np.zeros(len(self.sample_pool))
            for particle_index in range(len(self.sampled_indexes)):
                particle = self.sampled_particles[particle_index]
                # self.weights[self.sampled_indexes[particle_index]] = self.likelihood_fields_model(z, particle)
                self.weights[self.sampled_indexes[particle_index]] = np.random.uniform(0, 1)

            self.rate_pub.sleep()


if __name__ == '__main__':
    agente = Localizacion(fov = 0)
    z = [0.521]     # Distancia real: 0.52
    x = [1.18, 1.36, 0]
    import os
    print(agente.likelihood_fields_model(z, x))