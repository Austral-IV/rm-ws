#!/usr/bin/env python3

import rospy
import tf
import sys
import time
import numpy as np
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud, LaserScan
from geometry_msgs.msg import Twist, Pose, Point32
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float64
import cv2
from scipy import spatial, stats
from cv_bridge import CvBridge
from getpass import getuser
import time


from matplotlib import pyplot as plt


zhit = 1/29.947114020071634
zrandom = 0
zmax = 1
dist_zhit = stats.norm(loc = 0, scale = 0.03)

NUM_PARTICLES = 400

if not NUM_PARTICLES % 2:
    NUM_PARTICLES += 1

class Localizacion(object):

    def __init__(self, move = True, fov = 57):
        rospy.init_node('likelihood_fields', anonymous = True)
        self.bridge = CvBridge()
        rospy.Subscriber( '/map', OccupancyGrid, self.set_map)
        rospy.Subscriber('/odom', Odometry, self.update_odometry)
        # rospy.Subscriber( '/lidar_points', PointCloud, self.update_readings)
        rospy.Subscriber('/scan', LaserScan, self.update_readings, queue_size=1)

        self.vel_pub = rospy.Publisher("/yocs_cmd_vel_mux/input/navigation", Twist, queue_size = 10)

        self.pub_pcl = rospy.Publisher('/particle_filter', PointCloud, queue_size=5)
        self.pub_sens = rospy.Publisher('/sensor_reading_best', PointCloud, queue_size=1)
        self.pub_pcl_img = rospy.Publisher('/lidar_points_img', PointCloud, queue_size=5)
        self.pub_init_pose = rospy.Publisher('/initial_pose', Pose, queue_size=1)
        self.pub_believe_pos = rospy.Publisher('/belief_pos', Pose, queue_size=1)
        
        self.fov = fov # Grados

        self.min_depth = 0.0 # [m]
        self.limit_depth = 4.0 # [m]
        self.rate_pub = rospy.Rate(5) # 5 [Hz]

        self.init_x = 0.5
        self.init_y = 0.5

        self.x = 0
        self.y = 0
        self.yaw = 0

        self.previous_odom = [0, 0, 0]

        self.linear_speed = 0.2 # [m/s]
        self.ang_speed = 0.5

        self.resolution = 0.01
        self.map = None
        # self.map = cv2.imread('rm-ws/src/lab-3/mapas/mapa.pgm', cv2.IMREAD_GRAYSCALE)

        # Verificamos si se ha actualizado la odometría
        self.updated_odom = False

        self.reading = []

        self.consistency = 0
        self.cluster_coords = [0, 0]

        self.tree = None
        self.occupied_pix = None
        self.available = None
        self.sample_points = None
        self.weights = np.full(NUM_PARTICLES, 1 / NUM_PARTICLES)

        self.localized = False

        self.move = move
        # self.set_init_pose()
        self.run()
 
    def timer_st(self):self.time = time.time()
    def timer_end(self, msg=None): print(msg, time.time() - self.time)

    def set_init_pose(self):
        rospy.sleep(2)
        
        init_pose = Pose()
        init_pose.position.x = self.init_x 
        init_pose.position.y = self.init_y 

        x,y,z,w = tf.transformations.quaternion_from_euler(0, 0, 0)
        
        init_pose.orientation.x = x
        init_pose.orientation.y = y
        init_pose.orientation.z = z
        init_pose.orientation.w = w

        self.pub_init_pose.publish(init_pose)
        rospy.sleep(2)

    def update_readings(self, scan):        
        self.reading = scan
    
    def publish_point_cloud(self, points):
        point_cloud = PointCloud()
        for p in points:
            point = Point32()
            point.x = p[0] * 100
            point.y = p[1] * 100
            point_cloud.points.append(point)
            
        self.pub_pcl.publish(point_cloud)

    def publish_sensor_cloud(self, points):
        points = points.tolist()
        point_cloud = PointCloud()
        for p in points:
            point = Point32()
            point.x = p[0]
            point.y = p[1]
            point_cloud.points.append(point)

        self.pub_sens.publish(point_cloud)

    def send_best(self, best_pred, sensors = False):
        b_pos = Pose()
        b_pos.position.x, b_pos.position.y = best_pred[0] * 100, best_pred[1] * 100
        b_pos.orientation.x, b_pos.orientation.y, b_pos.orientation.z, b_pos.orientation.w = tf.transformations.quaternion_from_euler(0, 0, best_pred[2])
        self.pub_believe_pos.publish(b_pos)

        if sensors:
            z = self.reading
            useless = len(z.ranges) - self.fov
            ranges = z.ranges[useless//2:-useless//2]
            num_angles = len(z.ranges) - 2 * (useless//2)
            angles = np.linspace(-27 * np.pi/180, 27 * np.pi/180, num_angles)    # angle_min = -fov/2, angle_max = fov/2

            px = np.array(ranges) * np.cos(best_pred[2] - angles)
            py = np.array(ranges) * np.sin(best_pred[2] - angles)

            x_medicion = (100 * (best_pred[0] + px)).astype(int)[:, None]
            y_medicion = (100 * (best_pred[1] + py)).astype(int)[:, None]

            self.publish_sensor_cloud(np.concatenate([x_medicion, y_medicion], axis = 1))
    
    def update_odometry(self, odom):
        """ Actualiza la posición y orientación actual del Turtlebot dada la pose odom. """
        self.x = odom.pose.pose.position.x
        self.y = odom.pose.pose.position.y
        self.z = odom.pose.pose.position.z
        self.roll, self.pitch, self.yaw = tf.transformations.euler_from_quaternion((odom.pose.pose.orientation.x,
                                                                                    odom.pose.pose.orientation.y,
                                                                                    odom.pose.pose.orientation.z,
                                                                                    odom.pose.pose.orientation.w))
        
        self.updated_odom = True

    def set_map(self, map):
        width = map.info.width
        height = map.info.height

        np_map = np.array(map.data)
        np_map = np_map.reshape((height, width))
        np_map = np.swapaxes(np_map, 0, 1)
        mapimg = 100 - np_map
        mapimg = (mapimg/100 * 255).astype(int)

        self.detect_available(mapimg)
        self.map = mapimg

    def detect_available(self, map):
        global NUM_PARTICLES
        self.occupied_pix = np.argwhere(map == 0)
        self.tree = spatial.KDTree(self.occupied_pix)

        self.available = np.argwhere(map == 255)/100

        chosen_indexes = np.random.choice(range(self.available.shape[0]), NUM_PARTICLES)
        # chosen_points = np.array([self.available[i] for i in chosen_indexes])
        chosen_points = self.available[chosen_indexes]

        self.sample_points = np.append(chosen_points, np.random.uniform(-2 * np.pi, 2 * np.pi, chosen_points.shape[0])[:, None], axis = 1)
        # self.sample_points = np.append(chosen_points, np.random.normal(0, np.pi, chosen_points.shape[0])[:, None], axis = 1)

    def compute_weights(self):
        n_valid_particles = self.sample_points.shape[0]
        # indexes = np.arange(0, n_valid_particles)
        
        if self.reading is not None:
            for particle_index in range(n_valid_particles):
                particle = self.sample_points[particle_index]
                self.weights[particle_index] = self.likelihood_fields_model(self.reading, particle)
            
            # particles = self.sample_points[indexes] Cómo aplicar likelihood_fields_model al array completo?
            # self.weights[indexes] = self.likelihood_fields_model(self.reading, particle)
            
        else:
            self.weights = np.full(n_valid_particles, 1 / n_valid_particles)
        return self.weights

    def generate_particles(self):
        self.weights = self.weights/self.weights.sum()
        self.weights = np.nan_to_num(self.weights)
        sample_range = NUM_PARTICLES if self.sample_points.shape[0] > NUM_PARTICLES else self.sample_points.shape[0]
        random_fraction = 1/20
        n_chosen = int(NUM_PARTICLES * (1 - random_fraction))

        chosen_indexes = np.random.choice(range(sample_range), n_chosen, p = self.weights)
        chosen_points = self.sample_points[chosen_indexes]
        # chosen_points = np.array([self.sample_points[i] for i in chosen_indexes])

        best_pred = self.sample_points[np.argmax(self.weights)]
        self.send_best(best_pred, sensors = False)

        if np.std(chosen_points[:, 0]) < 0.028 and np.std(chosen_points[:, 1]) < 0.028:
            if np.sqrt((self.cluster_coords[0] - np.mean(chosen_points[:, 0]))**2 + (self.cluster_coords[1] - np.mean(chosen_points[:, 1]))**2) < 0.1:
                self.consistency += 1
                if self.consistency == 3:
                    print("EL ROBOT SE HA LOCALIZADO")
                    self.localized = True
            else:
                self.consistency = 0
            self.cluster_coords = [np.mean(chosen_points[:, 0]), np.mean(chosen_points[:, 1])]
        else:
            self.consistency = 0

        random_indexes = np.random.choice(range(self.available.shape[0]), NUM_PARTICLES - n_chosen)
        # random_points = np.array([self.available[i] for i in random_indexes])
        random_points = self.available[random_indexes]
        random_points = np.append(random_points, np.random.uniform(-2 * np.pi, 2 * np.pi, random_points.shape[0])[:, None], axis = 1)

        chosen_points = np.append(chosen_points, random_points, axis = 0)

        noise = np.array([np.random.normal(0, 0.02, chosen_points.shape[0]),
                        np.random.normal(0, 0.02, chosen_points.shape[0]),
                        np.random.normal(0, np.pi/8, chosen_points.shape[0])])
        noise = np.swapaxes(noise, 0, 1)

        # self.sample_points = chosen_points
        self.sample_points = np.add(chosen_points, noise)
        self.weights = np.full(NUM_PARTICLES, 1 / NUM_PARTICLES)

    def move_samples(self, dx, dy, dyaw):
        original_movement_list = [np.full(self.sample_points.shape[0], dx), np.full(self.sample_points.shape[0], dy), np.full(self.sample_points.shape[0], dyaw)]
        particle_movement = np.array([original_movement_list[0] * np.cos(self.sample_points[:, 2]) + original_movement_list[1] * np.sin(self.sample_points[:, 2]),
                                    original_movement_list[0] * -np.sin(self.sample_points[:, 2]) + original_movement_list[1] * np.cos(self.sample_points[:, 2]),
                                    original_movement_list[2]])
        particle_movement = np.swapaxes(particle_movement, 0, 1)
        
        noise = np.array([np.random.normal(0, 0.03, particle_movement.shape[0]),
                        np.random.normal(0, 0.03, particle_movement.shape[0]),
                        np.random.normal(0, np.pi/32, particle_movement.shape[0])])
        noise = np.swapaxes(noise, 0, 1)

        noisy_movement = np.add(particle_movement, noise)
        # self.sample_points = np.add(self.sample_points, particle_movement)
        self.sample_points = np.add(self.sample_points, noisy_movement)

    def remove_invalids(self):
        int_pool = (self.sample_points * 100).astype(int)
        valid_indexes = array_isin(int_pool[:, :2], (self.available * 100).astype(int))

        self.sample_points = self.sample_points[valid_indexes]
        self.weights = self.weights[valid_indexes]

    def likelihood_fields_model(self, z, x):
        """
        Retorna la probabilidad de que la medición sea correcta con un valor entre [0, 1]
        z = Mediciones del sensor laser, dado un FOV para su rango, formato np.array()
        x = Lista que representa un vector desde el punto que fue
        lanzado el laser con dirección de lanzamiento, formato [x, y, yaw]
        """

        q = 1

        angle_max = self.fov/2 * np.pi/180
        angle_min = -self.fov/2 * np.pi/180

        useless = len(z.ranges) - self.fov
        ranges = np.array(z.ranges[useless//2:-useless//2])

        mask = ranges < 4.0
        ranges = ranges[mask]

        num_angles = len(z.ranges) - 2 * (useless//2)
        angles = np.linspace(angle_min, angle_max, num_angles)[mask]    # angle_min = -fov/2, angle_max = fov/2

        px = ranges * np.cos(x[2] - angles)
        py = ranges * np.sin(x[2] - angles)

        x_medicion = (100 * (x[0] + px)).astype(int)[:, None]
        y_medicion = (100 * (x[1] + py)).astype(int)[:, None]

        dist = np.array([self.kd_tree(coords.tolist()) for coords in np.concatenate([x_medicion, y_medicion], axis = 1)])
        dist /= 100

        q = zhit * dist_zhit.pdf(dist) + zrandom/zmax
        # q = np.array([zhit * dist_zhit.pdf(x) + zrandom/zmax for x in dist])
        return 1 * np.prod(q)


    def kd_tree(self, point):
        dist, _ = self.tree.query([point])
        # print(f"El punto más cercano a {point} es {self.occupied_pix[a]}")
        return dist[0]

    def run(self):
        while not rospy.is_shutdown() and not self.localized:
            if self.map is not None and len(self.sample_points) > 0 and self.updated_odom:

                self.publish_point_cloud(self.sample_points)

                if self.move and self.map is not None:
                    speed = Twist()
                    if self.reading.ranges[len(self.reading.ranges)//2] < 0.6:
                        speed.linear.x = 0
                        speed.angular.z = self.ang_speed
                    else:
                        speed.linear.x = self.linear_speed
                        speed.angular.z = 0
                    
                    # Publicamos la velocidad deseada y esperamos a que pase el rate
                    self.vel_pub.publish(speed)
                    rospy.sleep(1)

                delta_x = self.x - self.previous_odom[0]
                delta_y = self.y - self.previous_odom[1]
                delta_yaw = self.yaw - self.previous_odom[2]

                self.timer_st()
                self.move_samples(delta_x, delta_y, delta_yaw)      # AVANZAMOS LOS PUNTOS DEL FILTRO, MÁS UN CIERTO RUIDO
                self.remove_invalids()                              # REMOVEMOS TODA PARTICULA EN UNA POSICION INVALIDA
                self.compute_weights()                              # PARA TODOS LOS PUNTOS DEL FILTRO, CALCULAMOS EL PESO EN BASE A LA LECTURA ACTUAL
                self.generate_particles()                           # RESAMPLEAMOS LOS PUNTOS DEL FILTRO
                self.remove_invalids()
                self.timer_end()

            self.previous_odom = [self.x, self.y, self.yaw]
            self.rate_pub.sleep()

def array_isin(a, b): # a, b are arrays
    a = np.ascontiguousarray(a)
    b = np.ascontiguousarray(b)
    void_dt = np.dtype((np.void, a.dtype.itemsize * a.shape[1]))
    A, B = a.view(void_dt).ravel(),  b.view(void_dt).ravel()
    return np.isin(A,B)

if __name__ == '__main__':
    agente = Localizacion(move = True)


import numpy as np
n_chosen = 5
NUM_PARTICLES = 10
random_indexes = np.random.choice(range(10), NUM_PARTICLES - n_chosen)
available = np.arange(1, 20)
random_points = np.array([available[i] for i in random_indexes])
random_points = available[random_indexes]

random_points = np.append(random_points, np.random.uniform(-2 * np.pi, 2 * np.pi, random_points.shape[0])[:, None], axis = 1)