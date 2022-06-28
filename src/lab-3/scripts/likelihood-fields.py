import rospy
import tf
import sys
import time
import numpy as np
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud, Image
import cv2
from scipy import spatial, stats
from cv_bridge import CvBridge

zhit = 1/19.947114020071634
zrandom = 0
zmax = 1
dist_zhit = stats.norm(loc = 0, scale = 0.02)

class Localizacion(object):

    def __init__(self, fov = 57):
        # rospy.init_node('localization', anonymous = True)
        self.bridge = CvBridge()
        # rospy.Subscriber( '/img_map', Image, self.set_map)
        self.fov = fov # Grados

        self.min_depth = 0.0 # [m]
        self.limit_depth = 4.0 # [m]

        self.x = 0
        self.y = 0
        self.yaw = 0

        self.linear_speed = 0.2 # [m/s]
        self.ang_speed = 0

        # self.map = None
        self.map = cv2.imread('rm-ws/src/lab-3/mapas/mapa.pgm', cv2.IMREAD_GRAYSCALE)

        # Verificamos si se ha actualizado la odometría
        self.updated_odom = False

    def set_map(self, msg):
        self.map = self.bridge.imgmsg_to_cv2(msg)
        return

    def kd_tree(self, point, mapimg):
        obstacle_coords = []
        for h in range( mapimg.shape[0] ):
            for w in range( mapimg.shape[1] ):
                if mapimg[h, w] == 0:
                    obstacle_coords.append([h, w])
        
        tree = spatial.KDTree(obstacle_coords)
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

                dist = self.kd_tree([x_medicion, y_medicion], self.map)
                dist = dist/100
                q = q * (zhit * dist_zhit.pdf(dist) + zrandom/zmax)
        return q


if __name__ == '__main__':
    agente = Localizacion(fov = 0)
    z = [0.521]     # Distancia real: 0.52
    x = [1.18, 1.36, 0]
    print(agente.likelihood_fields_model(z, x))