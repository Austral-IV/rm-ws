#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
from points2img import get_image

class LaserReader():
    """
    Clase que lee y almacena la información del tópico /scan.
    
    """
    def __init__(self):
        rospy.init_node("laser_reader", anonymous=True)
        laser_sub = rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        self.angle_range = 57 * np.pi/180 # Por enunciado es el rango de grados
        self.resolution=0.01              # útil.
        self.zmax = 4
        distances = int(self.zmax/self.resolution)
        self.prior = np.ones(distances)/distances

    def laser_range_filter(self, prior, ranges, range_accuracy = 0.9):
        # NO FUNCIONA
        X = len(prior)
        T = len(ranges)
        Z = np.ones(X)*range_accuracy
        px_u = np.zeros(X)

        #no hay movimiento, por lo que p(x|u,xt-1) = 1
        # bel_bar = np.sum(prior)*prior
        # # hacer tq pz_x
        # #nu = 1/range_acc*la distancia medida + (1-r_acc*el resto)
        # nu = 1/(range_accuracy*prior[int(np.fix(ranges[T-1]))] + np.sum((1-range_accuracy)*prior) 
        #     - (1-range_accuracy)*prior[int(np.fix(ranges[T-1]))])

        # belt = nu*Z*bel_bar
        belt = []
        for t in range(T):
            #no hay movimiento, por lo que p(x|u,xt-1) = 1
            print(prior)
            bel_bar = np.sum(prior)*prior[t]
            # hacer tq pz_x
            #nu = 1/range_acc*la distancia medida + (1-r_acc*el resto)
            nu = 1/(range_accuracy*prior[int(np.fix(ranges[t]))] + np.sum((1-range_accuracy)*prior) 
                - (1-range_accuracy)*prior[int(np.fix(ranges[t]))])

            belt += nu*range_accuracy*bel_bar
            # prior = belt

        return belt

    def laser_callback(self, info):
        """ información del mensaje """
        angle_min = info.angle_min        # start angle of the scan [rad]
        angle_max = info.angle_max        # end angle of the scan [rad]
        angle_increment = info.angle_increment  # angular distance between measurements [rad]

        time_increment = info.time_increment  # time between measurements [seconds] - if your scanner
                                              # is moving, this will be used in interpolating position
                                              # of 3d points
        scan_time = info.scan_time        # time between scans [seconds]

        range_min = info.range_min        # minimum range value [m]
        range_max = info.range_max        # maximum range value [m]

        ranges = np.array(info.ranges)         # range data [m] (Note: values < range_min or > range_max should be discarded)
        intensities = info.intensities
        point_count = len(ranges)
        
        self.ranges = []
        self.intensities = []
        self.angles = []

        # Seleccionamos los valores de rango y intensidad que nos interesan (entre +- 57 grados)
        if np.abs(angle_min) > self.angle_range or np.abs(angle_max) > self.angle_range:
            # asumiendo que el rango de ángulos obtenido siempre será mayor que +-57:
            # tomamos desde -57° hasta +57°
            min_index = int((np.abs(angle_min)) / angle_increment) - int(self.angle_range / angle_increment)
            max_index = int((np.abs(angle_min)) / angle_increment) + int(self.angle_range / angle_increment)
            self.ranges = ranges[min_index:max_index]
            self.ranges = ranges[np.where(ranges != 4.0, True, False)]
            self.intensities = intensities[min_index:max_index]
            self.angles = np.arange(angle_min, angle_max+angle_increment, angle_increment)
        
        # self.prior = self.laser_range_filter(self.prior, self.ranges)
        # rospy.loginfo(self.prior)
        
        get_image(self.ranges)
        # rospy.loginfo(angle_increment)
        # rospy.loginfo(57*2*np.pi/180 // angle_increment)
        # rospy.loginfo(len(self.ranges))
            # creo que lo de arriba logra lo mismo que esto, pero el de arriba debe ser más rápido
            # ang = angle_min
            # k = 0
            # for i in range(point_count):
            #     if abs(ang) > self.angle_range: 
            #         self.ranges[k] = ranges[i]
            #         self.intensities[k] = intensities[i]
            #     k += 1
            #     ang += angle_increment

# import numpy as np
# print(np.zeros(2))
if __name__ == "__main__":
    LR = LaserReader()
    rospy.spin()