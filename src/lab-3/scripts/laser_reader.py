#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
class LaserReader():
    """
    Clase que lee y almacena la información del tópico /scan.
    
    """
    def __init__(self):
        laser_sub = rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        self.angle_range = 57 * np.pi/180 # Por enunciado e sel rango de grados útil.

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

        ranges = info.ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)
        intensities = info.intensities
        point_count = len(ranges)
        
        self.ranges = []
        self.intensities = []

        # Seleccionamos los valores de rango y intensidad que nos interesan (entre +- 57 grados)
        if np.abs(angle_min) > self.angle_range or np.abs(angle_max) > self.angle_range:
            ang = angle_min
            k = 0
            # asumiendo que el rango de ángulos obtenido siempre será mayor que +-57:
            # tomamos desde -57° hasta +57°
            min_index = int((np.abs(angle_min)) / angle_increment) - int(self.angle_range / angle_increment)
            max_index = int((np.abs(angle_min)) / angle_increment) + int(self.angle_range / angle_increment)
            self.ranges = ranges[min_index:max_index]
            self.intensities = intensities[min_index:max_index]

            # creo que lo de arriba logra lo mismo que esto, pero el de arriba debe ser más rápido
            # for i in range(point_count):
            #     if abs(ang) > self.angle_range: 
            #         self.ranges[k] = ranges[i]
            #         self.intensities[k] = intensities[i]
            #     k += 1
            #     ang += angle_increment

# import numpy as np
# print(np.zeros(2))