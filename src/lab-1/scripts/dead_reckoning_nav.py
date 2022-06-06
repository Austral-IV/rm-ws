#!/usr/bin/env python3

from curses import raw
import rospy
import tf
from geometry_msgs.msg import Twist, PoseArray
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
import numpy as np
import time

class MovimientoTurtlebot():

    def __init__(self):
        rospy.init_node("dead_reckoning_nav", anonymous = True)
        self.vel_pub = rospy.Publisher("/yocs_cmd_vel_mux/input/navigation", Twist, queue_size = 10)
        self.tts_pub = SoundClient(blocking = True, sound_topic = 'robotsound')
        self.rate_pub = rospy.Rate(10) # 10 [Hz]

        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.actualizar_posicion)
        self.mover_sub = rospy.Subscriber('/goal_list', PoseArray, self.accion_mover_cb)
        self.obstacle_sub = rospy.Subscriber('/occupancy_state', String, self.obstaculo_detectado)

        self.linear_speed = 0.2 # [m/s]
        self.angular_speed = 1.0 # [rad/s]

        # Verificamos si se ha actualizado la odometría
        self.updated_odom = False

        # Verificador de si el robot se encuentra realizando una rutina de trayectorias
        self.moving = False

        # Verificador si el robot tiene un obstáculo en frente
        self.pause = False

        # Verificador si se ha notificado un obstáculo en x dirección
        self.notif = {}
        self.notif["obstacle_left"] = False
        self.notif["obstacle_center"] = False
        self.notif["obstacle_right"] = False

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

    def aplicar_velocidad(self, vel_lineal, vel_angular, tiempo):
        """ Para cada indice i en el rango len(vel_lineal) = len(vel_angular) = len(tiempo) otorga al turtlebot
        la velocidad lineal vel_lineal[i] y velocidad angular vel_angular[i] durante tiempo[i] segundos. """

        for i in range(len(vel_lineal)):
            # Actualizamos cual es el proceso actual
            self.current_process = i

            # Caso borde
            if tiempo[i] == 0:
                continue

            start_time = time.time()
            remaining_time = tiempo[i] * 1.09

            while time.time() - start_time < remaining_time:

                if self.pause and vel_lineal[i] != 0:
                    remaining_time = tiempo[i] - (time.time() - start_time)

                    while self.pause:
                        start_time = time.time()
                        continue

                # Creamos el objeto con la velocidad deseada
                speed = Twist()
                speed.linear.x = vel_lineal[i]
                speed.angular.z = vel_angular[i]
                
                # Publicamos la velocidad deseada y esperamos a que pase el rate
                self.vel_pub.publish(speed)
                self.rate_pub.sleep()

    def mover_robot_a_destino(self, goal_pose):
        """Lleva al Turtlebot a la posicion (goal_pose[-1][0], goal_pose[-1][1]) con un angulo
        de orientacion goal_pose[-1][2], pasando por cada uno de los puntos de la lista. """

        while not self.updated_odom:
            continue
            
        # Declarar que el robot si se está moviendo
        self.moving = True

        print(f"Pose inicial: ({self.x}, {self.y}, {self.yaw})")
        # print("")
        for pose in goal_pose:
            print("..")
            # Creamos las listas con las direcciones para el Turtlebot
            move_linear = [0, 0, 0]
            move_angular = [0, 0, 0]
            move_time = [0, 0, 0]

            # 1. Orientamos el Turtlebot a la posición (x, y) que se desea llegar
            # Posiciones relativas de la pose objetivo a la posición actual
            rel_pos_x = pose[0] - self.x
            rel_pos_y = pose[1] - self.y

            angle_to_goal = np.arctan2(rel_pos_y, rel_pos_x)

            # Angulo hacia el goal relativo al actual
            rel_angle = angle_to_goal - self.yaw
            move_time[0] = abs(rel_angle/self.angular_speed)

            if rel_angle > 0:
                move_angular[0] = self.angular_speed

            elif rel_angle < 0:
                move_angular[0] = -self.angular_speed

            # 2. Avanzamos el Turtlebot hasta la posición (x, y) deseada
            # Ángulo objetivo relativo al actual (tras haber mirado hacia el punto)
            distance = np.sqrt(rel_pos_x ** 2 + rel_pos_y ** 2)
            move_linear[1] = self.linear_speed
            move_time[1] = abs(distance/self.linear_speed)

            # 3. Orientamos el Turtlebot al ángulo theta deseado
            rel_angle = pose[2] - angle_to_goal
            move_time[2] = abs(rel_angle/self.angular_speed)

            if rel_angle > 0:
                move_angular[2] = self.angular_speed

            elif rel_angle < 0:
                move_angular[2] = -self.angular_speed

            # Movemos al Turtlebot
            print(move_linear, move_angular, move_time)
            self.aplicar_velocidad(move_linear, move_angular, move_time)

        print(f"Pose final: ({self.x}, {self.y}, {self.yaw})")
    
        self.moving = False

    def accion_mover_cb(self, pose_array):

        # print(pose_array)

        while self.moving:
            continue

        move_poses = [[0, 0, 0]] * len(pose_array.poses)

        for i in range(len(pose_array.poses)):
            pose = pose_array.poses[i]

            new_pose = []
            new_pose.append(pose.position.x)
            new_pose.append(pose.position.y)
            _, _, yaw = tf.transformations.euler_from_quaternion((pose.orientation.x,
                                                                        pose.orientation.y,
                                                                        pose.orientation.z,
                                                                        pose.orientation.w))

            new_pose.append(yaw)
            move_poses[i] = new_pose

        print(move_poses)

        self.mover_robot_a_destino(move_poses)

    def obstaculo_detectado(self, raw_reading):

        reading = raw_reading.data
        # rospy.loginfo(reading)
        if reading == "free":
            self.pause = False
            for msg in self.notif.keys():
                self.notif[msg] = False

        else:
            self.pause = True
            obstacles = reading.split(",")
            say_str = "obstacle "
            for obs in obstacles:
                if obs == "":
                    continue
                b = obs.replace("obstacle_", "") 
                say_str += f"{b}, "

            if obs != "" and not self.notif[obs]:
                msg = SoundRequest(arg = say_str)
                self.tts_pub.say(say_str, "voice_kal_diphone")
                
                self.notif[obs] = True

            for msg in self.notif.keys():
                if msg not in obstacles:
                    self.notif[msg] = False

if __name__ == "__main__":
    nav = MovimientoTurtlebot()
    rospy.spin()
