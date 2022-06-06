#!/usr/bin/env python3

from curses import raw
import rospy
import tf
from geometry_msgs.msg import Twist, PoseArray
from std_msgs.msg import String, Float64
from nav_msgs.msg import Odometry
import numpy as np
from getpass import getuser


dist_testing= False
ang_testing = False

class MovimientoTurtlebot():

    def __init__(self):
        rospy.init_node("dead_reckoning_nav", anonymous = True)
        self.vel_pub = rospy.Publisher("/yocs_cmd_vel_mux/input/navigation", Twist, queue_size = 10)
        self.rate_pub = rospy.Rate(10) # 10 [Hz]

        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.actualizar_posicion)

        # Para el control PID
        #ángulo
        # self.ang_setpoint_pub = rospy.Publisher("robot_ang/setpoint", Float64, queue_size = 10)
        # self.ang_pub = rospy.Publisher("robot_ang/state", Float64, queue_size = 10)
        # self.ang_control_effort_sub = rospy.Subscriber("robot_ang/control_effort", Float64, self.ang_control_effort_cb)


        self.linear_speed_limit = 0.3 # [m/s]
        self.angular_speed_limit = 0.7 # [rad/s]
        self.dist_slack = 0.05 # [m]
        self.ang_slack = 0.015 # [rad]
        self._yaw = 0
        self.x = 0
        self.y = 0

        self.linear_speed = 0.1
        self.ang_speed = 0

        # Verificamos si se ha actualizado la odometría
        self.updated_odom = False

        # Verificador de si el robot se encuentra realizando una rutina de trayectorias
        self.moving = False
 
    def ang_control_effort_cb(self, msg):
        """ Callback para el control PID del ángulo. """
        self.ang_speed = msg.data
        # print(msg.data)
    
    #métodos para el control PID setpoint y state
    def pub_ang_setpoint(self, setpoint):
        msg = Float64()
        msg.data = setpoint
        if ang_testing: msg.data = np.pi/2 #Testing
        self.ang_setpoint_pub.publish(msg)
        print("published setpoint")

    def pub_ang_state(self, angle):
        msg = Float64()
        msg.data = angle
        self.ang_pub.publish(msg)
    
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

    def find_carrot(self, pos_list, start=0):

        # 1) encontramos el punto más cercano a la posición actual del robot
        self.carrot = [self.x, self.y]
        lookahead = 10
        dist_min = 100
        for pos in pos_list:
            dist = np.sqrt((pos[0] - self.x)**2 + (pos[1] - self.y)**2)
            if dist < dist_min:
                dist_min = dist
                pos_index = pos_list.index(pos)
        
        # 2)  colocamos la "zanahoria"  unas posiciones más adelante
        try:
           self.carrot = pos_list[pos_index + lookahead + start]
        except IndexError:
            # estamos llegando al final de la trayectoria
            self.carrot = pos_list[-1]
            self.reaching_goal = True
        print(f"carrot = {self.carrot}")

    def follow_carrot(self):
        self.reached_goal = False
        self.reaching_goal = False
        pos_list = self.read_trayectory()
        self.find_carrot(pos_list)
        # self.pub_ang_setpoint(0)

        i = 0
        while not self.reached_goal:
            ang, dist = self.angle_and_distance(self.carrot)
            rel_ang = (ang)-self.yaw
            if dist <= self.dist_slack:
                i += 1
                if not self.reaching_goal: self.find_carrot(pos_list, i)
                else: self.reached_goal = True
            
            # self.pub_ang_state(rel_ang)
            # msg = Float64()
            # msg.data = 0
            # if ang_testing: msg.data = np.pi/2 #Testing
            # self.ang_setpoint_pub.publish(msg)
            # print(f"{ang} - {self.yaw} = {rel_ang}")
            effort = self.control_P(self.yaw, ang)
            print(effort)
            speed = Twist()
            speed.angular.z = effort
            speed.linear.x = self.linear_speed
            self.vel_pub.publish(speed)
            self.rate_pub.sleep()

    def read_trayectory(self):
        user = getuser()
        pose_file = f"/home/{user}/rm-ws/src/lab-2/scripts/path_sin.txt"
        # pose_file = f"/home/{user}/rm-ws/src/lab-2/scripts/path_line.txt"
        # pose_file = f"/home/{user}/rm-ws/src/lab-2/scripts/path_sqrt.txt"
        with open(pose_file) as f:
            lines = f.readlines()
            pose_list = [pose.strip().split(",") for pose in lines]
            for pose in pose_list:
                pose[0] = float(pose[0]) - 1
                pose[1] = float(pose[1]) - 1
        return pose_list
        
    def angle_and_distance(self, pos):
        rel_pos_x = pos[0] - self.x
        rel_pos_y = pos[1] - self.y

        angle_to_goal = np.arctan2(rel_pos_y, rel_pos_x)
        
        # 2. Avanzamos el Turtlebot hasta la posición (x, y) deseada
        distance = np.sqrt(rel_pos_x ** 2 + rel_pos_y ** 2)
        return angle_to_goal, distance

    def control_P(self, angle, setpoint):
        Kp = 0.65
        if setpoint > angle: 
            angle_dif = min(setpoint - angle, 2*np.pi - (setpoint - angle))
            if angle_dif == 2*np.pi - (setpoint - angle): angle_dif = -angle_dif # básicamente 350° equivale a -10°
        if setpoint < angle: 
            angle_dif = max(setpoint - angle,  - 2*np.pi - (setpoint - angle))
            if angle_dif == -2*np.pi - (setpoint - angle): angle_dif = -angle_dif # básicamente -350° equivale a 10°
        
        effort = Kp * angle_dif
        if abs(effort) > self.angular_speed_limit:
            effort = self.angular_speed_limit * np.sign(effort)
        return effort

    def filter_angle(self, angle):
        # para pasar el ángulo de -pi_pi a 0_2pi
        if angle >= 2*np.pi:
            angle -= 2 * np.pi
        elif angle < 0:
            angle += 2 * np.pi
        return angle
    def anti_filter_angle(self, angle):
        if angle > np.pi:
            angle -= 2 * np.pi
        return angle

    @property # Property 
    def yaw(self):
        return self._yaw
    @yaw.setter
    def yaw(self, ang):
        self._yaw = self.filter_angle(ang)

if __name__ == "__main__":
    print("Starting follow the carrot")
    nav = MovimientoTurtlebot()
    nav.follow_carrot()
    rospy.spin()
