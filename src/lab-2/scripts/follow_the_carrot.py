#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import numpy as np
from nav_msgs.msg import Path
from getpass import getuser

dist_testing= False
ang_testing = False
rec_path = None

def get_dist(a, b):
    return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

class MovimientoTurtlebot():

    def __init__(self):
        rospy.init_node("dead_reckoning_nav", anonymous = True)
        self.vel_pub = rospy.Publisher("/yocs_cmd_vel_mux/input/navigation", Twist, queue_size = 10)
        self.rate_pub = rospy.Rate(10) # 10 [Hz]

        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.actualizar_posicion)

        # Para el control PID
        #ángulo
        self.ang_setpoint_pub = rospy.Publisher("robot_ang/setpoint", Float64, queue_size = 10)
        self.ang_pub = rospy.Publisher("robot_ang/state", Float64, queue_size = 10)
        self.ang_control_effort_sub = rospy.Subscriber("robot_ang/control_effort", Float64, self.ang_control_effort_cb)

        #Para la navegación:
        self.nav_sub = rospy.Subscriber("/nav_plan", Path, self.nav_cb, queue_size=100)

        # valore siniciales
        self.linear_speed_limit = 0.3 # [m/s]
        self.angular_speed_limit = 0.7 # [rad/s]
        self.dist_slack = 0.05 # [m]
        self.ang_slack = 0.015 # [rad]
        self._yaw = 0
        self.x = 0
        self.y = 0
        self.carrot = None
        self.pos_list = None
        self.history = []

        self.linear_speed = 0.1
        self.ang_speed = 0

        # Verificamos si se ha actualizado la odometría
        self.updated_odom = False

 
    def ang_control_effort_cb(self, msg):
        """ Callback para el control PID del ángulo. """
        self.ang_speed = msg.data
    
    def nav_cb(self, msg):
        pos_list = msg.poses
        
        self.pos_list = [[pose.pose.position.x, pose.pose.position.y] for pose in msg.poses]
        for pos in self.pos_list:
            print(f"                  ---------{pos}")

    #métodos para el control PID setpoint y state
    def pub_ang_setpoint(self, setpoint):
        msg = Float64()
        msg.data = setpoint
        if ang_testing: msg.data = np.pi/2 #Testing
        self.ang_setpoint_pub.publish(msg)

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
        if rec_path is not None:
            user = getuser()
            if not self.reached_goal:
                self.history.append([self.x, self.y])

    def find_carrot(self, pos_list, start=0):
        
        lookahead = 2 # metros
        dist_min = 100
        # 1) encontramos el punto más cercano a la posición actual del robot
        for pos in pos_list:
            dist = np.sqrt((pos[0] - self.x)**2 + (pos[1] - self.y)**2)
            if dist < dist_min:
                dist_min = dist
                pos_index = pos_list.index(pos)
        
        # 2)  colocamos la "zanahoria"  unas lookahead más adelante más adelante
        # supuse que 2 "metros" sería demasiado, pero parece que no. Quizá no son metros.
        
        # Vamos actualizando pos_prev para no volver a revisar posiciones anteriores de la trayectoria
        pos_prev = pos_list[pos_index]
        point_dist = 0
        self.carrot = pos_list[-1]

        # barremos la trayectoria hasta haber recorrido la distancia lookahead
        for pos in pos_list[pos_index:]:
            ddist = get_dist(pos_prev, pos)
            point_dist += ddist
            self.carrot = pos

            if point_dist >= lookahead:
                self.carrot = pos
                break

        if self.carrot == pos_list[-1]:
            self.reaching_goal = True
        # print(f"carrot = {self.carrot}")
        return 1


    def follow_carrot(self):
        self.reached_goal = False
        self.reaching_goal = False
        # pos_list = self.read_trayectory()

        i = 0
        while not self.reached_goal:
            
            if self.pos_list is None: continue
            pos_list = self.pos_list
            # colocamos la zanahoria
            self.find_carrot(pos_list)
            self.pub_ang_setpoint(0)
            
            #obtenemos el ángulo y distancia hasta la zanahoria
            ang, dist = self.angle_and_distance(self.carrot)
            rel_ang = (ang)-self.yaw
            
            # si nos acercamos mucho, cambiamos la zanahoria
            if dist <= self.dist_slack:
                
                if not self.reaching_goal: i = self.find_carrot(pos_list, i)
                else: self.reached_goal = True
            
            # publicamos al PID
            self.pub_ang_state(rel_ang)
            msg = Float64()
            msg.data = 0
            self.ang_setpoint_pub.publish(msg)

            # y publicamos la velocidad
            speed = Twist()
            speed.angular.z = -self.ang_speed
            speed.linear.x = self.linear_speed
            self.vel_pub.publish(speed)
            self.rate_pub.sleep()

        if rec_path is not None: # función para plotear el recorrido
            user = getuser()
            with open(f"/home/{user}/rm-ws/src/lab-2/scripts/followed_{rec_path}_path.txt", "w") as f:
                for pos in self.history:
                    f.write(f"{pos[0]}, {pos[1]}\n")
    
    def angle_and_distance(self, pos):
        rel_pos_x = pos[0] - self.x
        rel_pos_y = pos[1] - self.y

        angle_to_goal = np.arctan2(rel_pos_y, rel_pos_x)
        
        # 2. Avanzamos el Turtlebot hasta la posición (x, y) deseada
        distance = np.sqrt(rel_pos_x ** 2 + rel_pos_y ** 2)
        return angle_to_goal, distance

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
