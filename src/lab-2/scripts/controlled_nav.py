#!/usr/bin/env python3

from curses import raw
import rospy
import tf
from geometry_msgs.msg import Twist, PoseArray
from std_msgs.msg import String, Float64
from nav_msgs.msg import Odometry
import numpy as np

dist_testing= False
ang_testing = False

class MovimientoTurtlebot():

    def __init__(self):
        rospy.init_node("dead_reckoning_nav", anonymous = True)
        self.vel_pub = rospy.Publisher("/yocs_cmd_vel_mux/input/navigation", Twist, queue_size = 10)
        self.rate_pub = rospy.Rate(10) # 10 [Hz]

        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.actualizar_posicion)
        self.mover_sub = rospy.Subscriber('/goal_list', PoseArray, self.accion_mover_cb)

        # Para el control PID
        #Distancia
        self.dist_setpoint_pub = rospy.Publisher("robot_dist/setpoint", Float64, queue_size = 10)
        self.dist_pub = rospy.Publisher("robot_dist/state", Float64, queue_size = 10)
        self.dist_control_effort_sub = rospy.Subscriber("robot_dist/control_effort", Float64, self.dist_control_effort_cb)
        #ángulo
        self.ang_setpoint_pub = rospy.Publisher("robot_ang/setpoint", Float64, queue_size = 10)
        self.ang_pub = rospy.Publisher("robot_ang/state", Float64, queue_size = 10)
        self.ang_control_effort_sub = rospy.Subscriber("robot_ang/control_effort", Float64, self.ang_control_effort_cb)


        self.linear_speed_limit = 0.3 # [m/s]
        self.angular_speed_limit = 0.7 # [rad/s]
        self.dist_slack = 0.05 # [m]
        self.ang_slack = 0.015 # [rad]
        self._yaw = 0

        self.linear_speed = 0
        self.ang_speed = 0

        # Verificamos si se ha actualizado la odometría
        self.updated_odom = False

        # Verificador de si el robot se encuentra realizando una rutina de trayectorias
        self.moving = False
 
    def dist_control_effort_cb(self, msg):
        """ Callback para el control PID de la dist. """
        self.linear_speed = msg.data
        # print(msg.data)
    def ang_control_effort_cb(self, msg):
        """ Callback para el control PID del ángulo. """
        self.ang_speed = msg.data
        # print(msg.data)
    
    #métodos para el control PID setpoint y state
    def pub_dist_setpoint(self, setpoint):
        msg = Float64()
        msg.data = setpoint
        if dist_testing: msg.data = 0
        self.dist_setpoint_pub.publish(msg)
    def pub_ang_setpoint(self, setpoint):
        msg = Float64()
        msg.data = setpoint
        if ang_testing: msg.data = np.pi/2 #Testing

        self.ang_setpoint_pub.publish(msg)
    def pub_ang_state(self, angle):
        msg = Float64()
        msg.data = angle
        self.ang_pub.publish(msg)
    def pub_dist_state(self, dist):
        msg = Float64()
        msg.data = dist
        self.dist_pub.publish(msg)
   
    def giro_controlado_v4(self, angulo_objetivo):  
        # controla la rotación del bot usando el controlador PID de ROS
        # Toma el ángulo entre 0 y 360 pero, si estamos cerca de la división entre
        # 0 y 360, cambiamos a verlo entre -180 y 180, para que el ocntrolador 
        # no vea un error exagerado  
        print("girando controladamente")
        
        #Parámetros iniciales
        cambiar_referencia = False #originalmente se usa entre 0 y 360
        angulo_objetivo = self.filter_angle(angulo_objetivo) # pasamos a 0_2pi
        angulo_objetivo_og = angulo_objetivo
        angulo_anterior = self.yaw
        self.pub_ang_setpoint(angulo_objetivo) #publicamos el setpoint
        # self.ang_setpoint_pub.publish(angulo_objetivo) #publicamos el setpoint

        # Mientras la diferencia entre el ángulo actual y el objetivo sea mayor que el slack, 
        # ajustamos.
        ang_actual = self.yaw
        # while np.abs(self.yaw - angulo_objetivo_og) > self.ang_slack:
        while np.abs(angulo_objetivo - ang_actual) > self.ang_slack or ang_testing:
            speed = Twist()
            speed.angular.z = self.ang_speed 
            ang_actual = self.yaw
            # Revisamos si deberíamos cambiar de referencia
            if not cambiar_referencia:
                if np.abs(angulo_anterior - self.yaw) > np.pi: # si hubo un salto muy brusco en ang
                    cambiar_referencia = True # apuntando en eje x a la derecha, por lo que se cambia la referencia
                elif (angulo_objetivo_og > 3*np.pi/2 and self.yaw < np.pi/2):
                    # para evitar que de una vuelta en 270° en lugar de una de 90° también 
                    # podemos cambiar la referencia
                    cambiar_referencia = True

            if cambiar_referencia: # pasamos de vuelta a -pi_pi
                ang_actual = self.anti_filter_angle(self.yaw)
                angulo_objetivo = self.anti_filter_angle(angulo_objetivo)
                self.pub_ang_setpoint(angulo_objetivo)
                # self.ang_setpoint_pub.publish(angulo_objetivo)

            # print(f"{angulo_objetivo} - {ang_actual}  = {angulo_objetivo - ang_actual}")
            
            self.pub_ang_state(ang_actual)
            # self.ang_pub.publish(ang_actual) # publicamos el estado de la planta

            # Publicamos la velocidad deseada y esperamos a que pase el rate
            self.vel_pub.publish(speed)
            self.rate_pub.sleep() 

        print(f"error = {angulo_objetivo - ang_actual}")
        print(f"error = {self.yaw - angulo_objetivo_og}")
        print("girado")

    def desplazamiento_controlado(self, dist):
            # print(f"distancia: {dist} m")
            speed = Twist()
            self.dist_pub.publish(dist)
            
            speed.linear.x = -self.linear_speed # como el setpoint es 0, el error es negativo
            # speed.angular.z = vel_angular[i]
            
            # Publicamos la velocidad deseada y esperamos a que pase el rate
            self.vel_pub.publish(speed)
            self.rate_pub.sleep()        

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

            # 1. Orientamos el Turtlebot a la posición (x, y) que se desea llegar
            # Posiciones relativas de la pose objetivo a la posición actual
            rel_pos_x = pose[0] - self.x
            rel_pos_y = pose[1] - self.y

            angle_to_goal = np.arctan2(rel_pos_y, rel_pos_x)

            self.giro_controlado_v4(angle_to_goal)
            
            # 2. Avanzamos el Turtlebot hasta la posición (x, y) deseada
            self.dist_setpoint_pub.publish(0) #el objetivo es una llegar a dist de 0 +-0.05m
            distance = np.sqrt(rel_pos_x ** 2 + rel_pos_y ** 2)
            while np.abs(distance) > self.dist_slack or dist_testing:
                rel_pos_x = pose[0] - self.x
                rel_pos_y = pose[1] - self.y
                distance = np.sqrt(rel_pos_x ** 2 + rel_pos_y ** 2)
                angle_to_goal = np.arctan2(rel_pos_y, rel_pos_x)
                
                # en caso de que el robot se pase, la distancia será negativa
                rel_angle = self.anti_filter_angle(angle_to_goal - self.anti_filter_angle(self.yaw))
                robot_se_pasa = np.abs(rel_angle) >= 2*np.pi/3
                if robot_se_pasa: 
                    distance = -distance
                    print(self.anti_filter_angle(angle_to_goal - self.anti_filter_angle(self.yaw)))

                self.desplazamiento_controlado(distance)
            
            # print(f"distancia con el punto: {dist} m")
                


            # 3. Orientamos el Turtlebot al ángulo theta deseado
            self.giro_controlado_v4(pose[2])

            

        print(f"Pose final: ({self.x}, {self.y}, {self.yaw})")
    
        self.moving = False

    def accion_mover_cb(self, pose_array):

        # print(pose_array)
        while dist_testing: self.mover_robot_a_destino([[1, 0, 0]])
        
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
    print("Starting")
    nav = MovimientoTurtlebot()
    rospy.spin()
