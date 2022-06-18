#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

class KinectkTrimmer(object):
  """ Código para recortar la imagen del kinect tal que no se vean los obstáculos
  Puede que sea inutil? creo que en realidad el profe nos entrega algo para esto
  
  En todo caso, es una base; no lo he trabajado mucho"""

  def __init__(self):
    rospy.init_node('obstacle_detector' , anonymous = True)

    self.bridge = CvBridge()
    self.depth_img_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_image_cb)
    self.detector_pub = rospy.Publisher("/occupancy_state", String, queue_size=1)
    self.depth_image_np = None

    # Resolución de detección de obstáculos. Menor resolución reduce la velocidad de cada iteración
    self.obst_res = 10
    
    # luminosidad de pizel de obstáculo
    self.obst_brightness = 0.5

    self.rate = rospy.Rate(5) # 5 Hz

  def detector(self, depth_image):
    """ Permite detectar obstáculos en tres direcciones con respecto al robot dada una imagen de profundidad"""

    if depth_image is None:
        pass

    height, width= depth_image.shape

    obstacle_x = [] 
    obstacle_y = int(height/2)

    # Buscamos en el punto medio de la vista de nuestro robot si existe
    # algún obstáculo a menos de la distancia mínima aceptada
    for x in range(0, width, self.obst_res):   
        if depth_image[obstacle_y, x]  <= self.obst_brightness:
          obstacle_x.append(x)      
    
    right = left = centre = False
    
    # Vemos su ubicación
    for x in obstacle_x:
        if x <= width//3:
            left = True

        elif x >= 2* width//3:
            right = True

        else:
            centre = True

    obst_str = ",".join([left * "obstacle_left", centre * "obstacle_center", right * "obstacle_right"])

    if not (right or centre or left):
      obst_str = "free"
    rospy.loginfo(obst_str)
    self.detector_pub.publish(obst_str)
    self.rate.sleep()
  
  def depth_image_cb( self, msg ):

    try:
        self.depth_image_np = self.bridge.imgmsg_to_cv2( msg )
        # Transformamos los nan a ceros (directamente frente al robot)
        self.detector(np.nan_to_num(self.depth_image_np))
        
        img2display = self.depth_image_np - self.depth_image_np.min()
        img2display = ( img2display * (255.0/img2display.max())).astype(np.uint8)
        # img2display = 
        img2display = 255 - img2display
        img2display = cv2.flip(img2display, 0)
        cv2.imshow('Depth Sensor', img2display)
        cv2.waitKey( 1 )
    
    except CvBridgeError as e:
      rospy.logerr(e)


if __name__ == "__main__":
    trimmer = KinectkTrimmer()
    rospy.spin()