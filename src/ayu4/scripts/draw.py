# fuente: https://omes-va.com/funciones-dibujo/
# descargar opencv: pip3 install opencv-python
import cv2
import numpy as np

imagen = 255*np.ones((400,600,3),dtype=np.uint8)

# Dibujaos
# lineas: img, punto_1, punto_2, color(r,g,b), grosor)
cv2.line(imagen,(0,0),(600,400),(255,0,0),4)
cv2.line(imagen,(300,0),(300,200),(255,100,255),10)

# rectangulos: img, punto superior izquierdo, punto inferior derecho, color(r,g,b), valor positivo = grosor; -1 = relleno)
cv2.rectangle(imagen,(50,80),(200,200),(0,255,0),1)
cv2.rectangle(imagen,(300,80),(450,230),(0,0,0),-1)

# circulo: img, centro, radio, color(r,g,b), valor positivo = grosor; -1 = relleno)
cv2.circle(imagen,(300,200),100,(255,255,0),-1)
cv2.circle(imagen,(300,20),10,(255,0,255),3)

# texto:
font = cv2.FONT_HERSHEY_SIMPLEX
cv2.putText(imagen,'Practicando con OpenCV',(10,30),font,1,(0,255,255),2,cv2.LINE_AA)

cv2.imshow('imagen',imagen)
cv2.waitKey(0)
cv2.destroyAllWindows()

