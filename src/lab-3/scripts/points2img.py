#!/usr/bin/env python3
import numpy as np
import cv2
from getpass import getuser
import rospy

user = getuser()
img_loc = f"/home/{user}/rm-ws/src/lab-3/scripts/"

def get_image2(point_cloud):
    """ similar a get_image pero utiliza la info rnviada por el nodo 
    display_map y funciona mejor. Cambiar wall_width ajusta el tamaño
    de los cuadrados generados"""
    points = point_cloud.points
    # rospy.loginfo("halo")
    zmax = 4
    pxperm = 100
    wall_width = 5
    angle_limit = 57
    max_width = int(2*zmax*pxperm*np.sin(angle_limit*np.pi/180))
    max_height = zmax*pxperm
    max_vision = [max_height, max_width] # el espacio 
                                # que puede ver el sensor laser
    og = [0, max_width//2] # el origen; las coord del bot en la img
        
    img_array = np.zeros(max_vision)
    
    for point in points:
        # dibujamos las paredes que cree ver el robot
        # paint_circle(img_array, i, og, 10)
        x, y = int(point.x), int(point.y)    
        paint_box(img_array, [x, y + og[1]], og, wall_width)
    
    if True: cv2.imwrite(f'{img_loc}/sensor_img.jpg', img_array*255)
    return img_array

def get_image(ranges, angle_limit = 57, pxperm=100, write_img=True):
    """ recibe un np.array, entrega un np.array que contiene 0s 
    excepto donde "ranges" indica que hay pared. También guarda 
    opcionalmente el resultado como una imágen: sensor_img.jpg"""
    wall_width = 5
    angles = np.arange(-angle_limit, angle_limit + 2*angle_limit / len(ranges),
                    2*angle_limit / len(ranges))*np.pi/180

    # max_x = np.fix(ranges.max()*pxperm)
    # max_ylow = abs(np.sin(angles[0]) *  ranges[0] * pxperm)
    # max_yhigh = abs(np.sin(angles[-1]) *  ranges[-1] * pxperm)
    # width = int(np.fix((max_ylow + max_yhigh)))
    # og = np.array([width//2, 0]) # el origen; las coord del bot
    # img_array = np.zeros([int(max_x) + wall_width, width + wall_width])

    zmax = 4
    max_width = int(2*zmax*pxperm*np.sin(angle_limit*np.pi/180))
    max_height = zmax*pxperm
    max_vision = [max_width, max_height] # el espacio 
                                # que puede ver el sensor laser
    og = [max_width//2, 0] # el origen; las coord del bot en la img
        
    img_array = np.zeros(max_vision)
    
    indexes = []
    
    for r,t in zip(ranges, angles):
        x = r * np.cos(t) * pxperm; y = r * np.sin(t) * pxperm
        indexes.append( [int(y) + og[0], int(x)]) # *
        
    for i in indexes:
        # dibujamos las paredes que cree ver el robot
        # paint_circle(img_array, i, og, 10)
        paint_box(img_array, i, og, wall_width)
    
    if write_img: cv2.imwrite(f'{img_loc}/sensor_img.jpg', img_array*255)
    return img_array

def paint_box(arr, coord, og, size=5): 
    """ dibuja una caja. Ojo que no está centrada en la coordenada, si no
    que la dibuja alejándose del origen og """
    x, y = coord
    dx = (-1) ** (x < og[0])
    dy = (-1) ** (y < og[1])
    topaint = [[x],[y]]

    
    for _ in range(size):
        y_iter = y
        for _ in range(size):
            topaint[1].append(y_iter)
            topaint[0].append(x)
            y_iter += dy
        x += dx

    for x, y in zip(topaint[0], topaint[1]): # no funciona?
        if y < 0 or x < 0: continue
        try:
            arr[x, y] = 1
        except IndexError: continue

def paint_circle(arr, coord, og = [0,0], diam=5): 
    """ dibuja una círculo. Ojo que no está centrado en la coordenada, si no
    que lo dibuja alejándose del origen og """
    x, y = coord
    dx = (-1) ** (x < og[0])
    dy = (-1) ** (y < og[1])
    topaint = [[x],[y]]

    circ_centre = [dx * (x+diam/2), dy * (y+diam/2)]
    for _ in range(diam): # cambiar a circ
        y_iter = y
        for _ in range(diam):
            if abs((x-circ_centre[0])**2 + (y_iter-circ_centre[1])**2) <= (diam/2)**2:
                topaint[1].append(y_iter)
                topaint[0].append(x)
            y_iter += dy
        x += dx

    for x, y in zip(topaint[0], topaint[1]): # no funciona?
        if y < 0 or x < 0: continue
        try:
            arr[x, y] = 1
        except IndexError:
            # print(f"index {[x, y]} is out of range")
            continue
        
    # arr[topaint] = 1

"""     it = np.nditer(img, flags=['multi_index'])
    for x in it:
        print("%d <%s>" % (x, it.multi_index), end=' ')
"""    

if __name__ == "__main__":
    whole_scan = np.array((4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 0.6447733044624329, 0.6588694453239441, 0.7021180391311646, 0.6856034994125366, 0.7042323350906372, 0.748081386089325, 0.7749257683753967, 0.7863368988037109, 0.8676528334617615, 0.8797444105148315, 0.9350643754005432, 0.996394693851471, 1.0389155149459839, 1.1391164064407349, 1.2054017782211304, 1.3177130222320557, 1.428375005722046, 1.5215823650360107, 1.684959888458252, 1.8602752685546875, 2.0032408237457275, 1.9930235147476196, 2.001223087310791, 1.9797776937484741, 1.9633065462112427, 2.0146634578704834, 1.9883993864059448, 1.9432657957077026, 1.9648109674453735, 2.002964973449707, 1.9943315982818604, 2.0179038047790527, 1.9992074966430664, 1.968610167503357, 2.002018690109253, 2.0383243560791016, 1.993588924407959, 2.0217769145965576, 2.0245561599731445, 1.986923098564148, 1.9312995672225952, 1.746825098991394, 1.6026893854141235, 1.5106457471847534, 1.4331568479537964, 1.3171223402023315, 1.25045907497406, 1.1794075965881348, 1.1162046194076538, 1.097705364227295, 1.035810947418213, 1.0215920209884644, 0.9696410298347473, 0.9017016291618347, 0.9165634512901306, 0.8745397925376892, 0.8276211619377136, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0))
    real_scan2 = whole_scan[np.where(whole_scan < 4, True, False)]
    len(real_scan2)
    real_scan = np.array((0.6493123769760132, 0.6286704540252686, 0.6799650192260742, 0.7138640880584717, 0.715155303478241, 0.7510764002799988, 0.7658082246780396, 0.8652892112731934, 0.873450756072998, 0.8964787721633911, 0.9316491484642029, 1.002589225769043, 1.059494972229004, 1.135742425918579, 1.210590124130249, 1.3140610456466675, 1.4401755332946777, 1.5159786939620972, 1.6763763427734375, 1.901537537574768, 1.9928878545761108, 1.9724998474121094, 2.0162594318389893, 2.0015673637390137, 1.9725977182388306, 2.057797431945801, 1.9705857038497925, 1.9975489377975464, 1.985300898551941, 2.008363723754883, 1.9978258609771729, 1.9880261421203613, 1.9672825336456299, 2.0047030448913574, 2.009394884109497, 2.0135343074798584, 2.033346652984619, 2.00101375579834, 2.012737274169922, 2.0469400882720947, 1.8997715711593628, 1.7996971607208252, 1.6094024181365967, 1.5130031108856201, 1.441062092781067, 1.3197970390319824, 1.2173092365264893, 1.2177634239196777, 1.1724059581756592, 1.1214100122451782, 1.0396032333374023, 1.0060086250305176, 0.9997872114181519, 0.9469492435455322, 0.8766757249832153, 0.8617734909057617, 0.8378868103027344))
    

    img = get_image(real_scan2)


    # paint_box(img, [0,0], [0, 0], 2); cv2.imwrite('color_img.jpg', img*255)
    # cv2.imshow("image", img);cv2.waitKey(10000);cv2.destroyAllWindows() 
