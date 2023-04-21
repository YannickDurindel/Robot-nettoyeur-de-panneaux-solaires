#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Apr 14 11:35:03 2023

@author: yannick
"""

import os
from adafruit_rplidar import RPLidar
import time
import matplotlib.pyplot as plt
import numpy as np
from math import *


# Setup the RPLidar
PORT_NAME = '/dev/ttyUSB4'
lidar = RPLidar(None, PORT_NAME, timeout=3)

# used to scale data to fit on the screen
max_distance = 0


scan_data = [0]*360
#print(lidar.get_info())
x_data = np.arange(360)
n = 0


try:
    for scan in lidar.iter_scans():
        for (_, angle, distance) in scan:
            scan_data[min([359, floor(angle)])] = distance
        
        xlidar =  [0]
        ylidar = [0]
        plt.plot(xlidar,ylidar,color='red',marker='+',markersize=12)
        
        x = [-scan_data[i]*cos(x_data[i]*pi/180) for i in range(225,315)]
        y = [scan_data[i]*sin(x_data[i]*pi/180) for i in range(225,315)]
        
        X = []
        Y = []
        for i in range(len(x)):
            if sqrt(x[i]**2 + y[i]**2)<700 and y[i]!= 0:
                X.append(x[i])
                Y.append(y[i])
        #face = (X[round(len(X)/2)],Y[round(len(Y)/2)])
        face = (0,-440) 
        
        x_panneau = []
        y_panneau = []
        for i in range(len(X)):
            if Y[i]<face[1]+10 and Y[i]>face[1]-10:
                x_panneau.append(X[i])
                y_panneau.append(Y[i])
        
        bord_droit = (x_panneau[-1],y_panneau[-1])
        bord_gauche = (x_panneau[0],y_panneau[0])
        centre_panneau = ((bord_droit[0]+bord_gauche[0])/2,(bord_droit[1]+bord_gauche[1])/2)
        d_gauche = abs(bord_gauche[0]-face[0])
        d_droit = abs(bord_droit[0]-face[0])
        
        print("centre = ",face)
        print("gauche",bord_gauche)
        print("droit",bord_droit)
        print("distance à gauche =",d_gauche)
        print("distance à droite",d_droit,"\n")
        
        plt.scatter(x_panneau,y_panneau,s=5)
        plt.title('Lidar Data Plot')
        plt.xlabel('X - Distance (mm)')
        plt.ylabel('Y - Distance (mm)')
        plt.show()


except KeyboardInterrupt:
    print('Stopping.')
    
lidar.stop()
lidar.disconnect()