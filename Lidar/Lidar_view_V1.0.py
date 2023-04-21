#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 11 16:59:55 2023

@author: yannick
"""

import os
from adafruit_rplidar import RPLidar
import time
import matplotlib.pyplot as plt
import numpy as np
from math import *


# Setup the RPLidar
PORT_NAME = '/dev/ttyUSB3'
lidar = RPLidar(None, PORT_NAME, timeout=3)

# used to scale data to fit on the screen
max_distance = 0

def process_data(data):
    print(data)

scan_data = [0]*360
#print(lidar.get_info())
x_data = np.arange(360)
n = 0


try:
    for scan in lidar.iter_scans():
        for (_, angle, distance) in scan:
            scan_data[min([359, floor(angle)])] = distance
        process_data(scan_data)
        
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
                
        plt.scatter(X,Y,s=5)
        plt.title('Lidar Data Plot')
        plt.xlabel('X - Distance (mm)')
        plt.ylabel('Y - Distance (mm)')
        plt.show()


except KeyboardInterrupt:
    print('Stopping.')
    
lidar.stop()
lidar.disconnect()
