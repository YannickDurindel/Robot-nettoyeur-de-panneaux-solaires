#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr  4 15:27:55 2023

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
        
        x = [-scan_data[i]*cos(x_data[i]*pi/180) for i in range(360)]
        y = [scan_data[i]*sin(x_data[i]*pi/180) for i in range(360)]
        
        x_near = []
        y_near = []
        for i in range(50):
            if sqrt(x[i]**2+y[i]**2)<1000:
                x_near.append(x[i])
                y_near.append(y[i])
        for i in range(50):
            if sqrt(x[-i]**2+y[-i]**2)<1000:
                x_near.append(x[-i])
                y_near.append(y[-i])
        
        plt.scatter(x[-50:-1]+x[0:50],y[-50:-1]+y[0:50],s=5)
        plt.title('Lidar Data Plot')
        plt.xlabel('X - Distance (mm)')
        plt.ylabel('Y - Distance (mm)')
        plt.xlim([-1000, 100])
        plt.ylim([-1000, 1000])
        plt.show()


except KeyboardInterrupt:
    print('Stopping.')
    
lidar.stop()
lidar.disconnect()

scan_data[min([359, floor(angle)])] = distance
