#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 28 16:36:48 2023

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

def process_data(data):
    print(data)
    
freq = 360

scan_data = [0]*freq
#print(lidar.get_info())
x_data = np.arange(freq)


try:
    for scan in lidar.iter_scans():
        for (_, angle, distance) in scan:
            scan_data[min([freq-1, floor(angle)])] = distance
        process_data(scan_data)
        x = [-scan_data[i]*cos(x_data[i]*pi/180) for i in range(freq)]
        y = [scan_data[i]*sin(x_data[i]*pi/180) for i in range(freq)]

        dpoint = []
        for i in range(1,freq):
            d = sqrt((x[i]-x[i-1])**2 + (y[i]-y[i-1])**2)
            dpoint.append(d)
            
        n = 0
        line = []
        while n<len(dpoint):
            if dpoint[n]<100 and n<len(dpoint)-1:
                l = []
                while dpoint[n]<100 and n<len(dpoint)-1:
                    l.append((x[n],y[n]))
                    n += 1
                line.append(l)
            elif dpoint[n]>=500:
                n += 1
            else :
                n += 1
        for i in range(len(line)):
            if line[0][0] != (0.0,0.0):
                x = []
                y = []
                for j in range(0,len(line[i])):
                    x.append(round(line[i][j][0],0))
                    y.append(round(line[i][j][1],0))
                plt.plot(x,y,color="black")
                
        xlidar =  [0]
        ylidar = [0]
        plt.plot(xlidar,ylidar,color='red',marker='+',markersize=12)
        
        plt.xlim([-3000, 3000])
        plt.ylim([-3000, 3000])
        
        plt.show()

except KeyboardInterrupt:
    print('Stopping.')

    
lidar.stop()
lidar.disconnect()

scan_data[min([freq-1, floor(angle)])] = distance
