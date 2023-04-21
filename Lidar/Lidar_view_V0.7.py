#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 11 16:22:16 2023

@author: yannick
"""

import os
from adafruit_rplidar import RPLidar
import time
import matplotlib.pyplot as plt
import numpy as np
from math import *
import math
from scipy import stats


# Setup the RPLidar
PORT_NAME = '/dev/ttyUSB4'
lidar = RPLidar(None, PORT_NAME, timeout=3)

# used to scale data to fit on the screen
max_distance = 0

def process_data(data):
    print(data)
    
def myfunc(x):
  return slope * x + intercept
    
def teta(line,n):
    init = [(line[n][0][0],line[n][0][1]),(line[n][1][0],line[n][1][1])]
    teta = []
    for j in range(1,len(line[n])):
        ang = math.degrees(math.atan2(line[n][j][1]-line[n][1][1], line[n][j][0]-line[n][1][0]) - math.atan2(line[n][0][1]-line[n][1][1], line[n][0][0]-line[n][1][0]))
        teta.append(ang + 360 if ang < 0 else ang)
    return teta
    
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
            if dpoint[n]<300 and n<len(dpoint)-1:
                l = []
                while dpoint[n]<300 and n<len(dpoint)-1:
                    l.append((x[n],y[n]))
                    n += 1
                if len(l)>2:
                    line.append(l)
            elif dpoint[n]>=300:
                n += 1
            else :
                n += 1
        for i in range(len(line)):
            if line[0][0] != (0.0,0.0):
                x = []
                y = []
                for j in range(0,len(line[i])):
                    if len(line[i])>5:
                        x.append(round(line[i][j][0],0))
                        y.append(round(line[i][j][1],0))
                        
                if x != [] and y != []:
                    slope, intercept, r, p, std_err = stats.linregress(x, y)
                    mymodel = list(map(myfunc, x))
                    #plt.scatter(x, y,color='black',marker="x")
                    plt.plot(x, mymodel,color='black')
                
            print("\n teta = ")
            print(teta(line,i))
            print("\n")
                
        xlidar =  [0]
        ylidar = [0]
        plt.plot(xlidar,ylidar,color='red',marker='+',markersize=12)
        
        plt.xlim([-500, 500])
        plt.ylim([-600, 100])
        
        plt.show()

except KeyboardInterrupt:
    print('Stopping.')

    
lidar.stop()
lidar.disconnect()

scan_data[min([freq-1, floor(angle)])] = distance