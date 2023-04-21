#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 11 15:55:47 2023

@author: yannick
"""

import math

def teta(line,n):
    init = [(line[n][0][0],line[n][0][1]),(line[n][1][0],line[n][1][1])]
    teta = []
    for j in range(1,len(line[n])):
        ang = math.degrees(math.atan2(line[n][j][1]-line[n][1][1], line[n][j][0]-line[n][1][0]) - math.atan2(line[n][0][1]-line[n][1][1], line[n][0][0]-line[n][1][0]))
        teta.append(ang + 360 if ang < 0 else ang)
    
    return teta

def linear(line,n):
    init = [(line[n][0][0],line[n][0][1]),(line[n][1][0],line[n][1][1])]
    teta = []
    line_streak = []
    
    for j in range(1,len(line[n])):
        ang = math.degrees(math.atan2(line[n][j][1]-line[n][1][1], line[n][j][0]-line[n][1][0]) - math.atan2(line[n][0][1]-line[n][1][1], line[n][0][0]-line[n][1][0]))
        teta.append(ang + 360 if ang < 0 else ang)
        
    for k in range(0,len())