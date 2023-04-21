#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Apr 17 16:25:51 2023

@author: yannick
"""

#code du cas2

"""
Cas 2 : pas de panneau en face et ligne à droite
    -> avancer droit de 30 cm (asservissement)
    -> mesurer la distance à gauche
    => si la distance > 50cm : gauche avec a=0 --> Cas 1
    => Si la distance = d < 50 cm : gauche avec a!=0 --> Cas 4
"""

from adafruit_rplidar import RPLidar
from math import *
import numpy as np

PORT_NAME = '/dev/ttyUSB3'
lidar = RPLidar(None, PORT_NAME, timeout=3)
scan_data = [0]*360
x_data = np.arange(360)

def data_analysis():
    x = [-scan_data[i]*cos(x_data[i]*pi/180) for i in range(225,315)]
    y = [scan_data[i]*sin(x_data[i]*pi/180) for i in range(225,315)]
    X = []
    Y = []
    x_panneau = []
    y_panneau = []
    for i in range(len(x)):
        if sqrt(x[i]**2 + y[i]**2)<700 and y[i]!= 0:
            X.append(x[i])
            Y.append(y[i])
    face = (X[round(len(X)/2)],Y[round(len(Y)/2)])
    for i in range(len(X)):
        if Y[i]<face[1]+10 and Y[i]>face[1]-10:
            x_panneau.append(X[i])
            y_panneau.append(Y[i])
    bord_droit = (x_panneau[-1],y_panneau[-1])
    bord_gauche = (x_panneau[0],y_panneau[0])
    centre_panneau = ((bord_droit[0]+bord_gauche[0])/2,(bord_droit[1]+bord_gauche[1])/2)
    d_gauche = abs(bord_gauche[0]-face[0])
    d_droit = abs(bord_droit[0]-face[0])
    return {"face":face,"bord_droit":bord_droit,"bord_gauche":bord_gauche,"centre_panneau":centre_panneau,"d_gauche":d_gauche,"d_droit":d_droit}

#code à insérer sous la condition du cas2
statistics = data_analysis()
avancer()
distance_parcourue = 0
d_gauche = statistics["d_gauche"]
if d_gauche>=50:
    parametre_virage = (1,0)
    c = 1
else :
    parametre_virage = (a,b) #à déterminer
    c = 4

while distance_parcourue<300:
    statistics = data_analysis()
    distance_parcourue = d()

while statistics["d_droit"] < 250:
    statistics = data_analysis()
    gauche(parametre_virage)

cas = c