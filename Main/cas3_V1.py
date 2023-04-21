#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Apr 20 15:17:46 2023

@author: yannick
"""

#code du cas3

"""
Cas 3 : pas de panneau en face et ligne à gauche
    -> avancer gauche de 30 cm (asservissement)
    -> mesurer la distance à droite
    => si la distance > 50cm : droite avec b=0 --> Cas 0
    => Si la distance = d < 50 cm : droite avec b!=0 --> Cas 5
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
    if len(X)!=0:
        face = (X[round(len(X)/2)],Y[round(len(Y)/2)])
    else:
        face = (0,-440)
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

def avancer():
    print("avancer")
    #code à insérer
    
def gauche(parametre_virage):
    print("gauche a=",parametre_virage[0],"et b = ",parametre_virage[1])
    #code à insérer
    
def droite(parametre_virage):
    print("droite a=",parametre_virage[0],"et b = ",parametre_virage[1])
    #code à insérer
    
def d():
    print("distance parcourue par le robot obtenue avec l'accélerometre")
    
    
#code à insérer sous la condition du cas3
statistics = data_analysis()
avancer()
distance_parcourue = 0
d_droit = statistics["d_droit"]
if statistics["d_gauche"]>=50:
    parametre_virage = (0,1)
    c = 0
else :
    parametre_virage = (a,b) #à déterminer
    c = 5

while distance_parcourue < 30:
    statistics = data_analysis()
    distance_parcourue = d()

while statistics["d_gauche"] < 25:
    statistics = data_analysis()
    droite(parametre_virage)

cas = c