#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Apr 20 14:48:00 2023

@author: yannick
"""

#code du cas4

"""
Cas 5 : Bas du panneau et dernier virage à droite
    -> Avancer tant que l'on voit la panneau
    -> Quand on ne voit plus de panneau : avancer de 30cm + gauche/2
    -> arreter les brosses et pompe
    -> avancer vers le haut jusqu'au bord + 30cm
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

#code à insérer sous la condition du cas4
avancer()   #fonctionner avancer à déterminer
statistics = data_analysis()
distance_parcourue = 0
d_gauche = statistics["d_gauche"]
parametre_virage = (1,0)

while statistics["face"][1]<430:   #tant qu'il y a le panneau devant (à une distance de 430mm)
    statistics = data_analysis()
    if d_gauche >25:
        gauche()    #fonction à déterminer
    elif d_gauche<25:
        droite()
        
while distance_parcourue < 300:
    statistics = data_analysis()
    distance_parcourue = d()
    
while statistics["d_droite"] < 250:
    statistics = data_analysis()
    gauche(parametre_virage)
    
while statistics["face"][1]<430:   #tant qu'il y a le panneau devant (à une distance de 430mm)
    statistics = data_analysis()
    if d_gauche >25:
        gauche()    #fonction à déterminer
    elif d_gauche<25:
        droite()
        
distance_parcourue = 0
while distance_parcourue < 300:
    statistics = data_analysis()
    distance_parcourue = d()