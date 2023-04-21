#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Apr 17 15:40:24 2023

@author: yannick
"""

from adafruit_rplidar import RPLidar
from math import *
import numpy as np

PORT_NAME = '/dev/ttyUSB3'
lidar = RPLidar(None, PORT_NAME, timeout=3)
scan_data = [0]*360
x_data = np.arange(360)
cas = 0

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


for scan in lidar.iter_scans():
    for (_, angle, distance) in scan:
        scan_data[min([359, floor(angle)])] = distance
    statistics = data_analysis()

    if cas == 0: 
        """Cas 0 : panneau devant & ligne à droite
        -> maintenir la droite sans la dépasser (asservissement sans dépassement / rapide)
        -> avancer tant qu'il y a un panneau devant
        => si plus de panneau : Cas 2"""
        
    elif cas == 1:
        """Cas 1 : panneau devant & ligne à gauche
        -> maintenir la gauche sans la dépasser (asservissement sans dépassement / rapide)
        -> avancer tant qu'il y a un panneau devant
        => si plus de panneau : Cas 3"""
        
    elif cas == 2:
        """Cas 2 : pas de panneau en face et ligne à droite
        -> avancer droit de 30 cm (asservissement)
        -> mesurer la distance à gauche
        => si la distance > 50cm : gauche avec b=0 --> Cas 1
        => Si la distance = d < 50 cm : gauche avec b!=0 --> Cas 4"""
    
    elif cas == 3:
        """Cas 3 : pas de panneau en face et ligne à gauche
        -> avancer droit de 30 cm (asservissement)
        -> mesurer la distance à droite
        => si la distance > 50cm : droite avec a=0 --> Cas 0
        => Si la distance = d < 50 cm : droite avec a!=0 --> Cas 5"""
    
    elif cas == 4:
        """Cas 4 : Bas du panneau et dernier virage à gauche
        -> Avancer tant que l'on voit la panneau
        -> Quand on ne voit plus de panneau : avancer de 30cm + droite/2
        -> arreter les brosses et pompe
        -> avancer vers le haut jusqu'au bord + 30cm"""
    
    elif cas == 5:
        """Cas 5 : Bas du panneau et dernier virage à droite
        -> Avancer tant que l'on voit la panneau
        -> Quand on ne voit plus de panneau : avancer de 30cm + gauche/2
        -> arreter les brosses et pompe
        -> avancer vers le haut jusqu'au bord + 30cm"""