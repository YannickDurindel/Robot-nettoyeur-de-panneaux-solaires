#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Apr 20 15:59:25 2023

@author: yannick
"""

from adafruit_rplidar import RPLidar
from math import *
import numpy as np
import RPi.GPIO as GPIO
import time

PORT_NAME = '/dev/ttyUSB3'
lidar = RPLidar(None, PORT_NAME, timeout=3)
scan_data = [0]*360
x_data = np.arange(360)

# Definition des pins
M1_En = 16
M1_In1 = 20
M1_In2 = 21
M2_En = 5
M2_In1 = 6
M2_In2 = 13
Pins = [[M1_En, M1_In1, M1_In2], [M2_En, M2_In1, M2_In2]]

# Setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(M1_En, GPIO.OUT)
GPIO.setup(M1_In1, GPIO.OUT)
GPIO.setup(M1_In2, GPIO.OUT)
GPIO.setup(M2_En, GPIO.OUT)
GPIO.setup(M2_In1, GPIO.OUT)
GPIO.setup(M2_In2, GPIO.OUT)
M1_Vitesse = GPIO.PWM(M1_En, 100)
M2_Vitesse = GPIO.PWM(M2_En, 100)
M1_Vitesse.start(100)
M2_Vitesse.start(100)


def sens1(moteurNum) :
    GPIO.output(Pins[moteurNum - 1][1], GPIO.HIGH)
    GPIO.output(Pins[moteurNum - 1][2], GPIO.LOW)
    print("Moteur", moteurNum, "tourne dans le sens 1.")


def sens2(moteurNum) :
    GPIO.output(Pins[moteurNum - 1][1], GPIO.LOW)
    GPIO.output(Pins[moteurNum - 1][2], GPIO.HIGH)
    print("Moteur", moteurNum, "tourne dans le sens 2.")

def arret(moteurNum) :
    GPIO.output(Pins[moteurNum - 1][1], GPIO.LOW)
    GPIO.output(Pins[moteurNum - 1][2], GPIO.LOW)
    print("Moteur", moteurNum, "arret.")

def arretComplet() :
    GPIO.output(Pins[0][1], GPIO.LOW)
    GPIO.output(Pins[0][2], GPIO.LOW)
    GPIO.output(Pins[1][1], GPIO.LOW)
    GPIO.output(Pins[1][2], GPIO.LOW)
    print("Moteurs arretes.")

def avancer():
    sens1(1)
    sens2(2)

def gauche_slow():
    arret(1)
    sens2(2)

def gauche_fast():
    sens2(2)
    sens2(1)

def droite_slow():
    arret(2)
    sens1(1)

def droite_fast():
    sens1(1)
    sens1(2)

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

def d():
    print("distance parcourue par le robot obtenue avec l'accélerometre")

arretComplet()

for scan in lidar.iter_scans():
    for (_, angle, distance) in scan:
        scan_data[min([359, floor(angle)])] = distance
    statistics = data_analysis()

    if cas == 0:    #panneau devant & ligne à droite
        avancer()   #fonctionner avancer à déterminer
        statistics = data_analysis()
        while statistics["face"][1]<430:   #tant qu'il y a le panneau devant (à une distance de 430mm)
            statistics = data_analysis()
            if statistics["d_droit"] >25:
                droite_slow()    #fonction à déterminer
            elif statistics["d_droit"]<25:
                gauche_slow()
        cas = 2


    elif cas == 1:  #panneau devant & ligne à gauche
        avancer()   #fonctionner avancer à déterminer
        statistics = data_analysis()
        while statistics["face"][1]<430:   #tant qu'il y a le panneau devant (à une distance de 430mm)
            statistics = data_analysis()
            if statisitcs["d_gauche"] >25:
                gauche_slow()    #fonction à déterminer
            elif statisitcs["d_gauche"]<25:
                droite_slow()
        cas = 3


    elif cas == 2:  #pas de panneau en face et ligne à droite
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

        while distance_parcourue<30:
            statistics = data_analysis()
            distance_parcourue = d()

        while statistics["d_droit"] < 25:
            statistics = data_analysis()
            gauche_slow()

        cas = c


    elif cas == 3:  #pas de panneau en face et ligne à gauche
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
            droite_slow()

        cas = c


    elif cas == 4:  #Bas du panneau et dernier virage à gauche
        avancer()   #fonctionner avancer à déterminer
        statistics = data_analysis()
        distance_parcourue = 0
        d_droit = statistics["d_droit"]
        parametre_virage = (0,1)

        while statistics["face"][1]<430:   #tant qu'il y a le panneau devant (à une distance de 430mm)
            statistics = data_analysis()
            if statistics["d_droit"] >25:
                droite_salow()    #fonction à déterminer
            elif statistics["d_droit"]<25:
                gauche_slow()

        while distance_parcourue < 300:
            statistics = data_analysis()
            distance_parcourue = d()

        while statistics["d_gauche"] < 250:
            statistics = data_analysis()
            droite_slow()

        while statistics["face"][1]<430:   #tant qu'il y a le panneau devant (à une distance de 430mm)
            statistics = data_analysis()
            if statistics["d_droit"] >25:
                droite_slow()    #fonction à déterminer
            elif statistics["d_droit"]<25:
                gauche_slow()

        distance_parcourue = 0
        while distance_parcourue < 300:
            statistics = data_analysis()
            distance_parcourue = d()


    elif cas == 5:  #Bas du panneau et dernier virage à droite
        avancer()   #fonctionner avancer à déterminer
        statistics = data_analysis()
        distance_parcourue = 0
        d_gauche = statistics["d_gauche"]
        parametre_virage = (1,0)

        while statistics["face"][1]<430:   #tant qu'il y a le panneau devant (à une distance de 430mm)
            statistics = data_analysis()
            if statistics["d_gauche"] >25:
                gauche_slow()    #fonction à déterminer
            elif statistics["d_gauche"]<25:
                droite_slow()

        while distance_parcourue < 30:
            statistics = data_analysis()
            distance_parcourue = d()

        while statistics["d_droite"] < 25:
            statistics = data_analysis()
            gauche_slow()

        while statistics["face"][1]<430:   #tant qu'il y a le panneau devant (à une distance de 430mm)
            statistics = data_analysis()
            if statistics["d_gauche"] >25:
                gauche_slow()    #fonction à déterminer
            elif statistics["d_gauche"]<25:
                droite_slow()

        distance_parcourue = 0
        while distance_parcourue < 30:
            statistics = data_analysis()
            distance_parcourue = d()
