from adafruit_rplidar import RPLidar
from math import *
import numpy as np
import RPi.GPIO as GPIO
import time

class Robot:
    def __init__(self):
        self.PORT_NAME = '/dev/ttyUSB3'
        self.lidar = RPLidar(None, self.PORT_NAME, timeout=3)
        self.scan_data = [0] * 360
        self.x_data = np.arange(360)
        self.Pins = [
            [16, 20, 21],  # M1_En, M1_In1, M1_In2
            [5, 6, 13]  # M2_En, M2_In1, M2_In2
        ]
        self.M1_Vitesse = None
        self.M2_Vitesse = None


    def initialize(self):
        self.lidar = RPLidar(None, self.PORT_NAME, timeout=3)
        self.scan_data = [0] * 360
        self.x_data = np.arange(360)

        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        for pins in self.Pins:
            GPIO.setup(pins, GPIO.OUT)
        self.M1_Vitesse = GPIO.PWM(self.Pins[0][0], 100)
        self.M2_Vitesse = GPIO.PWM(self.Pins[1][0], 100)
        self.M1_Vitesse.start(100)
        self.M2_Vitesse.start(100)


    def sens1(self, moteurNum):
        GPIO.output(self.Pins[moteurNum - 1][1], GPIO.HIGH)
        GPIO.output(self.Pins[moteurNum - 1][2], GPIO.LOW)
        print("Moteur", moteurNum, "tourne dans le sens 1.")


    def sens2(self, moteurNum):
        GPIO.output(self.Pins[moteurNum - 1][1], GPIO.LOW)
        GPIO.output(self.Pins[moteurNum - 1][2], GPIO.HIGH)
        print("Moteur", moteurNum, "tourne dans le sens 2.")


    def arret(self, moteurNum):
        GPIO.output(self.Pins[moteurNum - 1][1], GPIO.LOW)
        GPIO.output(self.Pins[moteurNum - 1][2], GPIO.LOW)
        print("Moteur", moteurNum, "arret.")


    def arretComplet(self):
        for pins in self.Pins:
            GPIO.output(pins[1], GPIO.LOW)
            GPIO.output(pins[2], GPIO.LOW)
        print("Moteurs arretes.")


    def avancer(self):
        self.sens1(1)
        self.sens2(2)


    def gauche_slow(self):
        self.arret(1)
        self.sens2(2)


    def droite_slow(self):
        self.arret(2)
        self.sens1(1)


    def data_analysis(self):
        x = [-self.scan_data[i] * math.cos(self.x_data[i] * pi / 180) for i in range(225, 315)]
        y = [self.scan_data[i] * math.sin(self.x_data[i] * pi / 180) for i in range(225, 315)]
        X = []
        Y = []
        x_panneau = []
        y_panneau = []
        for i in range(len(x)):
            if sqrt(x[i] ** 2 + y[i] ** 2) < 700 and y[i] != 0:
                X.append(x[i])
                Y.append(y[i])
        if len(X) != 0:
            face = (X[round(len(X) / 2)], Y[round(len(Y) / 2)])
        else:
            face = (0, -440)
        for i in range(len(X)):
            if Y[i] < face[1] + 10 and Y[i] > face[1] - 10:
                x_panneau.append(X[i])
                y_panneau.append(Y[i])
        bord_droit = (x_panneau[-1], y_panneau[-1])
        bord_gauche = (x_panneau[0], y_panneau[0])
        centre_panneau = ((bord_droit[0] + bord_gauche[0]) / 2, (bord_droit[1] + bord_gauche[1]) / 2)
        d_gauche = abs(bord_gauche[0] - face[0])
        d_droit = abs(bord_droit[0] - face[0])
        return {
            "face": face,
            "bord_droit": bord_droit,
            "bord_gauche": bord_gauche,
            "centre_panneau": centre_panneau,
            "d_gauche": d_gauche,
            "d_droit": d_droit
        }


    def distance(self):
        print("Distance parcourue par le robot obtenue avec l'accéléromètre")


    def cas0(self):
        self.avancer()
        statistics = self.data_analysis()
        while statistics["face"][1] < 430:  # tant qu'il y a le panneau devant (à une distance de 430mm)
            statistics = self.data_analysis()
            if statistics["d_droit"] > 25:
                self.droite_slow()
            elif statistics["d_droit"] < 25:
                self.gauche_slow()
        return 2


    def cas1(self):
        self.avancer()
        statistics = self.data_analysis()
        while statistics["face"][1] < 430:  # tant qu'il y a le panneau devant (à une distance de 430mm)
            statistics = self.data_analysis()
            if statistics["d_gauche"] > 25:
                self.gauche_slow()  # fonction à déterminer
            elif statistics["d_gauche"] < 25:
                self.droite_slow()
        return 3


    def cas2(self):
        self.arretComplet()
        statistics = self.data_analysis()
        self.avancer()
        distance_parcourue = 0
        d_gauche = statistics["d_gauche"]
        if d_gauche >= 50:
            parametre_virage = (1, 0)
            c = 1
        else:
            parametre_virage = (a, b)  # To be determined
            c = 4

        while distance_parcourue < 30:
            statistics = self.data_analysis()
            distance_parcourue = self.d()

        while statistics["d_droit"] < 25:
            statistics = self.data_analysis()
            self.gauche_slow()

        return c


    def cas3(self):
        self.arretComplet()
        statistics = self.data_analysis()
        self.avancer()
        distance_parcourue = 0
        d_droit = statistics["d_droit"]
        if statistics["d_gauche"] >= 50:
            parametre_virage = (0, 1)
            c = 0
        else:
            parametre_virage = (a, b)  # To be determined
            c = 5

        while distance_parcourue < 30:
            statistics = self.data_analysis()
            distance_parcourue = self.d()

        while statistics["d_gauche"] < 25:
            statistics = self.data_analysis()
            self.droite_slow()

        cas = c
        return cas


    def cas4(self):
        self.arretComplet()
        self.avancer()
        statistics = self.data_analysis()
        distance_parcourue = 0
        d_droit = statistics["d_droit"]
        parametre_virage = (0, 1)

        while statistics["face"][1] < 430:   # tant qu'il y a le panneau devant (à une distance de 430mm)
            statistics = self.data_analysis()
            if statistics["d_droit"] > 25:
                self.droite_slow()    # fonction à déterminer
            elif statistics["d_droit"] < 25:
                self.gauche_slow()

        while distance_parcourue < 300:
            statistics = self.data_analysis()
            distance_parcourue = self.d()

        while statistics["d_gauche"] < 250:
            statistics = self.data_analysis()
            self.droite_slow()

        while statistics["face"][1] < 430:   # tant qu'il y a le panneau devant (à une distance de 430mm)
            statistics = self.data_analysis()
            if statistics["d_droit"] > 25:
                self.droite_slow()    # fonction à déterminer
            elif statistics["d_droit"] < 25:
                self.gauche_slow()

        distance_parcourue = 0
        while distance_parcourue < 300:
            statistics = self.data_analysis()
            distance_parcourue = self.d()


    def cas5(self):
        self.arretComplet()

        # Code à insérer sous la condition du cas4
        self.avancer()  # fonctionner avancer à déterminer
        statistics = self.data_analysis()
        distance_parcourue = 0
        d_gauche = statistics["d_gauche"]
        parametre_virage = (1, 0)

        while statistics["face"][1] < 430:  # tant qu'il y a le panneau devant (à une distance de 430mm)
            statistics = self.data_analysis()
            if statistics["d_gauche"] > 25:
                self.gauche_slow()  # fonction à déterminer
            elif statistics["d_gauche"] < 25:
                self.droite_slow()

        while distance_parcourue < 30:
            statistics = self.data_analysis()
            distance_parcourue = self.d()

        while statistics["d_droite"] < 25:
            statistics = self.data_analysis()
            self.gauche_slow()

        while statistics["face"][1] < 430:  # tant qu'il y a le panneau devant (à une distance de 430mm)
            statistics = self.data_analysis()
            if statistics["d_gauche"] > 25:
                self.gauche_slow()  # fonction à déterminer
            elif statistics["d_gauche"] < 25:
                self.droite_slow()

        distance_parcourue = 0
        while distance_parcourue < 30:
            statistics = self.data_analysis()
            distance_parcourue = self.d()

