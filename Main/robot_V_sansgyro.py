from adafruit_rplidar import RPLidar    # importation des modules
from math import *
import numpy as np
import RPi.GPIO as GPIO
import time

# Création de la class Robot et des fonctions utilisées
class Robot:    
    # Définition de la fonction __init__ qui initialise les variables et données de l'algorithme        
    def __init__(self):
        # Définion des données nécessaire au fonctionnement du lidar
        self.PORT_NAME = '/dev/ttyUSB0'
        self.lidar = RPLidar(None, self.PORT_NAME, timeout=3)
        self.scan_data = [0] * 360
        self.x_data = np.arange(360)

        # Définition des pins GPIO utilisés sur le rpi
        # 9 GPIO used ; 17 available       
        self.M1_En = 16
        self.M1_In1 = 20
        self.M1_In2 = 21        # M1 et M2 sont les moteurs avancements
        self.M2_En = 13
        self.M2_In1 = 19
        self.M2_In2 = 26
        self.EV = 6
        self.Pins = [[self.M1_En, self.M1_In1, self.M1_In2], [self.M2_En, self.M2_In1, self.M2_In2]]

        # Paramétrage des GPIO et des commandes de moteurs (vitesse avec pwm)
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.M1_En, GPIO.OUT)
        GPIO.setup(self.M1_In1, GPIO.OUT)
        GPIO.setup(self.M1_In2, GPIO.OUT)
        GPIO.setup(self.M2_En, GPIO.OUT)
        GPIO.setup(self.M2_In1, GPIO.OUT)
        GPIO.setup(self.M2_In2, GPIO.OUT)
        GPIO.setup(self.M_brosses_cw, GPIO.OUT)
        GPIO.setup(self.M_brosses_cw, GPIO.OUT)
        GPIO.setup(self.EV_En, GPIO.OUT)
        GPIO.setup(self.EV_In1, GPIO.OUT)
        GPIO.setup(self.EV_In2, GPIO.OUT)
        GPIO.output(self.EV,HIGH)
        self.M1_Vitesse = GPIO.PWM(self.M1_En, 100)
        self.M2_Vitesse = GPIO.PWM(self.M2_En, 100)
        self.M1_Vitesse.start(100)
        self.M2_Vitesse.start(100)
        self.p1 = GPIO.PWM(M1_En, 100)
        self.p2 = GPIO.PWM(M2_En, 100)
        self.p1.ChangeDutyCycle(25)
        self.p2.ChangeDutyCycle(25)


    # Fonction qui fait tourner un moteur dans un sens qu'on dira positif
    def sens1(self, moteurNum):
        GPIO.output(self.Pins[moteurNum - 1][1], GPIO.HIGH)
        GPIO.output(self.Pins[moteurNum - 1][2], GPIO.LOW)
        print("Moteur", moteurNum, "tourne dans le sens 1.")

    # Fonction qui fait tourner un moteur dans un sens qu'on dira négatif
    def sens2(self, moteurNum):
        GPIO.output(self.Pins[moteurNum - 1][1], GPIO.LOW)
        GPIO.output(self.Pins[moteurNum - 1][2], GPIO.HIGH)
        print("Moteur", moteurNum, "tourne dans le sens 2.")

    # Fonction qui fat arreter un moteur
    def arret(self, moteurNum):
        GPIO.output(self.Pins[moteurNum - 1][1], GPIO.LOW)
        GPIO.output(self.Pins[moteurNum - 1][2], GPIO.LOW)
        print("Moteur", moteurNum, "arret.")

    # Fonction qui fai arreter tout les moteurs
    def arretComplet(self):
        for pins in self.Pins[0:1]:
            GPIO.output(pins[1], GPIO.LOW)
            GPIO.output(pins[2], GPIO.LOW)
        print("Moteurs arretes.")
    
    # Fonction qui fait avancer le robot
    def avancer(self):
        self.sens1(1)
        self.sens2(2)
    
    # Fonction qui fait tourner le robot à gauche en faisant tourner qu'une chenille
    def gauche_slow(self):
        self.arret(1)
        self.sens2(2)

    # Fonction qui fait tourner le robot à droite en faisant tourner qu'une chenille
    def droite_slow(self):
        self.arret(2)
        self.sens1(1)

    # Fonction qui fait tourner le robot à gauche en faisant tourner deux chenilles en sens opposé
    def gauche_fast(self):
        self.sens2(1)
        self.sens2(2)

    # Fonction qui fait tourner le robot à droite en faisant tourner deux chenilles en sens opposé
    def droite_fast(self):
        self.sens1(2)
        self.sens1(1)

    # Arret les projection de l'eau
    def EV_stop(self):
        GPIO.output(self.EV,LOW)

    # Fonction qui analyse les données du lidar dans le cas précis du robot sur le panneau solaire permettant de sortir les distances aux bords
    def scan(self):
        try:
            for scan in self.lidar.iter_scans():
                for (_, angle, distance) in scan:
                    self.scan_data[min([359, floor(angle)])] = distance
                
                xlidar = [0]
                ylidar = [0]
                plt.plot(xlidar, ylidar, color='red', marker='+', markersize=12)
                
                x = [-self.scan_data[i] * cos(self.x_data[i] * pi / 180) for i in range(0, 90)] + [-self.scan_data[i] * cos(self.x_data[i] * pi / 180) for i in range(270, 360)]
                y = [self.scan_data[i] * sin(self.x_data[i] * pi / 180) for i in range(0, 90)] + [self.scan_data[i] * sin(self.x_data[i] * pi / 180) for i in range(270, 360)]
                
                x_near = []
                y_near = []
                for i in range(len(x)):
                    if sqrt(x[i] ** 2 + y[i] ** 2) < 1500:
                        x_near.append(x[i])
                        y_near.append(y[i])
                face = (0, y_near[len(y_near) // 2])
                
                x_panneau = []
                y_panneau = []
                for i in range(len(x_near)):
                    if y_near[i] < face[1] + 100 and y_near[i] > face[1] - 100:
                        x_panneau.append(x_near[i])
                        y_panneau.append(y_near[i])
                
                bord_droit = (x_panneau[-1], y_panneau[-1])
                bord_gauche = (x_panneau[0], y_panneau[0])
                centre_panneau = (
                (bord_droit[0] + bord_gauche[0]) / 2, (bord_droit[1] + bord_gauche[1]) / 2)
                d_gauche = abs(bord_gauche[0] - face[0])
                d_droit = abs(bord_droit[0] - face[0])
        
        except KeyboardInterrupt:
            print('Stopping.')
        
        return {"centre ": face,"gauche": bord_gauche,"droit": bord_droit,"distance à gauche": d_gauche,"distance à droite": d_droit}


    
    # Fonction du cas 0
    def cas0(self,n):
        self.avancer()
        statistics = self.scan()
        while statistics["face"][1] < 430:  # tant qu'il y a le panneau devant (à une distance de 430mm)
            statistics = self.scan()
            if statistics["d_droit"] > 280 :
                self.droite_slow()
            elif statistics["d_droit"] < 220 :
                self.gauche_slow()
        n += 1
        return 2

    
    # Fonction du cas 1
    def cas1(self,n):
        self.avancer()
        statistics = self.scan()
        while statistics["face"][1] < 430:  # tant qu'il y a le panneau devant (à une distance de 430mm)
            statistics = self.scan()
            if statistics["d_gauche"] > 220 :
                self.gauche_slow()  
            elif statistics["d_gauche"] < 280 :
                self.droite_slow()
        n += 1
        return 3


    # Fonction du cas 2
    def cas2(self):
        statistics = self.data_analysis()
        self.avancer()
        d_gauche = statistics["d_gauche"]

        self.arret(3)
        if d_gauche >= 750:
            self.gauche_slow()
            c = 1
        else:
            d_droite = statictics["d_droite"] - 250
            while statistics["d_gauche"] < 250:
               statistics = self.data_analysis()
               self.gauche_slow()
               
            while statistics["d_gauche"] < 250:
                statistics = self.data_analysis()
                self.gauche_slow()
            c = 4
        return c


    # Fonction du cas 3
    def cas3(self):
        statistics = self.data_analysis()
        self.avancer()
        self.clean_counterclockwise()
        distance_parcourue = 0
        d_droit = statistics["d_droit"]

        self.arret(3)
        if statistics["d_gauche"] >= 750:
            self.droite_slow()
            c = 0
        else:
            d_gauche = statictics["d_gauche"] - 250
            while statistics["d_droite"] < 250:
               statistics = self.data_analysis()
               self.gauche_slow()
               
            while statistics["d_droite"] < 250:
                statistics = self.data_analysis()
                self.gauche_slow()
            c = 5
        return c


    def cas4(self):
        self.arretComplet()
        self.avancer()
        statistics = self.data_analysis()
        d_droit = statistics["d_droit"]

        while statistics["face"][1] < 430:   # tant qu'il y a le panneau devant (à une distance de 430mm)
            statistics = self.data_analysis()
            if statistics["d_droit"] > 280:
                self.droite_slow()    # fonction à déterminer
            elif statistics["d_droit"] < 220:
                self.gauche_slow()

        self.arret(3)
        while statistics["d_gauche"] < 250:
            statistics = self.data_analysis()
            self.droite_slow()

        while statistics["face"][1] < 430:   # tant qu'il y a le panneau devant (à une distance de 430mm)
            statistics = self.data_analysis()
            if statistics["d_droit"] > 280:
                self.droite_slow()    # fonction à déterminer
            elif statistics["d_droit"] < 220:
                self.gauche_slow()
        return None


    def cas5(self):
        self.arretComplet()

        # Code à insérer sous la condition du cas5
        self.avancer()  # fonctionner avancer à déterminer
        statistics = self.data_analysis()
        d_gauche = statistics["d_gauche"]

        while statistics["face"][1] < 430:  # tant qu'il y a le panneau devant (à une distance de 430mm)
            statistics = self.data_analysis()
            if statistics["d_gauche"] > 280:
                self.gauche_slow()  # fonction à déterminer
            elif statistics["d_gauche"] < 220:
                self.droite_slow()

        self.arret(3)
        while statistics["d_droite"] < 250:
            statistics = self.data_analysis()
            self.gauche_slow()

        while statistics["face"][1] < 430:  # tant qu'il y a le panneau devant (à une distance de 430mm)
            statistics = self.data_analysis()
            if statistics["d_gauche"] > 280:
                self.gauche_slow()  # fonction à déterminer
            elif statistics["d_gauche"] < 220:
                self.droite_slow()

        return None


Robot = Robot()     # Démarrage et déclaration de l'objet robot
cas = 0     # initialisation sur le cas 0

while True:         # boucle infini en fonction des différents cas
    if cas == 0:
        cas = Robot.cas0()
    elif cas == 1:
        cas = Robot.cas1()
    elif cas == 2:
        cas = Robot.cas2()
    elif cas == 3:
        cas = Robot.cas3()
    elif cas == 4:
        cas = Robot.cas4()
        break                   # cas pour sortir de la boucle et finir le code
    elif cas == 5:
        cas = Robot.cas5()
        break                   # cas pour sortir de la boucle et finir le code
