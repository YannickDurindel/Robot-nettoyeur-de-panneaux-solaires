from adafruit_rplidar import RPLidar    # importation des modules
from math import *
import numpy as np
import RPi.GPIO as GPIO
import time
import smbus

# Création de la class Robot et des fonctions utilisées
class Robot:    
    # Définition de la fonction __init__ qui initialise les variables et données de l'algorithme        
    def __init__(self):
        # Définion des données nécessaire au fonctionnement du lidar
        self.PORT_NAME = '/dev/ttyUSB3'
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
        self.M_brosses_cw = 12      # M_brosses_cw est la commande d'un relai qui détermine l'état des 5 moteurs nettoyage branchés en parallèle
        self.EV_En = EV_En          # Par manque de GPIO, on la branchera juste sur du 5V
        self.EV_In1 = 5             # EV est une electrovanne controlée par un driver pour l'alimenter en 12V
        self.EV_In2 = 6
        self.Pins = [[self.M1_En, self.M1_In1, self.M1_In2], [self.M2_En, self.M2_In1, self.M2_In2], [self.EV_En, self.EV_In1, self.EV_In2]]

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
        self.M1_Vitesse = GPIO.PWM(self.M1_En, 100)
        self.M2_Vitesse = GPIO.PWM(self.M2_En, 100)
        self.M1_Vitesse.start(100)
        self.M2_Vitesse.start(100)
        self.p1 = GPIO.PWM(M1_En, 100)
        self.p2 = GPIO.PWM(M2_En, 100)
        self.p1.ChangeDutyCycle(25)
        self.p2.ChangeDutyCycle(25)

        # Définition des variables initiales qui seront utilisées dans le code du gyroscode et accéléromètre
        self.bus = smbus.SMBus(1)  # bus = smbus.SMBus(0)
        self.device_address = 0x68  # MPU6050 adresse
        self.a0 = [0, 0, 0]
        self.v0 = [0, 0, 0]
        self.d0 = [0, 0, 0]
        self.i0 = [0, 0, 0]
        self.t0 = time.time()

        # Définition des ports d'acquisiton des données
        self.PWR_MGMT_1 = 0x6B
        self.SMPLRT_DIV = 0x19
        self.CONFIG = 0x1A
        self.GYRO_CONFIG = 0x1B
        self.INT_ENABLE = 0x38
        self.ACCEL_XOUT_H = 0x3B
        self.ACCEL_YOUT_H = 0x3D
        self.ACCEL_ZOUT_H = 0x3F
        self.GYRO_XOUT_H = 0x43
        self.GYRO_YOUT_H = 0x45
        self.GYRO_ZOUT_H = 0x47

        # Démarrage du fonctionnement du gyroscope
        self.MPU_Init()

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

    # Fonction qui fait tourner les brosses dans le sens des aiguilles d'une montre et qui active l'éléctrovanne, donc projette de l'eau
    def clean_clockwise(self):
        GPIO.output(self.M_brosses_cw,HIGH)
        self.sens1(3)

    # Arret des brosses de nettoyage et des projection de l'eau
    def clean_stop(self):
        GPIO.output(self.M_brosses_cw,LOW)
        self.arret(3)

    # Fonction qui analyse les données du lidar dans le cas précis du robot sur le panneau solaire permettant de sortir les distances aux bords
    def scan(self):
        try:
            for scan in self.lidar.iter_scans():
                for (_, angle, distance) in scan:
                    self.scan_data[min([359, floor(angle)])] = distance
                
                xlidar = [0]
                ylidar = [0]
                plt.plot(xlidar, ylidar, color='red', marker='+', markersize=12)
                
                x = [-self.scan_data[i] * cos(self.x_data[i] * pi / 180) for i in range(0, 90)] + [
                    -self.scan_data[i] * cos(self.x_data[i] * pi / 180) for i in range(270, 360)]
                y = [self.scan_data[i] * sin(self.x_data[i] * pi / 180) for i in range(0, 90)] + [
                    self.scan_data[i] * sin(self.x_data[i] * pi / 180) for i in range(270, 360)]
                
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

    # Fonction qui initialise les adresses et services du Gyroscope et accéléromètre
    def MPU_Init(self):
        self.bus.write_byte_data(self.device_address, self.SMPLRT_DIV, 7)  # écrit un rythme enregistré
        self.bus.write_byte_data(self.device_address, self.PWR_MGMT_1, 1)  # écrit le managment de puissance enregistré
        self.bus.write_byte_data(self.device_address, self.CONFIG, 0)  # écrit la configuration enregistré
        self.bus.write_byte_data(self.device_address, self.GYRO_CONFIG, 24)  # écrit à la configuration du gyroscope enregistré
        self.bus.write_byte_data(self.device_address, self.INT_ENABLE, 1)  # écrit pour interrompre l'autorisation enregistré

    # Fonction qui permet de lire les données du gyroscope et accéléromètre
    def read_raw_data(self, addr):
        high = self.bus.read_byte_data(self.device_address, addr)  # Accelero et Gyro valeur sont 16-bit
        low = self.bus.read_byte_data(self.device_address, addr + 1)
        value = ((high << 8) | low)  # concatène hautes et basses valeures
        if value > 32768:  # pour avoir la valeur depuis mpu6050
            value = value - 65536
        return value

    # Fonction retourne la vitesse de rotation du capteur et son angle à sa position initiale
    def giro(self):
        time.sleep(0.1)
        t1 = time.time()

        gyro_x = self.read_raw_data(self.GYRO_XOUT_H)
        gyro_y = self.read_raw_data(self.GYRO_YOUT_H)
        gyro_z = self.read_raw_data(self.GYRO_ZOUT_H)

        Gx = gyro_x / 131.0
        Gy = gyro_y / 131.0
        Gz = gyro_z / 131.0

        #intègre la vitesse de roation pour avoir l'angle
        ix = Gx * (t1 - self.t0) + self.i0[0]
        iy = Gy * (t1 - self.t0) + self.i0[1]
        iz = Gz * (t1 - self.t0) + self.i0[2]

        return {"Gx": Gx, "Gy": Gy, "Gz": Gz, "ix": ix, "iy": iy, "iz": iz, "t0": t1}

    # Fonction retourne l'accélération, la vitesse et la distance tout en comparant avec les condition initiales.
    def a_v_d(self):
        time.sleep(0.1)
        t1 = time.time()

        Px = math.cos(self.giro["iy"]) * math.cos(self.giro["iz"])
        Py = math.cos(self.giro["ix"]) * math.sin(self.giro["iz"])
        Pz = math.cos(self.giro["ix"]) * math.cos(self.giro["iy"])

        acc_x = self.read_raw_data(self.ACCEL_XOUT_H)
        acc_y = self.read_raw_data(self.ACCEL_YOUT_H)
        acc_z = self.read_raw_data(self.ACCEL_ZOUT_H)

        Ax = acc_x / 16384.0 - Px
        Ay = acc_y / 16384.0 - Py
        Az = acc_z / 16384.0 - Pz

        # intégraion de l'accélération pour avoir la vitesse
        Vx = Ax * (t1 - self.t0) + self.v0[0]
        Vy = Ay * (t1 - self.t0) + self.v0[1]
        Vz = Az * (t1 - self.t0) + self.v0[2]

        # intégration de la vitesse pour avoir la distance parcourue
        Dx = Vx * (t1 - self.t0) + self.d0[0]
        Dy = Vy * (t1 - self.t0) + self.d0[1]
        Dz = Vz * (t1 - self.t0) + self.d0[2]

        return {"Ax": Ax, "Ay": Ay, "Az": Az, "Vx": Vx, "Vy": Vy, "Vz": Vz, "Dx": Dx, "Dy": Dy, "Dz": Dz, "t0": t1}

    
    # Fonction du cas 0
    def cas0(self,n):
        self.avancer()
        self.clean_clockwise()
        statistics = self.scan()
        while statistics["face"][1] < 430:  # tant qu'il y a le panneau devant (à une distance de 430mm)
            statistics = self.scan()
            if statistics["d_droit"] > 220 :
                self.droite_slow()
            elif statistics["d_droit"] < 280 :
                self.gauche_slow()
        n += 1
        return 2

    
    # Fonction du cas 1
    def cas1(self,n):
        self.avancer()
        self.clean_clockwise()
        statistics = self.scan()
        while statistics["face"][1] < 430:  # tant qu'il y a le panneau devant (à une distance de 430mm)
            statistics = self.scan()
            if statistics["d_gauche"] > 280 :
                self.gauche_slow()  
            elif statistics["d_gauche"] < 220 :
                self.droite_slow()
        n += 1
        return 3


    # Fonction du cas 2
    def cas2(self):
        statistics = self.scan()
        angle = self.giro()
        self.avancer()
        self.clean_clockwise()
        distance_parcourue = 0
        d_gauche = statistics["d_gauche"]
        ix = angle["ix"]

        while distance_parcourue < 200:
            statistics = self.scan()
            adv = self.a_v_d()
            self.d0 = [adv["Dx"],adv["Dy"],adv["Dz"]]
            distance_parcourue = sqrt(self.d0[0]**2,self.d0[1]**2,self.d0[2]**2)

        self.arret(3)
        if d_gauche >= 750:
            while abs(angle["ix"]) < abs(ix):
                angle = self.giro()
                self.gauche_slow()
            c = 1
        else:
            d_droite = statictics["d_droite"] - 250
            while angle["ix"] > 0 :
                statistics = self.scan()
                self.gauche_fast()
            while distance_parcourue < d_droite :
                angle = self.giro()
                adv = self.a_v_d()
                self.d0 = [adv["Dx"],adv["Dy"],adv["Dz"]]
                distance_parcourue = sqrt(self.d0[0]**2,self.d0[1]**2,self.d0[2]**2)
            while abs(angle["ix"]) < abs(ix) : 
                angle = self.giro()
                self.gauche_fast()
            c = 4
        return c


    # Fonction du cas 3
    def cas3(self):
        statistics = self.scan()
        angle = self.giro()
        self.avancer()
        self.clean_clockwise()
        distance_parcourue = 0
        d_droit = statistics["d_droit"]
        ix = angle["ix"]

        while distance_parcourue < 200:
            statistics = self.scan()
            adv = self.a_v_d()
            self.d0 = [adv["Dx"],adv["Dy"],adv["Dz"]]
            distance_parcourue = sqrt(self.d0[0]**2,self.d0[1]**2,self.d0[2]**2)

        self.arret(3)
        if statistics["d_gauche"] >= 750:
            while abs(angle["ix"]) < abs(ix):
                angle = self.giro()
                self.droite_slow()
            c = 0
        else:
            d_gauche = statictics["d_gauche"] - 250
            while angle["ix"] < 0:
                angle = self.giro()
                self.droite_fast()
            while distance_parcourue < d_gauche :
                statistics = self.scan()
                adv = self.a_v_d()
                self.d0 = [adv["Dx"],adv["Dy"],adv["Dz"]]
                distance_parcourue = sqrt(self.d0[0]**2,self.d0[1]**2,self.d0[2]**2)
            while abs(angle["ix"]) < abs(ix):
                angle = self.giro()
                self.droite_fast()
            c = 5
        return c


    # Fonction du cas 4
    def cas4(self):
        self.arretComplet()
        self.avancer()
        self.clean_clockwise()
        angle = self.giro()
        statistics = self.scan()
        distance_parcourue = 0
        d_droit = statistics["d_droit"]
        ix = giro["ix"]

        while statistics["face"][1] < 430:   # tant qu'il y a le panneau devant (à une distance de 430mm)
            statistics = self.scan()
            if statistics["d_droit"] > 280:
                self.droite_slow()   
            elif statistics["d_droit"] < 220:
                self.gauche_slow()

        while distance_parcourue < 200:
            statistics = self.scan()
            adv = self.a_v_d()
            self.d0 = [adv["Dx"],adv["Dy"],adv["Dz"]]
            distance_parcourue = sqrt(self.d0[0]**2,self.d0[1]**2,self.d0[2]**2)

        self.arret(3)
        while angle["ix"] < 0:
            angle = self.giro()
            self.gauche_slow()

        self.clean_stop()
        while statistics["face"][1] < 430:   # tant qu'il y a le panneau devant (à une distance de 430mm)
            statistics = self.scan()
            if statistics["d_droit"] > 280:
                self.droite_slow()    
            elif statistics["d_droit"] < 220:
                self.gauche_slow()

        distance_parcourue = 0
        while distance_parcourue < 200:
            statistics = self.scan()
            adv = self.a_v_d()
            self.d0 = [adv["Dx"],adv["Dy"],adv["Dz"]]
            distance_parcourue = sqrt(self.d0[0]**2,self.d0[1]**2,self.d0[2]**2)

        while abs(angle["ix"]) < abs(ix):
            angle = self.giro()
            self.gauche_slow()
        return None

    
    # Fonction du cas 5
    def cas5(self):
        self.arretComplet()

        self.avancer()  
        self.clean_clockwise()
        angle = self.giro()
        statistics = self.scan()
        distance_parcourue = 0
        d_gauche = statistics["d_gauche"]
        ix = giro["ix"]

        while statistics["face"][1] < 430:  # tant qu'il y a le panneau devant (à une distance de 430mm)
            statistics = self.scan()
            if statistics["d_gauche"] > 280:
                self.gauche_slow()  
            elif statistics["d_gauche"] < 220:
                self.droite_slow()

        while distance_parcourue < 200:
            statistics = self.scan()
            adv = self.a_v_d()
            self.d0 = [adv["Dx"],adv["Dy"],adv["Dz"]]
            distance_parcourue = sqrt(self.d0[0]**2,self.d0[1]**2,self.d0[2]**2)

        self.arret(3)
        while angle["ix"] < 0:
            angle = self.giro()
            self.droite_slow()

        self.clean_stop()
        while statistics["face"][1] < 430:  # tant qu'il y a le panneau devant (à une distance de 430mm)
            statistics = self.scan()
            if statistics["d_gauche"] > 280:
                self.gauche_slow()  
            elif statistics["d_gauche"] < 220:
                self.droite_slow()

        distance_parcourue = 0
        while distance_parcourue < 200:
            statistics = self.scan()
            adv = self.a_v_d()
            self.d0 = [adv["Dx"],adv["Dy"],adv["Dz"]]
            distance_parcourue = sqrt(self.d0[0]**2,self.d0[1]**2,self.d0[2]**2)

        while abs(angle["ix"]) < abs(ix):
            angle = self.giro()
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
