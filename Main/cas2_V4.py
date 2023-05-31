from adafruit_rplidar import RPLidar
from math import *
import numpy as np
import RPi.GPIO as GPIO
import time
import smbus

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

        self.bus = smbus.SMBus(1)  # or bus = smbus.SMBus(0) for older version boards
        self.device_address = 0x68  # MPU6050 device address
        self.a0 = [0, 0, 0]
        self.v0 = [0, 0, 0]
        self.d0 = [0, 0, 0]
        self.i0 = [0, 0, 0]
        self.t0 = time.time()

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

        self.MPU_Init()

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
        x = [-self.scan_data[i] * cos(self.x_data[i] * pi / 180) for i in range(225, 315)]
        y = [self.scan_data[i] * sin(self.x_data[i] * pi / 180) for i in range(225, 315)]
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

    def MPU_Init(self):
        self.bus.write_byte_data(self.device_address, self.SMPLRT_DIV, 7)  # write to sample rate register
        self.bus.write_byte_data(self.device_address, self.PWR_MGMT_1, 1)  # Write to power management register
        self.bus.write_byte_data(self.device_address, self.CONFIG, 0)  # Write to Configuration register
        self.bus.write_byte_data(self.device_address, self.GYRO_CONFIG, 24)  # Write to Gyro configuration register
        self.bus.write_byte_data(self.device_address, self.INT_ENABLE, 1)  # Write to interrupt enable register

    def read_raw_data(self, addr):
        high = self.bus.read_byte_data(self.device_address, addr)  # Accelero and Gyro value are 16-bit
        low = self.bus.read_byte_data(self.device_address, addr + 1)
        value = ((high << 8) | low)  # concatenate higher and lower value
        if value > 32768:  # to get signed value from mpu6050
            value = value - 65536
        return value

    def giro(self):
        t1 = time.time()
        gyro_x = self.read_raw_data(self.GYRO_XOUT_H)
        gyro_y = self.read_raw_data(self.GYRO_YOUT_H)
        gyro_z = self.read_raw_data(self.GYRO_ZOUT_H)
        Gx = gyro_x / 131.0
        Gy = gyro_y / 131.0
        Gz = gyro_z / 131.0
        ix = (Gx - self.i0[0]) / (t1 - self.t0) + self.i0[0]
        iy = (Gy - self.i0[1]) / (t1 - self.t0) + self.i0[1]
        iz = (Gz - self.i0[2]) / (t1 - self.t0) + self.i0[2]
        return {"Gx": Gx, "Gy": Gy, "Gz": Gz, "ix": ix, "iy": iy, "iz": iz, "t0": t1}

    def a_v_d(self):
        t1 = time.time()
        acc_x = self.read_raw_data(self.ACCEL_XOUT_H)
        acc_y = self.read_raw_data(self.ACCEL_YOUT_H)
        acc_z = self.read_raw_data(self.ACCEL_ZOUT_H)
        Ax = acc_x / 16384.0
        Ay = acc_y / 16384.0
        Az = acc_z / 16384.0 - 1
        Vx = (Ax - self.a0[0]) / (t1 - self.t0) + self.v0[0]
        Vy = (Ay - self.a0[1]) / (t1 - self.t0) + self.v0[1]
        Vz = (Az - self.a0[2]) / (t1 - self.t0) + self.v0[2]
        Dx = (Vx - self.v0[0]) / (t1 - self.t0) + self.d0[0]
        Dy = (Vy - self.v0[1]) / (t1 - self.t0) + self.d0[1]
        Dz = (Vz - self.v0[2]) / (t1 - self.t0) + self.d0[2]
        return {"Ax": Ax, "Ay": Ay, "Az": Az, "Vx": Vx, "Vy": Vy, "Vz": Vz, "Dx": Dx, "Dy": Dy, "Dz": Dz, "t0": t1}

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
            adv = self.a_v_d()
            self.d0 = [adv["Dx"],adv["Dy"],adv["Dz"]]
            distance_parcourue = sqrt(self.d0[0]**2,self.d0[1]**2,self.d0[2]**2)

        while statistics["d_droit"] < 25:
            statistics = self.data_analysis()
            self.gauche_slow()

        return c

robot = Robot()
robot.initialize()
cas = robot.cas2()

