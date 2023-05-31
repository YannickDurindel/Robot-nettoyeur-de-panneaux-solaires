from math import *
import numpy as np
import RPi.GPIO as GPIO
import time

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
p1 = GPIO.PWM(M1_En, 100)
p2 = GPIO.PWM(M2_En, 100)
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

p1.ChangeDutyCycle(a*50)
p2.ChangeDutyCycle(50)
avancer()
time.sleep(5)
gauche_slow()
time.sleep(3)
p1.ChangeDutyCycle(50)
p2.ChangeDutyCycle(a*50)
avancer()
time.sleep(5)
droite_slow()
time.sleep(1.5)
p1.ChangeDutyCycle(a*50)
p2.ChangeDutyCycle(50)
sens1(1)
sens1(2)
time.sleep(1.5)
avancer()
time.sleep(5)
droite_slow()
time.sleep(1.5)
avancer()
time.sleep(3)
p1.ChangeDutyCycle(50)
p2.ChangeDutyCycle(50)
arret(1)
arret(2)
