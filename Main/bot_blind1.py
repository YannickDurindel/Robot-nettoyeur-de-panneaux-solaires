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

def avancer(n):
    if n == -1:
        sens1(1)
        sens2(2)
    elif n%2==0:
        sens1(1)
        sens2(2)
        time.sleep(1/(a*10))
        sens1(1)
        time.sleep(a/10)

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


#assuming a slope of the bot going y = -ax down with a<1 generally
t0 = time.time()
n = 0
while time.time() < t0+5:
    avancer(n)

while time.time() < t0+3:
    gauche_slow()
n+=1
while time.time() < t0+5:
    avancer(n)

while time.time() < t0+1.5:
    droite_slow()

while time.time() < t0+1.5:
    sens1(1)
    sens1(2)
n+=1
while time.time() < t0+5:
    avancer(n)

while time.time() < t0+1.5:
    droite_slow()

while time.time() < t0+3:
    avancer(-1)

arret(1)
arret(2)
