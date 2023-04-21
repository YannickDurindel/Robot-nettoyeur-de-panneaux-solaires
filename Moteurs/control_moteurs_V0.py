import RPi.GPIO as GPIO
from time import sleep
  

# Definition des pins
M1_En = 16
M1_In1 = 20
M1_In2 = 21

M2_En = 5
M2_In1 = 6
M2_In2 = 13


# Creation d'une liste des pins pour chaque moteur pour compacter la suite du code
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
arretComplet()


while True :
    # Exemple de motif de boucle
    sens1(1)
    sleep(3)
    sens1(2)
    sleep(3)
    arretComplet()
    sleep(5)
    sens2(1)
    sleep(2)
    arret(1)
    sleep(1)
    sens2(2)
    sleep(2)
    arret(2)
    sleep(1)
