import RPi.GPIO as GPIO
import time

# Configuration des broches de la Raspberry Pi pour le driver L298N
# Connectez IN1, IN2, IN3 et IN4 du driver L298N aux broches GPIO de votre choix
ENA = 21
IN1 = 20
IN2 = 16
IN3 = 25
IN4 = 8
ENB = 12

# Configuration des broches en mode BCM
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Configuration des broches comme des sorties
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)


pwm_ena = GPIO.PWM(ENA, 100)
pwm_enb = GPIO.PWM(ENB, 100)

def acceleration(moteur):
	for duty_cycle in range(0, 100, 2):
				M1_Vitesse.ChangeDutyCycle(duty_cycle)
				GPIO.output(ENA, GPIO.HIGH)
				GPIO.output(IN1, GPIO.LOW)
				GPIO.output(IN2, GPIO.HIGH)
				GPIO.output(ENB, GPIO.HIGH)
				GPIO.output(IN3, GPIO.LOW)
				GPIO.output(IN4, GPIO.HIGH)
				time.sleep(0.1)
				
# Fonction pour avancer
def avancer(vitesse):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ena.start(vitesse)
    pwm_enb.start(vitesse)

def arreter():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ena.stop()
    pwm_enb.stop()

# Fonction pour reculer
def reculer(vitesse):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ena.start(vitesse)
    pwm_enb.start(vitesse)

def tourner_droite(vitesse):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ena.start(vitesse)
    pwm_enb.start(vitesse)

def tourner_gauche(vitesse):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ena.start(vitesse)
    pwm_enb.start(vitesse)

# Exemple d'utilisation des fonctions pour faire avancer le robot pendant 3 secondes
try:
    pwm_ena.start(0)
    pwm_enb.start(0)
    vitesse = 50
    avancer(vitesse)
    time.sleep(3)
    arreter()
finally:
    GPIO.cleanup()
