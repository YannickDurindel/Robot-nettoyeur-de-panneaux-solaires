#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Apr 21 12:55:52 2023

@author: yannick
"""

#changement de base en panneau de 30°
"""
x' = x
y' = y*cos(30) + z*sin(30)
z' = z*cos(30) + y*sin(30)
"""
import smbus					
from time import sleep          
import time

#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47
bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address
a0 = [0,0,0]
v0 = [0,0,0]
d0 = [0,0,0]
i0 = [0,0,0]
t0 = time.time()

def MPU_Init():
	bus.write_byte_data(Device_Address, SMPLRT_DIV, 7) #write to sample rate register
	bus.write_byte_data(Device_Address, PWR_MGMT_1, 1) #Write to power management register
	bus.write_byte_data(Device_Address, CONFIG, 0) #Write to Configuration register
	bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)   #Write to Gyro configuration register
	bus.write_byte_data(Device_Address, INT_ENABLE, 1) #Write to interrupt enable register

def read_raw_data(addr):
    high = bus.read_byte_data(Device_Address, addr) #Accelero and Gyro value are 16-bit
    low = bus.read_byte_data(Device_Address, addr+1)
    value = ((high << 8) | low) #concatenate higher and lower value
    if(value > 32768):      #to get signed value from mpu6050
        value = value - 65536
    return value

def giro(i0,t0):
    #fonction giroscope qui renvoie l'angle du robot
    t1 = time.time()
    gyro_x = read_raw_data(GYRO_XOUT_H)
    gyro_y = read_raw_data(GYRO_YOUT_H)
    gyro_z = read_raw_data(GYRO_ZOUT_H)
    Gx = gyro_x/131.0
    Gy = gyro_y/131.0
    Gz = gyro_z/131.0
    ix = (Gx-i0[0])/(t1-t0) + i0[0]
    iy = (Gy-i0[1])/(t1-t0) + i0[1]
    iz = (Gz-i0[2])/(t1-t0) + i0[2]
    return {"Gx":Gx,"Gy":Gy,"Gz":Gz,"ix":ix,"iy":iy,"iz":iz,"t0":t1}

MPU_Init()

def a_v_d(a0,v0,d0,t0):
    #fonction acceleration qui la renvoie sur les 3 axes
    t1 = time.time()
    acc_x = read_raw_data(ACCEL_XOUT_H)
    acc_y = read_raw_data(ACCEL_YOUT_H)
    acc_z = read_raw_data(ACCEL_ZOUT_H)
    Ax = acc_x/16384.0
    Ay = acc_y/16384.0
    Az = acc_z/16384.0 - 1
    Vx = (Ax-a0[0])/(t1-t0) + v0[0]
    Vy = (Ay-a0[1])/(t1-t0) + v0[1]
    Vz = (Az-a0[2])/(t1-t0) + v0[2]
    Dx = (Vx-v0[0])/(t1-t0) + d0[0]
    Dy = (Vy-v0[1])/(t1-t0) + d0[1]
    Dz = (Vz-v0[2])/(t1-t0) + d0[2]
    return {"Ax":Ax,"Ay":Ay,"Az":Az,"Vx":Vx,"Vy":Vy,"Vz":Vz,"Dx":Dx,"Dy":Dy,"Dz":Dz,"t0":t1}

while True:
    stat = a_v_d(a0,v0,d0,t0)
    t0 = stat["t0"]
    a0 = [stat["Ax"],stat["Ay"],stat["Az"]]
    v0 = [stat["Vx"],stat["Vy"],stat["Vz"]]
    d0 = [stat["Dx"],stat["Dy"],stat["Dz"]]   