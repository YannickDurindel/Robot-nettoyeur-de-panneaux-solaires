#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Apr 21 10:36:42 2023

@author: yannick
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
v0 = [0,0,0]
d0 = [0,0,0]

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

MPU_Init()

def a():
    #fonction acceleration qui la renvoie sur les 3 axes
    acc_x = read_raw_data(ACCEL_XOUT_H)
    acc_y = read_raw_data(ACCEL_YOUT_H)
    acc_z = read_raw_data(ACCEL_ZOUT_H)
    Ax = acc_x/16384.0
    Ay = acc_y/16384.0
    Az = acc_z/16384.0
    return {"Ax":Ax,"Ay":Ay,"Az":Az-1}

def v(v0):
    #fonction vitesse qui la renvoie sur les 3 axes
    t0 = time.time()
    Ax = a()["Ax"]
    Ay = a()["Ay"]
    Az = a()["Az"]
    time.sleep(0.1)
    t1 = time.time()
    Vx = v0[0] + (a()["Ax"]-Ax)/(t1-t0)
    Vy = v0[1] + (a()["Ay"]-Ay)/(t1-t0)
    Vz = v0[2] + (a()["Az"]-Az)/(t1-t0)
    return {"Vx":Vx,"Vy":Vy,"Vz":Vz}
    
def d(d0):
    #fonction distance qui la renvoie sur les 3 axes
    t0 = time.time()
    Vx = v(v0)["Vx"]
    Vy = v(v0)["Vy"]
    Vz = v(v0)["Vz"]
    time.sleep(0.1)
    t1 = time.time()
    Dx = d0[0] + (v(v0)["Vx"]-Vx)/(t1-t0)
    Dy = d0[1] + (v(v0)["Vy"]-Vy)/(t1-t0)
    Dz = d0[2] + (v(v0)["Vz"]-Vz)/(t1-t0)
    return {"Dx":Dx,"Dy":Dy,"Dz":Dz}
    
while True :
    print(a())
    print(v(v0))
    print(d(d0))
