#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Apr 21 09:17:42 2023

@author: yannick
"""

import smbus					#import SMBus module of I2C
from time import sleep          #import

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

def read_acc():
    acc_x = read_raw_data(ACCEL_XOUT_H)
    acc_y = read_raw_data(ACCEL_YOUT_H)
    acc_z = read_raw_data(ACCEL_ZOUT_H)
    Ax = acc_x/16384.0
    Ay = acc_y/16384.0
    Az = acc_z/16384.0
    return {"Ax":Ax,"Ay":Ay,"Az":Az}