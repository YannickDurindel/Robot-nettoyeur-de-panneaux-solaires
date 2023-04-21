import os
from math import floor
from adafruit_rplidar import RPLidar
import time


# Setup the RPLidar
PORT_NAME = '/dev/ttyUSB3'
lidar = RPLidar(None, PORT_NAME, timeout=3)

# used to scale data to fit on the screen
max_distance = 0

def process_data(data):
    print(data)

scan_data = [0]*360
#print(lidar.get_info())
print(type(lidar.iter_scans()))

try:
    for scan in lidar.iter_scans():
        for (_, angle, distance) in scan:
            scan_data[min([359, floor(angle)])] = distance
        process_data(scan_data)
        f = open("LidarOUTPUT.txt", "a")
        f.write("\n")
        f.write(str(scan_data))



except KeyboardInterrupt:
    print('Stopping.')
    f.write("\n")
    f.close()
    
lidar.stop()
lidar.disconnect()

scan_data[min([359, floor(angle)])] = distance
