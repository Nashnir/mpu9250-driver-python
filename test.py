#!/usr/bin/env python3

from time import sleep
from Mpu9250Driver import Mpu9250

mpu9250 = Mpu9250(1, 0x68)

while True:
    sensorData = mpu9250.getSensorData()
    print(sensorData)
    sleep(0.33)
