#!/usr/bin/env python3

from time import sleep
from Mpu9250Driver import Mpu9250

mpu9250 = Mpu9250(1)
print(mpu9250.getMagnetometerSensitivity())

while True:
    rawSensorData = mpu9250.getRawSensorData()
    print(rawSensorData)
    print(mpu9250.convert(rawSensorData))
    print('')
    sleep(0.33)
