# http://www.invensense.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf
# http://www.invensense.com/wp-content/uploads/2017/11/RM-MPU-9250A-00-v1.6.pdf

import smbus

class Mpu9250:
    """MPU-9250 gyroscope/accelerometer/magnetometer I2C driver"""
    
    def __init__(self, busNumber, deviceAddress):
        """Opens the Linux device."""
        self.smbus = smbus.SMBus(busNumber)
        self.address = deviceAddress
    
    def getByte(self, register):
        """Read byte from given register address."""
        return self.smbus.read_byte_data(self.address, register)
    
    def getWord(self, register):
        """Read word from given register address."""
        high = self.getByte(register)
        low = self.getByte(register + 1)
        return (high << 8) | low
    
    def getGyroscopeX(self):
        return self.getWord(0x43)
    
    def getGyroscopeY(self):
        return self.getWord(0x45)
    
    def getGyroscopeZ(self):
        return self.getWord(0x47)
    
    def getGyroscope(self):
        return (self.getGyroscopeX(), self.getGyroscopeY(), self.getGyroscopeZ())
    
    def getAccelerometerX(self):
        return self.getWord(0x3B)
    
    def getAccelerometerY(self):
        return self.getWord(0x3D)
    
    def getAccelerometerZ(self):
        return self.getWord(0x3F)
    
    def getAccelerometer(self):
        return (self.getAccelerometerX(), self.getAccelerometerY(), self.getAccelerometerZ())
    
    def getMagnetometerX(self):
        return 0
    
    def getMagnetometerY(self):
        return 0
    
    def getMagnetometerZ(self):
        return 0
    
    def getMagnetometer(self):
        return (self.getMagnetometerX(), self.getMagnetometerY(), self.getMagnetometerZ())
    
    def getTemperature(self):
        return self.getWord(0x41)
    
    def getSensorData(self):
        return {
            'gyroscope': self.getGyroscope(),
            'accelerometer': self.getAccelerometer(),
            'magnetometer': self.getMagnetometer(),
            'temperature': self.getTemperature()
        }
