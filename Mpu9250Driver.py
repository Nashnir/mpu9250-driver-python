# http://www.invensense.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf
# http://www.invensense.com/wp-content/uploads/2017/11/RM-MPU-9250A-00-v1.6.pdf

import smbus

# MPU-9250 register addresses
SAMPLE_RATE_DIVIDER = 0x19
CONFIGURATION = 0x1A
I2C_MASTER_CONTROL = 0x24
I2C_SLAVE0_DEVICE_ADDRESS = 0x25
I2C_SLAVE0_REGISTER_ADDRESS = 0x26
I2C_SLAVE0_CONTROL = 0x27
INT_PIN_AND_BYPASS_CONFIGURATION = 0x37
I2C_MASTER_DELAY_CONTROL = 0x67
USER_CONTROL = 0x6A
DEVICE_ID = 0x75

# Magnetometer register addresses
MAGNETOMETER_DEVICE_ID = 0x00
CONTROL_1 = 0x0A

# INT_PIN_AND_BYPASS_CONFIGURATION_ADDRESS bits
BYPASS_ENABLE = 0x02

# CONTROL_1 bits
POWER_DOWN_MODE = 0x0
SINGLE_MEASUREMENT_MODE = 0x1
CONTINUOUS_MEASUREMENT_MODE_1 = 0x2
CONTINUOUS_MEASUREMENT_MODE_2 = 0x6
EXTERNAL_TRIGGER_MEASUREMENT_MODE = 0x4
SELF_TEST_MODE = 0x8
FUSE_ROM_ACCESS_MODE = 0xF

def convertBytesToWord(low, high):
    return (high << 8) | low

class Mpu9250:
    """MPU-9250 gyroscope/accelerometer/magnetometer I2C driver"""
    
    def __init__(self, busNumber, deviceAddress):
        """Opens the Linux device."""
        self.smbus = smbus.SMBus(busNumber)
        self.address = deviceAddress
        self.magnetometerAddress = 0x0C
        # Assert that the connection is OK
        self.setByte(USER_CONTROL, 0x00)
        self.orByte(INT_PIN_AND_BYPASS_CONFIGURATION, BYPASS_ENABLE)
        assert (self.getByte(DEVICE_ID) == 0x73)
        assert (self.getByte(MAGNETOMETER_DEVICE_ID, True) == 0x48)
        # Configure magnetometer.
        # Status registers must be configured into the read range of I2C
        # slave, otherwise the sensor data never gets updated.
        self.setByte(CONTROL_1, FUSE_ROM_ACCESS_MODE, True)
        self.setByte(CONTROL_1, POWER_DOWN_MODE, True)
        self.setByte(CONTROL_1, CONTINUOUS_MEASUREMENT_MODE_2, True)
        self.andByte(INT_PIN_AND_BYPASS_CONFIGURATION, ~BYPASS_ENABLE)
        self.setByte(I2C_MASTER_CONTROL, 0x40)
        self.setByte(I2C_MASTER_DELAY_CONTROL, 0x01)
        self.setByte(USER_CONTROL, 0x20)
        self.setByte(I2C_SLAVE0_DEVICE_ADDRESS, self.magnetometerAddress | 0x80)
        self.setByte(I2C_SLAVE0_REGISTER_ADDRESS, 0x02) # 0x02 is status register
        self.setByte(I2C_SLAVE0_CONTROL, 0x88)
    
    def getDeviceAddress(self, accessMagnetometer):
        return self.magnetometerAddress if accessMagnetometer else self.address
    
    def getByte(self, register, accessMagnetometer = False):
        """Read byte from given register address."""
        return self.smbus.read_byte_data(self.getDeviceAddress(accessMagnetometer), register)
    
    def getBigEndianWord(self, register):
        """Read big-endian word from given register address."""
        high = self.getByte(register)
        low = self.getByte(register + 1)
        return convertBytesToWord(low, high)
    
    def getLittleEndianWord(self, register):
        """Read little-endian word from given register address."""
        low = self.getByte(register)
        high = self.getByte(register + 1)
        return convertBytesToWord(low, high)
    
    def setByte(self, register, value, accessMagnetometer = False):
        """Write byte to given register address."""
        return self.smbus.write_byte_data(self.getDeviceAddress(accessMagnetometer), register, value)
    
    def orByte(self, register, mask):
        value = self.getByte(register)
        self.setByte(register, value | mask)
    
    def andByte(self, register, mask):
        value = self.getByte(register)
        self.setByte(register, value & mask)
    
    def getGyroscopeX(self):
        return self.getBigEndianWord(0x43)
    
    def getGyroscopeY(self):
        return self.getBigEndianWord(0x45)
    
    def getGyroscopeZ(self):
        return self.getBigEndianWord(0x47)
    
    def getGyroscope(self):
        return (self.getGyroscopeX(), self.getGyroscopeY(), self.getGyroscopeZ())
    
    def getAccelerometerX(self):
        return self.getBigEndianWord(0x3B)
    
    def getAccelerometerY(self):
        return self.getBigEndianWord(0x3D)
    
    def getAccelerometerZ(self):
        return self.getBigEndianWord(0x3F)
    
    def getAccelerometer(self):
        return (self.getAccelerometerX(), self.getAccelerometerY(), self.getAccelerometerZ())
    
    def getMagnetometerX(self):
        return self.getLittleEndianWord(0x4A)
    
    def getMagnetometerY(self):
        return self.getLittleEndianWord(0x4C)
    
    def getMagnetometerZ(self):
        return self.getLittleEndianWord(0x4E)
    
    def getMagnetometer(self):
        return (self.getMagnetometerX(), self.getMagnetometerY(), self.getMagnetometerZ())
    
    def getTemperature(self):
        return self.getBigEndianWord(0x41)
    
    def getSensorData(self):
        return {
            'gyroscope': self.getGyroscope(),
            'accelerometer': self.getAccelerometer(),
            'magnetometer': self.getMagnetometer(),
            'temperature': self.getTemperature()
        }
