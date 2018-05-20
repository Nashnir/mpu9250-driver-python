# http://www.invensense.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf
# http://www.invensense.com/wp-content/uploads/2017/11/RM-MPU-9250A-00-v1.6.pdf

import smbus

# I2C device addresses
MPU9250_ADDRESS = 0x68
AK8963_ADDRESS = 0x0C

# MPU-9250 register addresses
SAMPLE_RATE_DIVIDER = 0x19
CONFIGURATION = 0x1A
FIFO_ENABLE_REGISTER = 0x23
I2C_MASTER_CONTROL = 0x24
I2C_SLAVE0_DEVICE_ADDRESS = 0x25
I2C_SLAVE0_REGISTER_ADDRESS = 0x26
I2C_SLAVE0_CONTROL = 0x27
INTERRUPT_PIN_AND_BYPASS_CONFIGURATION = 0x37
INTERRUPT_ENABLE = 0x38
I2C_MASTER_DELAY_CONTROL = 0x67
ACCELEROMETER_INTERRUPT_CONTROL = 0x69
USER_CONTROL = 0x6A
DEVICE_ID = 0x75

# AK8963 magnetometer register addresses
MAGNETOMETER_DEVICE_ID = 0x00
CONTROL_1 = 0x0A

# CONFIGURATION bits
EXTERNAL_SYNCHRONIZATION_SET_BITS = 0x38

# I2C_MASTER_CONTROL bits
MULTI_MASTER_ENABLE = 0x80
SLAVE3_FIFO_ENABLE = 0x20

# INTERRUPT_PIN_AND_BYPASS_CONFIGURATION bits
FSYNC_INTERRUPT_MODE_ENABLE = 0x04
BYPASS_ENABLE = 0x02

# USER_CONTROL bits
FIFO_ENABLE = 0x40
I2C_MASTER_ENABLE = 0x20

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
        # Configure the device.
        # Pass-through mode is used, which means that the AK8963 magnetometer
        # is accessed as a separate I2C device.
        self.resetBits(CONFIGURATION, EXTERNAL_SYNCHRONIZATION_SET_BITS)
        self.setByte(FIFO_ENABLE_REGISTER, 0x00)
        self.resetBits(I2C_MASTER_CONTROL, (MULTI_MASTER_ENABLE | SLAVE3_FIFO_ENABLE))
        self.resetBits(INTERRUPT_PIN_AND_BYPASS_CONFIGURATION, FSYNC_INTERRUPT_MODE_ENABLE)
        self.setBits(INTERRUPT_PIN_AND_BYPASS_CONFIGURATION, BYPASS_ENABLE)
        self.setByte(INTERRUPT_ENABLE, 0x00)
        self.setByte(ACCELEROMETER_INTERRUPT_CONTROL, 0x00)
        self.resetBits(USER_CONTROL, (FIFO_ENABLE | I2C_MASTER_ENABLE))
        self.setByte(CONTROL_1, CONTINUOUS_MEASUREMENT_MODE_1, True)
        # Assert that the connection is OK
        assert (self.getByte(DEVICE_ID) == 0x73)
        assert (self.getByte(MAGNETOMETER_DEVICE_ID, True) == 0x48)
    
    def getDeviceAddress(self, accessMagnetometer):
        return AK8963_ADDRESS if accessMagnetometer else self.address
    
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
        low = self.getByte(register, True)
        high = self.getByte(register + 1, True)
        return convertBytesToWord(low, high)
    
    def setByte(self, register, value, accessMagnetometer = False):
        """Write byte to given register address."""
        return self.smbus.write_byte_data(self.getDeviceAddress(accessMagnetometer), register, value)
    
    def setBits(self, register, mask):
        value = self.getByte(register)
        self.setByte(register, value | mask)
    
    def resetBits(self, register, mask):
        value = self.getByte(register)
        self.setByte(register, value & (~mask))
    
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
        return self.getLittleEndianWord(0x03)
    
    def getMagnetometerY(self):
        return self.getLittleEndianWord(0x05)
    
    def getMagnetometerZ(self):
        return self.getLittleEndianWord(0x07)
    
    def getMagnetometer(self):
        data = (self.getMagnetometerX(), self.getMagnetometerY(), self.getMagnetometerZ())
        # Status registers must be read, otherwise the sensor data never
        # gets updated.
        self.getByte(0x02, True)
        self.getByte(0x09, True)
        return data
    
    def getTemperature(self):
        return self.getBigEndianWord(0x41)
    
    def getSensorData(self):
        return {
            'gyroscope': self.getGyroscope(),
            'accelerometer': self.getAccelerometer(),
            'magnetometer': self.getMagnetometer(),
            'temperature': self.getTemperature()
        }
