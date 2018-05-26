# http://www.invensense.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf
# http://www.invensense.com/wp-content/uploads/2017/11/RM-MPU-9250A-00-v1.6.pdf

from smbus import SMBus

# I2C device addresses
MPU9250_ADDRESS = 0x68
AK8963_ADDRESS = 0x0C

# MPU-9250 register addresses
SAMPLE_RATE_DIVIDER = 0x19
CONFIGURATION = 0x1A
GYROSCOPE_CONFIGURATION = 0x1B
ACCELEROMETER_CONFIGURATION = 0x1C
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

# GYROSCOPE_CONFIGURATION bits
GYROSCOPE_FULL_SCALE_SELECT_BITS = 0x18
GYROSCOPE_FULL_SCALE_SELECT_500 = 0x08
GYROSCOPE_FULL_SCALE_SELECT_1000 = 0x10
GYROSCOPE_FULL_SCALE_SELECT_2000 = 0x18

# ACCELEROMETER_CONFIGURATION bits
ACCELEROMETER_FULL_SCALE_SELECT_BITS = 0x18
ACCELEROMETER_FULL_SCALE_SELECT_4G = 0x08
ACCELEROMETER_FULL_SCALE_SELECT_8G = 0x10
ACCELEROMETER_FULL_SCALE_SELECT_16G = 0x18

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
OUTPUT_16_BIT = 0x10

def convertBytesToWord(low, high):
    return (high << 8) | low

def convertBytesToInteger(low, high):
    word = convertBytesToWord(low, high)
    return ((-((word ^ 0xFFFF) + 1)) if (word & 0x8000) else word)

def convertGyroscopeToDegreesPerSecond(rawGyroscope, scale):
    # TODO Sensitivity adjustment? Depends on temperature?
    gyroscope = []
    for index in range(3):
        gyroscope.append((rawGyroscope[index] / 32768) * scale)
    return gyroscope

def convertAccelerometerToG(rawAccelerometer, scale):
    # TODO Sensitivity adjustment? Depends on temperature?
    accelerometer = []
    for index in range(3):
        accelerometer.append((rawAccelerometer[index] / 32768) * scale)
    return accelerometer

def convertMagnetometerToMicroTesla(rawMagnetometer):
    # TODO Sensitivity adjustment? Depends on temperature?
    magnetometer = []
    for index in range(3):
        magnetometer.append((rawMagnetometer[index] / 32760) * 4912)
    return magnetometer

def convertTemperatureToCelsius(rawTemperature, roomTemperatureOffset, temperatureSensitivity):
    return ((rawTemperature - roomTemperatureOffset) / temperatureSensitivity) + 21

class Mpu9250:
    """MPU-9250 gyroscope/accelerometer/magnetometer I2C driver"""
    
    def __init__(self, busNumber):
        """Opens the Linux device."""
        self.smbus = SMBus(busNumber)
        # Calibration data
        self.roomTemperatureOffset = 1000
        self.temperatureSensitivity = 100
        self.gyroscopeScale = 2000
        self.accelerometerScale = 16
        # Configure the device.
        # Pass-through mode is used, which means that the AK8963 magnetometer
        # is accessed as a separate I2C device.
        self.resetBits(CONFIGURATION, EXTERNAL_SYNCHRONIZATION_SET_BITS)
        self.setByte(GYROSCOPE_CONFIGURATION, GYROSCOPE_FULL_SCALE_SELECT_2000)
        self.setByte(ACCELEROMETER_CONFIGURATION, ACCELEROMETER_FULL_SCALE_SELECT_16G)
        self.setByte(FIFO_ENABLE_REGISTER, 0x00)
        self.resetBits(I2C_MASTER_CONTROL, (MULTI_MASTER_ENABLE | SLAVE3_FIFO_ENABLE))
        self.resetBits(INTERRUPT_PIN_AND_BYPASS_CONFIGURATION, FSYNC_INTERRUPT_MODE_ENABLE)
        self.setBits(INTERRUPT_PIN_AND_BYPASS_CONFIGURATION, BYPASS_ENABLE)
        self.setByte(INTERRUPT_ENABLE, 0x00)
        self.setByte(ACCELEROMETER_INTERRUPT_CONTROL, 0x00)
        self.resetBits(USER_CONTROL, (FIFO_ENABLE | I2C_MASTER_ENABLE))
        self.setByte(CONTROL_1, (CONTINUOUS_MEASUREMENT_MODE_1 | OUTPUT_16_BIT), True)
        # Assert that the connection is OK
        assert (self.getByte(DEVICE_ID) == 0x73)
        assert (self.getByte(MAGNETOMETER_DEVICE_ID, True) == 0x48)
    
    def getDeviceAddress(self, accessMagnetometer):
        return AK8963_ADDRESS if accessMagnetometer else MPU9250_ADDRESS
    
    def getByte(self, register, accessMagnetometer = False):
        """Read byte from given register address."""
        return self.smbus.read_byte_data(self.getDeviceAddress(accessMagnetometer), register)
    
    def getTwoBytes(self, register):
        return (self.getByte(register), self.getByte(register + 1))
    
    def getBigEndianWord(self, register):
        """Read big-endian word from given register address."""
        bytes = self.getTwoBytes(register)
        return convertBytesToWord(bytes[1], bytes[0])
    
    def getSignedBigEndianWord(self, register):
        bytes = self.getTwoBytes(register)
        return convertBytesToInteger(bytes[1], bytes[0])
    
    def getLittleEndianWord(self, register):
        """Read little-endian word from given register address."""
        low = self.getByte(register, True)
        high = self.getByte(register + 1, True)
        integer = convertBytesToInteger(low, high)
        if (integer > 32760): integer = 32760
        if (integer < -32760): integer = -32760
        return integer
    
    def setByte(self, register, value, accessMagnetometer = False):
        """Write byte to given register address."""
        return self.smbus.write_byte_data(self.getDeviceAddress(accessMagnetometer), register, value)
    
    def setBits(self, register, mask):
        value = self.getByte(register)
        self.setByte(register, value | mask)
    
    def resetBits(self, register, mask):
        value = self.getByte(register)
        self.setByte(register, value & (~mask))
    
    def getThreeBigEndianWords(self, address):
        return [self.getSignedBigEndianWord(address), self.getSignedBigEndianWord(address + 2), self.getSignedBigEndianWord(address + 4)]
    
    def getGyroscope(self):
        return self.getThreeBigEndianWords(0x43)
    
    def getAccelerometer(self):
        return self.getThreeBigEndianWords(0x3B)
    
    def getMagnetometer(self):
        data = [self.getLittleEndianWord(0x03), self.getLittleEndianWord(0x05), self.getLittleEndianWord(0x07)]
        # Status registers must be read, otherwise the sensor data never
        # gets updated.
        self.getByte(0x02, True)
        self.getByte(0x09, True)
        return data
    
    def getTemperature(self):
        return self.getBigEndianWord(0x41)
    
    def getRawSensorData(self):
        return {
            'gyroscope': self.getGyroscope(),
            'accelerometer': self.getAccelerometer(),
            'magnetometer': self.getMagnetometer(),
            'temperature': self.getTemperature()
        }
    
    def convert(self, rawSensorData):
        """Convert raw sensor data to common units."""
        return {
            'gyroscope': convertGyroscopeToDegreesPerSecond(rawSensorData['gyroscope'], self.gyroscopeScale),
            'accelerometer': convertAccelerometerToG(rawSensorData['accelerometer'], self.accelerometerScale),
            'magnetometer': convertMagnetometerToMicroTesla(rawSensorData['magnetometer']),
            'temperature': convertTemperatureToCelsius(rawSensorData['temperature'], self.roomTemperatureOffset, self.temperatureSensitivity)
        }
    
    def getSensorData(self):
        return self.convert(self.getRawSensorData())
