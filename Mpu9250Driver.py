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
GYROSCOPE_FULL_SCALE_SELECT_250 = 0x00 # Unit is degrees per second
GYROSCOPE_FULL_SCALE_SELECT_500 = 0x08
GYROSCOPE_FULL_SCALE_SELECT_1000 = 0x10
GYROSCOPE_FULL_SCALE_SELECT_2000 = 0x18

# ACCELEROMETER_CONFIGURATION bits
ACCELEROMETER_FULL_SCALE_SELECT_BITS = 0x18
ACCELEROMETER_FULL_SCALE_SELECT_2 = 0x00 # Unit is G
ACCELEROMETER_FULL_SCALE_SELECT_4 = 0x08
ACCELEROMETER_FULL_SCALE_SELECT_8 = 0x10
ACCELEROMETER_FULL_SCALE_SELECT_16 = 0x18

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

gyroscopeScaleToSensitivityValue = {
    GYROSCOPE_FULL_SCALE_SELECT_250: 131,
    GYROSCOPE_FULL_SCALE_SELECT_500: 65.5,
    GYROSCOPE_FULL_SCALE_SELECT_1000: 32.8,
    GYROSCOPE_FULL_SCALE_SELECT_2000: 16.4
}

accelerometerScaleToSensitivityValue = {
    ACCELEROMETER_FULL_SCALE_SELECT_2: 16384,
    ACCELEROMETER_FULL_SCALE_SELECT_4: 8192,
    ACCELEROMETER_FULL_SCALE_SELECT_8: 4096,
    ACCELEROMETER_FULL_SCALE_SELECT_16: 2048
}

def convertBytesToWord(low, high):
    return (high << 8) | low

def convertBytesToInteger(low, high):
    word = convertBytesToWord(low, high)
    return ((-((word ^ 0xFFFF) + 1)) if (word & 0x8000) else word)

def multiplyIdentity(factor):
    return (factor, factor, factor)

def tupleMultiply(tuple0, tuple1):
    return (tuple0[0] * tuple1[0], tuple0[1] * tuple1[1], tuple0[2] * tuple1[2])

class Mpu9250:
    """MPU-9250 gyroscope/accelerometer/magnetometer I2C driver"""
    
    def __init__(self, busNumber, gyroscopeScale = GYROSCOPE_FULL_SCALE_SELECT_2000, accelerometerScale = ACCELEROMETER_FULL_SCALE_SELECT_16, magnetometer16BitMode = True):
        """Opens the Linux device."""
        self.smbus = SMBus(busNumber)
        # Calibration data
        self.gyroscopeScale = gyroscopeScale
        self.accelerometerScale = accelerometerScale
        self.magnetometer16BitMode = magnetometer16BitMode
        self.roomTemperatureOffset = 1000
        self.temperatureSensitivity = 100
        # Configure the device.
        # Pass-through mode is used, which means that the AK8963 magnetometer
        # is accessed as a separate I2C device.
        self.resetBits(CONFIGURATION, EXTERNAL_SYNCHRONIZATION_SET_BITS)
        self.setByte(GYROSCOPE_CONFIGURATION, gyroscopeScale)
        self.setByte(ACCELEROMETER_CONFIGURATION, accelerometerScale)
        self.setByte(FIFO_ENABLE_REGISTER, 0x00)
        self.resetBits(I2C_MASTER_CONTROL, (MULTI_MASTER_ENABLE | SLAVE3_FIFO_ENABLE))
        self.resetBits(INTERRUPT_PIN_AND_BYPASS_CONFIGURATION, FSYNC_INTERRUPT_MODE_ENABLE)
        self.setBits(INTERRUPT_PIN_AND_BYPASS_CONFIGURATION, BYPASS_ENABLE)
        self.setByte(INTERRUPT_ENABLE, 0x00)
        self.setByte(ACCELEROMETER_INTERRUPT_CONTROL, 0x00)
        self.resetBits(USER_CONTROL, (FIFO_ENABLE | I2C_MASTER_ENABLE))
        self.setMagnetometerByte(CONTROL_1, (CONTINUOUS_MEASUREMENT_MODE_1 | (OUTPUT_16_BIT if magnetometer16BitMode else 0)))
        # Read magnetometer sensitivity adjustment values for the 16-bit mode.
        self.magnetometerSensitivity = (self.getMagnetometerSensitivityValue(0), self.getMagnetometerSensitivityValue(1), self.getMagnetometerSensitivityValue(2))
        # Assert that the connection is OK
        assert (self.getByte(DEVICE_ID) == 0x73)
        assert (self.getMagnetometerByte(MAGNETOMETER_DEVICE_ID) == 0x48)
    
    def getByteLowLevel(self, deviceAddress, register):
        """Read byte from given register address."""
        return self.smbus.read_byte_data(deviceAddress, register)
    
    def setByteLowLevel(self, deviceAddress, register, value):
        """Write byte to given register address."""
        self.smbus.write_byte_data(deviceAddress, register, value)
    
    def getByte(self, register):
        return self.getByteLowLevel(MPU9250_ADDRESS, register)
    
    def getMagnetometerByte(self, register):
        return self.getByteLowLevel(AK8963_ADDRESS, register)
    
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
        low = self.getMagnetometerByte(register)
        high = self.getMagnetometerByte(register + 1)
        return convertBytesToInteger(low, high)
    
    def setByte(self, register, value):
        self.setByteLowLevel(MPU9250_ADDRESS, register, value)
    
    def setMagnetometerByte(self, register, value):
        self.setByteLowLevel(AK8963_ADDRESS, register, value)
    
    def setBits(self, register, mask):
        value = self.getByte(register)
        self.setByte(register, value | mask)
    
    def resetBits(self, register, mask):
        value = self.getByte(register)
        self.setByte(register, value & (~mask))
    
    def getThreeBigEndianWords(self, address):
        return (self.getSignedBigEndianWord(address), self.getSignedBigEndianWord(address + 2), self.getSignedBigEndianWord(address + 4))
    
    def getMagnetometerSensitivityValue(self, axis):
        rawValue = self.getMagnetometerByte(0x10 + axis)
        return ((rawValue - 128) * 0.5) / 128 + 1 # 0.5..1.5
    
    def getGyroscope(self):
        return self.getThreeBigEndianWords(0x43)
    
    def getAccelerometer(self):
        return self.getThreeBigEndianWords(0x3B)
    
    def getMagnetometer(self):
        data = (self.getLittleEndianWord(0x03), self.getLittleEndianWord(0x05), self.getLittleEndianWord(0x07))
        # Status registers must be read, otherwise the sensor data never
        # gets updated.
        self.getMagnetometerByte(0x02)
        self.getMagnetometerByte(0x09)
        return data
    
    def getTemperature(self):
        temperature = self.previousTemperature = self.getBigEndianWord(0x41)
        return temperature
    
    def getRawSensorData(self):
        return {
            'gyroscope': self.getGyroscope(),
            'accelerometer': self.getAccelerometer(),
            'magnetometer': self.getMagnetometer(),
            'temperature': self.getTemperature()
        }
    
    def getGyroscopeSensitivity(self):
        sensitivity = multiplyIdentity(1 / gyroscopeScaleToSensitivityValue[self.gyroscopeScale])
        return sensitivity # TODO Depends on temperature
    
    def getAccelerometerSensitivity(self):
        sensitivity = multiplyIdentity(1 / accelerometerScaleToSensitivityValue[self.accelerometerScale])
        return sensitivity # TODO Depends on temperature
    
    def getMagnetometerSensitivity(self):
        return self.magnetometerSensitivity if self.magnetometer16BitMode else multiplyIdentity(0.6)
    
    def convertGyroscopeToDegreesPerSecond(self, rawGyroscope):
        return tupleMultiply(rawGyroscope, self.getGyroscopeSensitivity())
    
    def convertAccelerometerToG(self, rawAccelerometer):
        return tupleMultiply(rawAccelerometer, self.getAccelerometerSensitivity())
    
    def convertMagnetometerToMicroTesla(self, rawMagnetometer):
        return tupleMultiply(rawMagnetometer, self.getMagnetometerSensitivity())
    
    def convertTemperatureToCelsius(self, rawTemperature):
        return ((rawTemperature - self.roomTemperatureOffset) / self.temperatureSensitivity) + 21
    
    def convert(self, rawSensorData):
        """Convert raw sensor data to common units."""
        return {
            'gyroscope': self.convertGyroscopeToDegreesPerSecond(rawSensorData['gyroscope']),
            'accelerometer': self.convertAccelerometerToG(rawSensorData['accelerometer']),
            'magnetometer': self.convertMagnetometerToMicroTesla(rawSensorData['magnetometer']),
            'temperature': self.convertTemperatureToCelsius(rawSensorData['temperature'])
        }
    
    def getSensorData(self):
        return self.convert(self.getRawSensorData())
