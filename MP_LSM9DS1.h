#ifndef MP_LSM9DS1_H_INCLUDED
#define MP_LSM9DS1_H_INCLUDED

#if __has_include("Wire.h")
#include "Wire.h"
#define I2C_AVAILABLE 1
#else
#define I2C_AVAILABLE 0
#pragma message ( "warning : You need Wire.h in order to use the I2C protocol." )
#endif // __has_include i2c

#if __has_include("SPI.h")
#include "SPI.h"
#define SPI_AVAILABLE 1
#else
#define SPI_AVAILABLE 0
#pragma message ( "warning : You need SPI.h in order to use the SPI protocol." )
#endif // __has_include spi

#define WHO_AM_I_REG 0x0F
#define GYRO_ID 0b01101000
#define MAG_ID 0b00111101

#define SPI_READ_FLAG 0b10000000

#define CONTROL_REG_8 0x22

#define GYRO_CONTROL_REG_1 0x10

#define ACC_CONTROL_REG_5 0x1F
#define ACC_CONTROL_REG_6 0x20

#define MAG_CONTROL_REG_1 0x20
#define MAG_CONTROL_REG_2 0x21
#define MAG_CONTROL_REG_3 0x22

#define TEMPERATURE_REG 0x15

#define GYRO_X_REG = 0x18
#define GYRO_Y_REG = 0x1A
#define GYRO_Z_REG = 0x1C

#define ACC_X_REG = 0x28
#define ACC_Y_REG = 0x2A
#define ACC_Z_REG = 0x2C

#define MAG_X_REG = 0x28
#define MAG_Y_REG = 0x2A
#define MAG_Z_REG = 0x2C


#define INT1_CTRL_REG = 0x0C
#define INT2_CTRL_REG = 0x0D

#define GYRO_INT_CFG_REG = 0x30
#define GYRO_X_INT_THR_REG = 0x31
#define GYRO_Y_INT_THR_REG = 0x33
#define GYRO_Z_INT_THR_REG = 0x35
#define GYRO_INT_DUR_REG = 0x37
#define GYRO_INT_SRC_REG = 0x14

#define ACC_INT_CFG_REG = 0x06
#define ACC_X_INT_THR_REG = 0x07
#define ACC_Y_INT_THR_REG = 0x08
#define ACC_Z_INT_THR_REG = 0x09
#define ACC_INT_DUR_REG = 0x0A
#define ACC_INT_SRC_REG = 0x26

#define MAG_INT_CFG_REG = 0x30
#define MAG_INT_THR_REG = 0x32
#define MAG_INT_SRC_REG = 0x31

enum class GyroscopeRange : float {
    AR_245DPS = 8.75f,
    AR_500DPS = 17.5f,
    AR_2000DPS = 70f
};

enum class AccelerometerRange : float {
    2_g = 0.061f,
    4_g = 0.122f,
    8_g = 0.244f,
    16_g = 0.732f
};

enum class MagnetometerRange : float {
    4_G = 0.14f,
    8_G = 0.29f,
    12_G = 0.43f,
    16_G = 0.58f
};

enum class OutputDataRate : uint8_t {
    POWER_DOWN = 0b000,
    10_Hz = 0b001,
    50_Hz = 0b010,
    119_Hz = 0b011,
    238_Hz = 0b100,
    476_Hz = 0b101,
    952_Hz = 0b110
};

enum class MagnetometerOutputDataRate : uint8_t {
    0_625_Hz = 0b000,
    1_25_Hz = 0b001,
    2_5_Hz = 0b010,
    5_Hz = 0b011,
    10_Hz = 0b100,
    20_Hz = 0b101,
    40_Hz = 0b110,
    80_Hz = 0b111
};

enum class MagnetometerMode : uint8_t {
    CONTINUOUS = 0b00,
    SINGLE = 0b01,
    POWER_DOWN = 0b10
};

class MP_LSM9DS1 {

    //instance variables
    public:
        uint8_t i2cGyroAddress;
        uint8_t i2cMagAddress;
        uint8_t spiGyroPin;
        uint8_t spiMagPin;
        uint8_t gyroIdentifier;
        uint8_t magIdentifier;
        SPISettings spiSettings;
        GyroscopeRange gyroScale;
        AccelerometerRange accScale;
        MagnetometerRange magScale;
        bool i2cEnabled;
        bool spiEnabled;

        int16_t accRawMeasurements[3];
        int16_t gyroRawMeasurements[3];
        int16_t magRawMeasurements[3];

        float accMeasurements[3]; /**Contains the last measured values for the accelerometer, expressed in gravities (g's, 1 g = 9.8 ms-2) */
        float gyroMeasurements[3]; /**Contains the last updated values for the gyroscope, expressed in degrees per second (dps) */
        float magMeasurements[3]; /**Contains the last measured values for the magntometer, expressed in Gauss */

    //constructor
    public:
        MP_LSM9DS1();

    //methods
    public:
        int8_t useI2C(uint8_t gyroAddress = 0x6b, uint8_t magAddress = 0x1e);
        int8_t useSPI(uint8_t gyroChipSelectPin , uint8_t magnetometerChipSelectPin, uint32_t maxTransmissionFreq = 10 ** 7);

        uint8_t readByte(uint8_t deviceIdentifier, uint8_t registerAddress);
        uint8_t writeByte(uint8_t deviceIdentifier, uint8_t registerAddress, uint8_t value);

        /**Writes the flag bits in the spcified register at the specified position.\n
         * start : start of the flag bits, the MSB has an index of 8 and the lsb has an index of 1.\n
         * length : how many bits of the flag to set.\n
         * Example:
         * register value = 0b11110000\n
         * flag value = 0b0011\n
         * start = 6\n
         * length = 4\n
         * result = 0b11001100 */
        uint8_t writeFlag(uint8_t deviceIdentifier, uint8_t registerAddress, uint8_t flag, uint8_t start, uint8_t length);

        void close();
        int8_t resetSettings();


        /** Sets the accelerometer output-data-rate. */
        int8_t setGyroODR(OutputDataRate odr);


        /** Sets the accelerometer output-data-rate. */
        int8_t setAccODR(OutputDataRate odr);


        /** Sets the magnetometer output-data-rate and operating mode.
         *  (continuous, single conversion, power down). */
        int8_t setMagODR(MagnetometerOutputDataRate odr, MagnetometerMode mode = MagnetometerMode::CONTINUOUS);

        int8_t setGyroRange(GyroscopeRange range);
        int8_t setAccRange(AccelerometerRange range);
        int8_t setMagRange(MagnetometerRange range);

        /**
         * @param x/y/zThreshold When a measure exceeds (or is lower) than the threshold
         *        value it triggers an interrupt. This values are expressed in g's (gravity = 9.8 ms^-2)
         *        The given value will be converted to an unsigned 8 bit int through this formula :
         *        rawThreshold = (uint8_t) (acceleratorRange / Threshold)
         * @param x/y/zDetect wether the given axis should be detected.
         * @param x/y/zDetectHigh if True detects samples that are over the threshold.
         * @param andEventCombination if True all specified interrupt conditions must be
         *        triggered to generate an interrupt
         * @param samplesToRecognize the number of samples that trigger the interrupt to measure
         *        before actually triggering the interrupt
         * @param waitBeforeExitingInterrupt the number of samples to measure before exiting the interrupt
         * @param hardwareInterrupt if True generates an hardware interrupt on the int1 pin
         */
        int8_t setAccInterrupt( float xThreshold, bool xDetect, bool xDetectHigh,
                                float yThreshold, bool yDetect, bool yDetectHigh,
                                float zThreshold, bool zDetect, bool zDetectHigh,
                                bool andEventCombination = false, uint8_t samplesToRecognize = 0,
                                bool waitBeforeExitingInterrupt = false, hardwareInterrupt = false);

        /**
         * @param x/y/zThreshold the interrupt threshold value expressed in dps (degrees per second).
         *        The given values will be converted to a signed 15 bit integer through this formula :
         *        rawThreshold = to15Bit((int16_t) ( Threshold * 1000 / this->gyroScale))
         * @param x/y/zDetect wether the given axis should be detected.
         * @param x/y/zDetectHigh if True detects samples that are over the threshold.
         * @param andEventCombination if True all specified interrupt conditions must be
         *        triggered to generate an interrupt
         * @param samplesToRecognize the number of samples that trigger the interrupt to measure
         *        before actually triggering the interrupt
         * @param waitBeforeExitingInterrupt the number of samples to measure before exiting the interrupt
         * @param decrementCounter
         * @param hardwareInterrupt if True generates an hardware interrupt on the int1 pin
         */
        int8_t setGyroInterrupt(float xThreshold, bool xDetect, bool xDetectHigh,
                                float yThreshold, bool yDetect, bool yDetectHigh,
                                float zThreshold, bool zDetect, bool zDetectHigh,
                                bool andEventCombination = false, uint8_t samplesToRecognize = 0,
                                bool waitBeforeExitingInterrupt = false, bool decrementCounter = false,
                                bool hardwareInterrupt = false);

        /**
         * @param threshold If measurement exceeds on positively (or negatively) the
         *        threshold, an interrupt is generated, This value is expressed in gauss and must be positive.
         *        The given values will be converted to an unsigned 15 bit integer with this formula:
         *        uint16_t uint15Threshold = (uint16_t) (threshold * 1000 / this->magScale);
         * @param x/y/zDetect if True detects samples from the specified axes
         * @param interruptActiveHigh if True the interrupt signal on the INT_M pin is HIGH
         * @param latchInterrupt Once latched, the INT_M pin remains in the same
                  state until getMagInterrupt is called.
         * @param hardwareInterrupt if True generates an hardware interrupt on the INT_M pin
         */
        int8_t setMagInterrupt( float threshold, bool xDetect, bool yDetect, bool zDetect,
                                bool interruptActiveHigh =false, bool latchInterrupt = false,
                                bool hardware_interrupt = false);

        int8_t resetInterruptSettings(bool resetAccInterrupt = true, bool resetGyroInterrupt = true, bool resetMagInterrupt = true);

        uint8_t getAccInterruptStatus();
        uint8_t getGyroInterruptStatus();
        uint8_t getMagInterruptStatus();

        void updateGyroMeasurements();
        void updateAccMeasurements();
        void updateMagMeasurements();

        char[] getErrorString(int8_t errorCode);

    private:
        uint16_t to15BitWord(int16_t value);

};

#endif // MP_LSM9DS1_H_INCLUDED
