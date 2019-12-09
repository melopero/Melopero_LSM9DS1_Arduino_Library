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
        bool useI2C(uint8_t gyroAddress = 0x6b, uint8_t magAddress = 0x1e);
        bool useSPI(uint8_t gyroChipSelectPin , uint8_t magnetometerChipSelectPin, uint32_t maxTransmissionFreq = 10 ** 7);

        uint8_t readByte(uint8_t deviceIdentifier, uint8_t registerAddress);
        uint8_t writeByte(uint8_t deviceIdentifier, uint8_t registerAddress, uint8_t value);

        uint8_t writeFlag(uint8_t deviceIdentifier, uint8_t registerAddress, uint8_t flag, uint8_t start, uint8_t length);

        void close();
        void resetSettings();

        void setGyroODR(OutputDataRate odr);
        void setAccODR(OutputDataRate odr);
        void setMagODR(MagnetometerOutputDataRate odr, MagnetometerMode mode = MagnetometerMode::CONTINUOUS);

        int8_t setGyroRange(GyroscopeRange range);
        int8_t setAccRange(AccelerometerRange range);
        int8_t setMagRange(MagnetometerRange range);

        int8_t setAccInterrupt( uint8_t xThreshold, bool xDetect, bool xDetectHigh,
                                uint8_t yThreshold, bool yDetect, bool yDetectHigh,
                                uint8_t zThreshold, bool zDetect, bool zDetectHigh,
                                bool andEventCombination = false, uint8_t samplesToRecognize = 0,
                                bool waitBeforeExitingInterrupt = false, hardwareInterrupt = false);

        int8_t setGyroInterrupt(int16_t xThreshold, bool xDetect, bool xDetectHigh,
                                int16_t yThreshold, bool yDetect, bool yDetectHigh,
                                int16_t zThreshold, bool zDetect, bool zDetectHigh,
                                bool andEventCombination = false, uint8_t samplesToRecognize = 0,
                                bool waitBeforeExitingInterrupt = false, bool decrementCounter = false,
                                bool hardwareInterrupt = false);

        int8_t setMagInterrupt( uint16_t threshold, bool xDetect, bool yDetect, bool zDetect,
                                bool interruptActiveHigh =false, bool latchInterrupt = false,
                                bool hardware_interrupt = false);

        int8_t resetInterruptSettings(bool resetAccInterrupt = true, bool resetGyroInterrupt = true, bool resetMagInterrupt = true);

        uint8_t getAccInterruptStatus();
        uint8_t getGyroInterruptStatus();
        uint8_t getMagInterruptStatus();

        void updateGyroMeasurements();
        void updateAccMeasurements();
        void updateMagMeasurements();

    private:
        uint16_t to15BitWord(int16_t value);

};

#endif // MP_LSM9DS1_H_INCLUDED
