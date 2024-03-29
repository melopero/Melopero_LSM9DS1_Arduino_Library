//Author: Leonardo La Rocca info@melopero.com

#include "Melopero_LSM9DS1.h"

//Creates the object representing the device. To actually use the device
//you have to call useSPI(gyropin, magpin) or useI2C() to set the communication
//protocol that you want to use.
Melopero_LSM9DS1::Melopero_LSM9DS1(){
    this->gyroScale = GyroscopeRange::AR_245DPS;
    this->accScale = AccelerometerRange::two_g;
    this->magScale = MagnetometerRange::four_G;
    this->i2cEnabled = false;
    this->spiEnabled = false;
}

//Configures the device (and library) to use the i2c protocol.
//If no gyroAddress or magAddress are specified the default
//used addresses will be.
int8_t Melopero_LSM9DS1::useI2C(uint8_t gyroAddress, uint8_t magAddress, TwoWire &i2c_bus){
    this->i2cEnabled = true;
    this->spiEnabled = false;

    i2c = &i2c_bus;

    this->i2cGyroAddress = gyroAddress;
    this->i2cMagAddress = magAddress;

    this->gyroIdentifier = this->i2cGyroAddress;
    this->magIdentifier = this->i2cMagAddress;

    //TODO: disable i2c flag in mag control 3.
    //      The flag is never changed by the library.

    return ErrorCodes::noError;
}

//Configures the device (and library) to use the spi protocol.
//You must specify the chip select pins used for gyro and magnetometer.
int8_t Melopero_LSM9DS1::useSPI(uint8_t gyroChipSelectPin , uint8_t magChipSelectPin, uint32_t maxTransmissionFreq){
    this->spiEnabled = true;
    this->i2cEnabled = false;

    //Setting up the CS pin.
    pinMode(gyroChipSelectPin, OUTPUT);
    pinMode(magChipSelectPin, OUTPUT);

    digitalWrite(gyroChipSelectPin, HIGH);
    digitalWrite(magChipSelectPin, HIGH);

    //Creating the SPISettings
    if ((this->spiSettings) != nullptr)
        delete (this->spiSettings);
    this->spiSettings = new SPISettings(maxTransmissionFreq, MSBFIRST, SPI_MODE0);
    SPI.begin();

    //Checking if the SPI pins are setup correctly.
    SPI.beginTransaction(*(this->spiSettings));
    //Starting communication with the gyroscope.
    digitalWrite (gyroChipSelectPin, LOW);

    //reading the WHO_AM_I register to see with which device we are communicating.
    SPI.transfer(WHO_AM_I_REG | SPI_READ_FLAG);
    uint8_t whoami = SPI.transfer(0x00);

    //Are we communicating with the gyroscope?
    if (whoami == GYRO_ID){
        this->spiGyroPin = gyroChipSelectPin;
        this->spiMagPin = magChipSelectPin;
    }
    //Are we communicating with the magnetometer?
    else if (whoami == MAG_ID){
        this->spiGyroPin = magChipSelectPin;
        this->spiMagPin = gyroChipSelectPin;
    }
    //Error!
    else {
        //Clean up and return error code
        this->spiEnabled = false;
        digitalWrite(gyroChipSelectPin, HIGH);
        SPI.endTransaction();
        return ErrorCodes::spiCommunicationError;
    }

    this->gyroIdentifier = this->spiGyroPin;
    this->magIdentifier = this->spiMagPin;

    //End the communication.
    digitalWrite(gyroChipSelectPin, HIGH);
    SPI.endTransaction();

    //enable spi read operations... ( fundamental )
    this->writeByte(this->magIdentifier, MAG_CONTROL_REG_3, 0x00);

    return ErrorCodes::noError;
}



uint8_t Melopero_LSM9DS1::readByte(uint8_t deviceIdentifier, uint8_t registerAddress){
    if (this->i2cEnabled){
        //set register pointer
        i2c->beginTransmission(deviceIdentifier);
        i2c->write(registerAddress);
        i2c->endTransmission();

        i2c->requestFrom(deviceIdentifier, 1);
        return i2c->read();
    }

    else if (this->spiEnabled){
        //Select device through CS pin.
        SPI.beginTransaction(*(this->spiSettings));
        digitalWrite(deviceIdentifier, LOW);

        //Select register in read mode.
        SPI.transfer(registerAddress | SPI_READ_FLAG);
        //Read reagister!
        uint8_t result = SPI.transfer(0x00);

        //End communication with the device.
        digitalWrite(deviceIdentifier, HIGH);
        SPI.endTransaction();
        return result;
    }
    //Error: No communication protocol specified!
    else {
        return ErrorCodes::noCommunicationProtocolSpecified;
      }
}

uint8_t Melopero_LSM9DS1::writeByte(uint8_t deviceIdentifier, uint8_t registerAddress, uint8_t value){
    uint8_t bytesWritten = 0;
    if (this->i2cEnabled){
        //set register pointer
        i2c->beginTransmission(deviceIdentifier);
        i2c->write(registerAddress);
        bytesWritten = i2c->write(value);
        i2c->endTransmission();
    }
    else if (this->spiEnabled){
        //Select device through CS pin.
        SPI.beginTransaction(*(this->spiSettings));
        digitalWrite(deviceIdentifier, LOW);

        //Select register in write mode.
        SPI.transfer(registerAddress);
        //Write value to register.
        SPI.transfer(value);

        //End communication with the device.
        digitalWrite(deviceIdentifier, HIGH);
        SPI.endTransaction();
        bytesWritten = 1;
    }
    else {
        return ErrorCodes::noCommunicationProtocolSpecified;
    }

    return bytesWritten;
}

/**Writes the flag bits in the spcified register at the specified position.\n
 * start : start of the flag bits, the MSB has an index of 8 and the lsb has an index of 1.\n
 * length : how many bits of the flag to set.\n
 * Example:
 * register value = 0b11110000\n
 * flag value = 0b0011\n
 * start = 6\n
 * length = 4\n
 * result = 0b11001100
 */
uint8_t Melopero_LSM9DS1::writeFlag(uint8_t deviceIdentifier, uint8_t registerAddress, uint8_t flag, uint8_t start, uint8_t length){
    uint8_t _end = start - length;
    uint8_t regContent = this->readByte(deviceIdentifier, registerAddress);


    uint8_t msb = regContent >> start << start;
    uint8_t flagBits = flag << _end;
    uint8_t lsb = regContent % (1 << _end);
    return this->writeByte(deviceIdentifier, registerAddress, msb | flagBits | lsb);
}


void Melopero_LSM9DS1::close(){
    if (this->i2cEnabled){
        //Nothing to do here
    }
    else if (this->spiEnabled){
        SPI.end();
        delete (this->spiSettings);
        //GPIO cleanup!
    }
}


int8_t Melopero_LSM9DS1::resetSettings(){
    int8_t bytesWritten = 0;
    bytesWritten = this->writeFlag(this->gyroIdentifier, CONTROL_REG_8, 0b101, 3, 3);
    if (bytesWritten == 0) return -1;
    bytesWritten = this->writeFlag(this->magIdentifier, MAG_CONTROL_REG_2, 0b11, 4, 2);
    if (bytesWritten == 0) return -2;
    return this->resetInterruptSettings();
}

//Sets the gyroscope's output data rate.
int8_t Melopero_LSM9DS1::setGyroODR(uint8_t odr){
    return this->writeFlag(this->gyroIdentifier, GYRO_CONTROL_REG_1, odr, 8, 3) > 0 ? 0 : -1;
}

//Sets the Accelerometer's output data rate.
int8_t Melopero_LSM9DS1::setAccODR(uint8_t odr){
    return this->writeFlag(this->gyroIdentifier, ACC_CONTROL_REG_6, odr, 8, 3) > 0 ? 0 : -1;
}

//Sets the magnetometer's output data rate and mode.
int8_t Melopero_LSM9DS1::setMagODR(uint8_t magOdr, uint8_t magMode){
    int8_t bytesWritten = 0;
    bytesWritten = this->writeFlag(this->magIdentifier, MAG_CONTROL_REG_3, magMode, 2, 2);
    if (bytesWritten == 0) return -2;
    bytesWritten = this->writeFlag(this->magIdentifier, MAG_CONTROL_REG_1, magOdr, 5, 3);
    return bytesWritten > 0 ? 0 : -2;
}

int8_t Melopero_LSM9DS1::setGyroRange(float gyroRange){
    uint8_t flag = 0b00;
    if (gyroRange == GyroscopeRange::AR_245DPS)
        flag = 0b00;
    else if (gyroRange == GyroscopeRange::AR_500DPS)
        flag = 0b01;
    else if (gyroRange == GyroscopeRange::AR_2000DPS)
        flag = 0b11;
    else
        return ErrorCodes::invalidDataFormatOrRange;

    this->gyroScale = gyroRange;
    int8_t bytesWritten = 0;
    bytesWritten = this->writeFlag(this->gyroIdentifier, GYRO_CONTROL_REG_1, flag, 5, 2);
    return bytesWritten == 0 ? -1 : 0;
}


int8_t Melopero_LSM9DS1::setAccRange(float accRange){
    uint8_t flag = 0b00;
    if (accRange == AccelerometerRange::two_g)
        flag = 0b00;
    else if (accRange == AccelerometerRange::four_g)
        flag = 0b10;
    else if (accRange == AccelerometerRange::eight_g)
        flag = 0b11;
    else if (accRange == AccelerometerRange::sixteen_g)
        flag = 0b01;
    else
        return ErrorCodes::invalidDataFormatOrRange;

    this->accScale = accRange;
    int8_t bytesWritten = 0;
    bytesWritten = this->writeFlag(this->gyroIdentifier, ACC_CONTROL_REG_6, flag, 5, 2);
    return bytesWritten == 0 ? -1 : 0;
}


int8_t Melopero_LSM9DS1::setMagRange(float magRange){
    uint8_t flag = 0b00;
    if (magRange == MagnetometerRange::four_G)
        flag = 0b00;
    else if (magRange == MagnetometerRange::eight_G)
        flag = 0b01;
    else if (magRange == MagnetometerRange::twelve_G)
        flag = 0b10;
    else if (magRange == MagnetometerRange::sixteen_G)
        flag = 0b11;
    else
        return ErrorCodes::invalidDataFormatOrRange;

    this->magScale = magRange;
    int8_t bytesWritten = 0;
    bytesWritten = this->writeFlag(this->magIdentifier, MAG_CONTROL_REG_2, flag, 7, 2);
    return bytesWritten == 0 ? -2 : 0;
}

int8_t Melopero_LSM9DS1::resetInterruptSettings(bool resetAccInterrupt, bool resetGyroInterrupt, bool resetMagInterrupt){
    int8_t bytesWritten = 0;
    if (resetAccInterrupt){
        bytesWritten = this->writeByte(this->gyroIdentifier, ACC_INT_CFG_REG, 0x00);
        if (bytesWritten == 0) return -1;
    }
    if (resetGyroInterrupt){
        bytesWritten = this->writeByte(this->gyroIdentifier, GYRO_INT_CFG_REG, 0x00);
        if (bytesWritten == 0) return -1;
    }
    if (resetMagInterrupt){
        bytesWritten = this->writeByte(this->magIdentifier, MAG_INT_CFG_REG, 0x00);
        if (bytesWritten == 0) return -2;
    }

    return 0;
}


/**
 * @param x/y/zThreshold When a measure exceeds (or is lower) than the threshold
 *        value it triggers an interrupt. This values are expressed in g's (gravity = 9.8 ms^-2)
 *        The given value will be converted to an unsigned 8 bit int through this formula :
 *        rawThreshold = (uint8_t) (Actual Threshold * 255.0f / Acc Range)
 * @param x/y/zDetect wether the given axis should be detected.
 * @param x/y/zDetectHigh if True detects samples that are over the threshold.
 * @param andEventCombination if True all specified interrupt conditions must be
 *        triggered to generate an interrupt
 * @param samplesToRecognize the number of samples that trigger the interrupt to measure
 *        before actually triggering the interrupt
 * @param waitBeforeExitingInterrupt the number of samples to measure before exiting the interrupt
 * @param hardwareInterrupt if True generates an hardware interrupt on the int1 pin
 */
int8_t Melopero_LSM9DS1::setAccInterrupt( float xThreshold, bool xDetect, bool xDetectHigh,
                                    float yThreshold, bool yDetect, bool yDetectHigh,
                                    float zThreshold, bool zDetect, bool zDetectHigh,
                                    bool andEventCombination, uint8_t samplesToRecognize,
                                    bool waitBeforeExitingInterrupt, bool hardwareInterrupt){

    if (samplesToRecognize > 0x7F) return -5;

    //and/or event combination + reset other settings
    this->writeByte(this->gyroIdentifier, ACC_INT_CFG_REG, andEventCombination ? 0x80 : 0x00);

    float accRange = -1;
    if (accScale == AccelerometerRange::two_g)
        accRange = 2;
    else if (accScale == AccelerometerRange::four_g)
        accRange = 4;
    else if (accScale == AccelerometerRange::eight_g)
        accRange = 8;
    else if (accScale == AccelerometerRange::sixteen_g)
        accRange = 16;
    else
        return ErrorCodes::invalidDataFormatOrRange;

    //real interrupt : raw ths = float ths * 255 / acc range
    if (xDetect){
        uint8_t rawXThreshold = xThreshold == 0 ? 0 : (uint8_t) (xThreshold * 255.0f / accRange);
        this->writeByte(this->gyroIdentifier, ACC_X_INT_THR_REG, rawXThreshold);
        this->writeFlag(this->gyroIdentifier, ACC_INT_CFG_REG, xDetectHigh ? 0b10 : 0b01, 2, 2);
    }
    if (yDetect){
        uint8_t rawYThreshold = yThreshold == 0 ? 0 : (uint8_t) (yThreshold * 255.0f / accRange);
        this->writeByte(this->gyroIdentifier, ACC_Y_INT_THR_REG, rawYThreshold);
        this->writeFlag(this->gyroIdentifier, ACC_INT_CFG_REG, yDetectHigh ? 0b10 : 0b01, 4, 2);
    }
    if (zDetect){
        uint8_t rawZThreshold = zThreshold == 0 ? 0 : (uint8_t) (zThreshold * 255.0f / accRange);
        this->writeByte(this->gyroIdentifier, ACC_Z_INT_THR_REG, rawZThreshold);
        this->writeFlag(this->gyroIdentifier, ACC_INT_CFG_REG, zDetectHigh ? 0b10 : 0b01, 6, 2);
    }

    this->writeByte(this->gyroIdentifier, ACC_INT_DUR_REG, samplesToRecognize | (waitBeforeExitingInterrupt ? 0x80 : 0x00));
    this->writeFlag(this->gyroIdentifier, INT1_CTRL_REG, hardwareInterrupt? 1 : 0, 7, 1);

    return ErrorCodes::noError;
}

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
int8_t Melopero_LSM9DS1::setGyroInterrupt(float xThreshold, bool xDetect, bool xDetectHigh,
                        float yThreshold, bool yDetect, bool yDetectHigh,
                        float zThreshold, bool zDetect, bool zDetectHigh,
                        bool andEventCombination, uint8_t samplesToRecognize,
                        bool waitBeforeExitingInterrupt, bool decrementCounter,
                        bool hardwareInterrupt){

    if (samplesToRecognize > 0x7F)
        return ErrorCodes::invalidDataFormatOrRange;

    //reset interrupt
    this->writeByte(this->gyroIdentifier, GYRO_INT_CFG_REG, andEventCombination ? 0x80 : 0x00);

    //x_threshold = int(x_threshold * 1000 / self.gyro_scale)

    if (xDetect){
        int16_t rawXThreshold = (int16_t) ( xThreshold * 1000 / this->gyroScale);
        uint16_t _15_bitXThreshold = this->to15BitWord(rawXThreshold);
        if (_15_bitXThreshold == 0x8000)
            return ErrorCodes::invalidDataFormatOrRange;
        _15_bitXThreshold |= decrementCounter ? 0x8000 : 0x0000;
        this->writeByte(this->gyroIdentifier, GYRO_X_INT_THR_REG, (uint8_t) (_15_bitXThreshold / (1 << 8)));
        this->writeByte(this->gyroIdentifier, GYRO_X_INT_THR_REG + 1, (uint8_t) (_15_bitXThreshold % (1 << 7)));
        this->writeFlag(this->gyroIdentifier, GYRO_INT_CFG_REG, xDetectHigh ? 0b10 : 0b01, 2, 2);
    }

    if (yDetect){
        int16_t rawYThreshold = (int16_t) ( yThreshold * 1000 / this->gyroScale);
        uint16_t _15_bitYThreshold = this->to15BitWord(rawYThreshold);
        if (_15_bitYThreshold == 0x8000)
            return ErrorCodes::invalidDataFormatOrRange;
        this->writeByte(this->gyroIdentifier, GYRO_Y_INT_THR_REG, (uint8_t) (_15_bitYThreshold / (1 << 8)));
        this->writeByte(this->gyroIdentifier, GYRO_Y_INT_THR_REG + 1, (uint8_t) (_15_bitYThreshold % (1 << 7)));
        this->writeFlag(this->gyroIdentifier, GYRO_INT_CFG_REG, yDetectHigh ? 0b10 : 0b01, 4, 2);
    }

    if (zDetect){
        int16_t rawZThreshold = (int16_t) ( zThreshold * 1000 / this->gyroScale);
        uint16_t _15_bitZThreshold = this->to15BitWord(rawZThreshold);
        if (_15_bitZThreshold == 0x8000)
            return ErrorCodes::invalidDataFormatOrRange;
        this->writeByte(this->gyroIdentifier, GYRO_Z_INT_THR_REG, (uint8_t) (_15_bitZThreshold / (1 << 8)));
        this->writeByte(this->gyroIdentifier, GYRO_Z_INT_THR_REG + 1, (uint8_t) (_15_bitZThreshold % (1 << 7)));
        this->writeFlag(this->gyroIdentifier, GYRO_INT_CFG_REG, zDetectHigh ? 0b10 : 0b01, 6, 2);
    }

    this->writeByte(this->gyroIdentifier, GYRO_INT_DUR_REG, samplesToRecognize | (waitBeforeExitingInterrupt ? 0x80 : 0x00));
    this->writeFlag(this->gyroIdentifier, INT1_CTRL_REG, hardwareInterrupt ? 1 : 0, 8, 1);

    return ErrorCodes::noError;
}

uint16_t Melopero_LSM9DS1::to15BitWord(int16_t value){
    if (value < -16384 && value > 16383) //minValue = -16384 maxValue = 16383
        return 0x8000; //threshold out of range error

    if (value > 0)
        return value;
    else {
        uint16_t convertedValue = -value; //get absolute value
        convertedValue = ~convertedValue; // flip all the bits
        convertedValue &= 0x7FFF; // flip msb to 0 (15 bit word not 16 !)
        convertedValue += 1; // add one to have the correct value
        return convertedValue;
    }
}

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
int8_t Melopero_LSM9DS1::setMagInterrupt( float threshold, bool xDetect, bool yDetect, bool zDetect,
                                    bool interruptActiveHigh, bool latchInterrupt,
                                    bool hardwareInterrupt){
    uint8_t flag = 0x00;
    flag |= xDetect << 7;
    flag |= yDetect << 6;
    flag |= zDetect << 5;
    flag |= interruptActiveHigh << 2;
    flag |= latchInterrupt << 1;
    flag |= hardwareInterrupt ? 1 : 0;

    this->writeByte(this->magIdentifier, MAG_INT_CFG_REG, flag);

    if (xDetect || yDetect || zDetect){
        if (threshold < 0)
            threshold = -threshold; // threshold out of range
        uint16_t uint15Threshold = (uint16_t) (threshold * 1000 / this->magScale);
        if (uint15Threshold > 0x7FFF)
            return ErrorCodes::invalidDataFormatOrRange; // threshold out of range
        this->writeByte(this->magIdentifier, MAG_INT_THR_REG, (uint8_t) (uint15Threshold % (1<<7)));
        this->writeByte(this->magIdentifier, MAG_INT_THR_REG + 1, (uint8_t) (uint15Threshold / (1 << 8)));
    }

    return ErrorCodes::noError;
}

//Returns the content of the interrupt status register of the accelerometer.
uint8_t Melopero_LSM9DS1::getAccInterruptStatus(){
    return this->readByte(this->gyroIdentifier, ACC_INT_SRC_REG);
}

//Returns the content of the interrupt status register of the gyroscope.
uint8_t Melopero_LSM9DS1::getGyroInterruptStatus(){
    return this->readByte(this->gyroIdentifier, GYRO_INT_SRC_REG);
}

//Returns the content of the interrupt status register of the magnetometer.
uint8_t Melopero_LSM9DS1::getMagInterruptStatus(){
    return this->readByte(this->magIdentifier, MAG_INT_SRC_REG);
}

//Updates the measurements in the gyroRawMeasurements and gyroMeasurements arrays
void Melopero_LSM9DS1::updateGyroMeasurements(){
    //read gyro output values: 16 bit integer in two's complement , byteorder = little (msb is last)
    uint8_t axisRegs[3] = {GYRO_X_REG, GYRO_Y_REG, GYRO_Z_REG};
    for (uint8_t i = 0; i <= 2; i++){
        this->gyroRawMeasurements[i] = this->readByte(this->gyroIdentifier, axisRegs[i]);
        this->gyroRawMeasurements[i] += this->readByte(this->gyroIdentifier, axisRegs[i] + 1) << 8;

        this->gyroMeasurements[i] = ((float) this->gyroRawMeasurements[i]) / 1000.0f * this->gyroScale;
    }
}

//Updates the measurements in the accRawMeasurements and accMeasurements arrays
void Melopero_LSM9DS1::updateAccMeasurements(){
    uint8_t axisRegs[3] = {ACC_X_REG, ACC_Y_REG, ACC_Z_REG};
    for (uint8_t i = 0; i <= 2; i++){
        this->accRawMeasurements[i] = this->readByte(this->gyroIdentifier, axisRegs[i]);
        this->accRawMeasurements[i] += this->readByte(this->gyroIdentifier, axisRegs[i] + 1) << 8;

        this->accMeasurements[i] = ((float) this->accRawMeasurements[i]) / 1000.0f * this->accScale;
    }
}

//Updates the measurements in the magRawMeasurements and magMeasurements arrays
void Melopero_LSM9DS1::updateMagMeasurements(){
    uint8_t axisRegs[3] = {MAG_X_REG, MAG_Y_REG, MAG_Z_REG};
    for (uint8_t i = 0; i <= 2; i++){
        this->magRawMeasurements[i] = this->readByte(this->magIdentifier, axisRegs[i]);
        this->magRawMeasurements[i] += this->readByte(this->magIdentifier, axisRegs[i] + 1) << 8;

        this->magMeasurements[i] = ((float) this->magRawMeasurements[i]) / 1000.0f * this->magScale;
    }
}

String Melopero_LSM9DS1::getErrorString(int8_t errorCode){
    if (errorCode >= ErrorCodes::noError) return "No error / Number of bytes written";
    if (errorCode == ErrorCodes::gyroCommunicationError) return "Communication Error with the gyroscope sensor.";
    if (errorCode == ErrorCodes::magCommunicationError) return "Communication Error with the magnetometer sensor.";
    if (errorCode == ErrorCodes::i2cCommunicationError) return "I2C communication channel error";
    if (errorCode == ErrorCodes::spiCommunicationError) return "SPI communication channel error";
    if (errorCode == ErrorCodes::invalidDataFormatOrRange) return "Invalid data format/range (input data out of range)";
    if (errorCode == ErrorCodes::noCommunicationProtocolSpecified) return "No communication protocol specified, please select one with useI2C or useSPI";
    else return "Unknown error";
}
