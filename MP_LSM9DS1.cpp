#include "MP_LSM9DS1.h"

//TODO: set interrupts functions accept only raw parameters (except for the mag one)
//TODO: update with normal paramter!(look at python code)

MP_LSM9DS1::MP_LSM9DS1(){
    this->gyroScale = GyroscopeRange::AR_245DPS;
    this->accScale = AccelerometerRange::2_g;
    this->magScale = MagnetometerRange::4_G;
    this->i2cEnabled = false;
    this->spiEnabled = false;

    this->accRawMeasurements = {0,0,0};
    this->gyroRawMeasurements = {0,0,0};
    this->magRawMeasurements = {0,0,0};

    this->accMeasurements = {0f,0f,0f};
    this->gyroMeasurements = {0f,0f,0f};
    this->magMeasurements = {0f,0f,0f};
}

int8_t MP_LSM9DS1::useI2C(uint8_t gyroAddress = 0x6b, uint8_t magAddress = 0x1e){
        if (I2C_AVAILABLE){
            this->i2cEnabled = true;
            this->spiEnabled = false;

            Wire.begin();
            this->i2cGyroAddress = gyroAddress;
            this->i2cMagAddress = magAddress;

            this->gyroIdentifier = this->i2cGyroAddress;
            this->magIdentifier = this->i2cMagAddress;
            //enable i2c in control registers mag control 3 ...
            return 0;
        }
        else {
            return -3;
        }
}

int8_t MP_LSM9DS1::useSPI(uint8_t gyroChipSelectPin , uint8_t magChipSelectPin, uint32_t maxTransmissionFreq = 10 ** 7){
    if (SPI_AVAILABLE){
        this->spiEnabled = true;
        this->i2cEnabled = false;

        pinMode(gyroChipSelectPin, OUTPUT);
        pinMode(magChipSelectPin, OUTPUT);

        this->spiSettings = SPISettings(maxTransmissionFreq, MSBFIRST, SPI_MODE2);
        SPI.begin();


        SPI.beginTransaction(this->spiSettings);
        digitalWrite (gyroChipSelectPin, LOW);

        uint8_t whoami = (SPI.transfer16(WHO_AM_I_REG | SPI_READ_FLAG << 8)) % (1 << 8);

        if (whoami == GYRO_ID){
            this->spiGyroPin = gyroChipSelectPin;
            this->spiMagPin = magChipSelectPin;
        }
        else if (whoami == MAG_ID){
            this->spiGyroPin = magChipSelectPin;
            this->spiMagPin = gyroChipSelectPin;
        }
        else {
            return -4;
        }

        this->gyroIdentifier = this->spiGyroPin;
        this->magIdentifier = this->spiMagPin;

        digitalWrite(gyroChipSelectPin, HIGH);
        SPI.endTransaction();

        //enable spi read operations
        this->writeFlagMag(MAG_CONTROL_REG_3, 0b1, 3, 1)
        //TODO: disable i2c ?

        return 0;
    }
    else {
        return -4;
    }
}



uint8_t MP_LSM9DS1::readByte(uint8_t deviceIdentifier, uint8_t registerAddress){
    if (this->i2cEnabled){
        //set register pointer
        Wire.beginTransmission(deviceIdentifier);
        Wire.write(registerAddress);
        Wire.endTransmission();

        Wire.requestFrom(deviceIdentifier, 1);
        return Wire.read();
    }

    else if (this->spiEnabled){
            SPI.beginTransaction(this->spiSettings);
            digitalWrite(deviceIdentifier, LOW);
            uint16_t registerQuery = registerAddress | SPI_READ_FLAG << 8;
            uint16_t result = SPI.transfer16(registerQuery);
            digitalWrite(deviceIdentifier, HIGH);
            SPI.endTransaction();
            return (uint8_t) (result % (1 << 8));
    }
    else {
        return 0;    }
}

uint8_t MP_LSM9DS1::writeByte(uint8_t deviceIdentifier, uint8_t registerAddress, uint8_t value){
    uint8_t bytesWritten = 0;
    if (this->i2cEnabled){
        //set register pointer
        Wire.beginTransmission(deviceIdentifier);
        Wire.write(registerAddress);
        bytesWritten = Wire.write(value);
        Wire.endTransmission();
    }
    else if (this->spiEnabled){
        SPI.beginTransaction(this->spiSettings);
        digitalWrite(deviceIdentifier, LOW)
        uint16_t data = registerAddress << 8 | value;
        SPI.transfer16(data);
        digitalWrite(deviceIdentifier, HIGH)
        SPI.endTransaction();
        bytesWritten = 1;
    }
    else {
        return bytesWritten;
    }

    return bytesWritten;
}

uint8_t MP_LSM9DS1::writeFlag(uint8_t deviceIdentifier, uint8_t registerAddress, uint8_t flag, uint8_t start, uint8_t length){
    uint8_t end = start - length;
    uint8_t regContent = this->readByte(deviceIdentifier, registerAddress);


    uint8_t msb = regContent >> start << start;
    uint8_t flagBits = flag << end;
    uint8_t lsb = regContent % (2 ** end);
    return this->writeByte(deviceIdentifier, registerAddress, msb | flag_bits | lsb);
}


void MP_LSM9DS1::close(){
    if (this->i2cEnabled){    }
    else if (this->spiEnabled){
        SPI.end();
        //GPIO cleanup!    }
}

int8_t MP_LSM9DS1::resetSettings(){
    int8_t bytesWritten = 0;
    bytesWritten = this->writeFlag(this->gyroIdentifier, CONTROL_REG_8, 0b101, 3, 3);
    if (bytesWritten == 0) return -1;
    bytesWritten = this->writeFlag(this->magIdentifier, MAG_CONTROL_REG_2, 0b11, 4, 2);
    if (bytesWritten == 0) return -2;
    return this->resetInterruptSettings();
}

int8_t MP_LSM9DS1::setGyroODR(OutputDataRate odr){
    return this->writeFlag(this->gyroIdentifier, GYRO_CONTROL_REG_1, odr, 8, 3) > 0 ? 0 : -1;
}

int8_t MP_LSM9DS1::setAccODR(OutputDataRate odr){
    return this->writeFlag(this->gyroIdentifier, ACC_CONTROL_REG_6, odr, 8, 3) > 0 ? 0 : -1;
}

int8_t MP_LSM9DS1::setMagODR(MagnetometerOutputDataRate odr, MagnetometerMode mode = MagnetometerMode::CONTINUOUS){
    int8_t bytesWritten = 0;
    bytesWritten = this->writeFlag(this.magIdentifier, MAG_CONTROL_REG_3, mode, 2, 2);
    if (bytesWritten == 0) return -2;
    bytesWritten = this->writeFlag(this->magIdentifier, MAG_CONTROL_REG_1, odr, 5, 3);
    return bytesWritten > 0 ? 0 : -2;
}

int8_t MP_LSM9DS1::setGyroRange(GyroscopeRange range){
    uint8_t flag = 0b00;
    if (range == GyroscopeRange::AR_245DPS)
        flag = 0b00;
    else if (range == GyroscopeRange::AR_500DPS)
        flag = 0b01;
    else if (range == GyroscopeRange::AR_2000DPS)
        flag = 0b11;
    else
        return -5;

    this->gyroScale = range;
    int8_t bytesWritten = 0;
    bytesWritten = this->writeFlag(this->gyroIdentifier, GYRO_CONTROL_REG_1, flag, 5, 2);
    return bytesWritten == 0 ? -1 : 0;
}


int8_t MP_LSM9DS1::setAccRange(AccelerometerRange range){
    uint8_t flag = 0b00;
    if (range == AccelerometerRange::2_g)
        flag = 0b00;
    else if (range == AccelerometerRange::4_g)
        flag = 0b10;
    else if (range == AccelerometerRange::8_g)
        flag = 0b11;
    else if (range == AccelerometerRange::16_g)
        flag = 0b01;
    else
        return -5;

    this->accScale = range;
    int8_t bytesWritten = 0;
    bytesWritten = this->writeFlag(this->gyroIdentifier, ACC_CONTROL_REG_6, flag, 5, 2);
    return bytesWritten == 0 ? -1 : 0;
}


int8_t MP_LSM9DS1::setMagRange(MagnetometerRange range){
    uint8_t flag = 0b00;
    if (range == MagnetometerRange::4_G)
        flag = 0b00;
    else if (range == MagnetometerRange::8_G)
        flag = 0b01;
    else if (range == MagnetometerRange::12_G)
        flag = 0b10;
    else if (range == MagnetometerRange::16_G)
        flag = 0b11;
    else
        return -5;

    this->magScale = range;
    int8_t bytesWritten = 0;
    bytesWritten = this->writeFlag(this->magIdentifier, MAG_CONTROL_REG_2, flag, 7, 2);
    return bytesWritten == 0 ? -2 : 0;
}

int8_t MP_LSM9DS1::resetInterruptSettings(bool resetAccInterrupt = true, bool resetGyroInterrupt = true, bool resetMagInterrupt = true){
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


int8_t MP_LSM9DS1::setAccInterrupt( float xThreshold, bool xDetect, bool xDetectHigh,
                                    float yThreshold, bool yDetect, bool yDetectHigh,
                                    float zThreshold, bool zDetect, bool zDetectHigh,
                                    bool andEventCombination = false, uint8_t samplesToRecognize = 0,
                                    bool waitBeforeExitingInterrupt = false, hardwareInterrupt = false){

    if (samplesToRecognize > 0x7F) return -5;

    #and/or event combination + reset other settings
    this->writeByte(this->gyroIdentifier, ACC_INT_CFG_REG, andEventCombination ? 0x80 : 0x00);

    float accRange = -1;
    switch (this->accScale){
        case AccelerometerRange::2_g: accRange = 2; break;
        case AccelerometerRange::4_g: accRange = 4; break;
        case AccelerometerRange::8_g: accRange = 8; break;
        case AccelerometerRange::16_g: accRange = 16; break;
    }

    //real interrupt : acc_range / raw_threshold therefore : float_thr = acc_range / raw_ths -> raw_ths = acc_range / float thr
    if (xDetect){
        uint8_t rawXThreshold = xThreshold == 0 ? 0 : (uint8_t) (accRange / xThreshold);
        this->writeByte(this->gyroIdentifier, ACC_X_INT_THR_REG, rawXThreshold);
        this->writeFlag(this->gyroIdentifier, ACC_INT_CFG_REG, xDetectHigh ? 0b10 : 0b01, 2, 2);
    }
    if (yDetect){
        uint8_t rawYThreshold = yThreshold == 0 ? 0 : (uint8_t) (accRange / yThreshold);
        this->writeByte(this->gyroIdentifier, ACC_Y_INT_THR_REG, rawYThreshold);
        this->writeFlag(this->gyroIdentifier, ACC_INT_CFG_REG, yDetectHigh ? 0b10 : 0b01, 4, 2);
    }
    if (zDetect){
        uint8_t rawZThreshold = zThreshold == 0 ? 0 : (uint8_t) (accRange / zThreshold);
        this->writeByte(this->gyroIdentifier, ACC_Z_INT_THR_REG, rawZThreshold);
        this->writeFlag(this->gyroIdentifier, ACC_INT_CFG_REG, zDetectHigh ? 0b10 : 0b01, 6, 2);
    }

    this->writeByte(this->gyroIdentifier, ACC_INT_DUR_REG, samplesToRecognize | waitBeforeExitingInterrupt ? 0x80 : 0x00);
    this->writeFlag(this->gyroIdentifier, INT1_CTRL_REG, hardwareInterrupt? 1 : 0, 7, 1);

    return 0;
}

int8_t setGyroInterrupt(float xThreshold, bool xDetect, bool xDetectHigh,
                        float yThreshold, bool yDetect, bool yDetectHigh,
                        float zThreshold, bool zDetect, bool zDetectHigh,
                        bool andEventCombination = false, uint8_t samplesToRecognize = 0,
                        bool waitBeforeExitingInterrupt = false, bool decrementCounter = false,
                        bool hardwareInterrupt = false){

    if (samplesToRecognize > 0x7F)
        return -5;

    #reset interrupt
    this->writeByte(this->gyroIdentifier, GYRO_INT_CFG_REG, andEventCombination ? 0x80 : 0x00);

    //x_threshold = int(x_threshold * 1000 / self.gyro_scale)

    if (xDetect){
        int16_t rawXThreshold = (int16_t) ( xThreshold * 1000 / this->gyroScale)
        uint16_t 15_bitXThreshold = this->to15BitWord(rawXThreshold);
        if (15_bitXThreshold == 0x8000)
            return -5;
        15_bitXThreshold |= decrementCounter ? 0x8000 : 0x0000;
        this->writeByte(this->gyroIdentifier, GYRO_X_INT_THR_REG, (uint8_t) (15_bitXThreshold / (1 << 8)));
        this->writeByte(this->gyroIdentifier, GYRO_X_INT_THR_REG + 1, (uint8_t) (15_bitXThreshold % (1 << 7)));
        this->writeFlag(this->gyroIdentifier, GYRO_INT_CFG_REG, xDetectHigh ? 0b10 : 0b01, 2, 2);
    }

    if (yDetect){
        int16_t rawYThreshold = (int16_t) ( yThreshold * 1000 / this->gyroScale)
        uint16_t 15_bitYThreshold = this->to15BitWord(rawYThreshold);
        if (15_bitYThreshold == 0x8000)
            return -5;
        this->writeByte(this->gyroIdentifier, GYRO_Y_INT_THR_REG, (uint8_t) (15_bitYThreshold / (1 << 8)));
        this->writeByte(this->gyroIdentifier, GYRO_Y_INT_THR_REG + 1, (uint8_t) (15_bitYThreshold % (1 << 7)));
        this->writeFlag(this->gyroIdentifier, GYRO_INT_CFG_REG, yDetectHigh ? 0b10 : 0b01, 4, 2);
    }

    if (zDetect){
        int16_t rawZThreshold = (int16_t) ( zThreshold * 1000 / this->gyroScale)
        uint16_t 15_bitZThreshold = this->to15BitWord(rawZThreshold);
        if (15_bitZThreshold == 0x8000)
            return -5;
        this->writeByte(this->gyroIdentifier, GYRO_Z_INT_THR_REG, (uint8_t) (15_bitZThreshold / (1 << 8)));
        this->writeByte(this->gyroIdentifier, GYRO_Z_INT_THR_REG + 1, (uint8_t) (15_bitZThreshold % (1 << 7)));
        this->writeFlag(this->gyroIdentifier, GYRO_INT_CFG_REG, ZDetectHigh ? 0b10 : 0b01, 6, 2);    }

    this->writeByte(this->gyroIdentifier, GYRO_INT_DUR_REG, samplesToRecognize | (waitBeforeExitingInterrupt ? 0x80 : 0x00));
    this->writeFlag(this->gyroIdentifier, INT1_CTRL_REG, hardwareInterrupt ? 1 : 0, 8, 1);

    return 0;
}

uint16_t MP_LSM9DS1::to15BitWord(int16_t value){
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
    }}


int8_t MP_LSM9DS1::setMagInterrupt( float threshold, bool xDetect, bool yDetect, bool zDetect,
                                    bool interruptActiveHigh =false, bool latchInterrupt = false,
                                    bool hardwareInterrupt = false){
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
            threshold = -threshold; // threshold out of ranfe
        uint16_t uint15Threshold = (uint16_t) (threshold * 1000 / this->magScale);
        if (uint15Threshold > 0x7FFF)
            return -5; // threshold out of range
        this->writeByte(this->magIdentifier, MAG_INT_THR_REG, (uint8_t) (uint15Threshold % (1<<7)));
        this->writeByte(this->magIdentifier, MAG_INT_THR_REG + 1, (uint8_t) (uint15Threshold / (1 << 8)));
    }

    return 0;
}

uint8_t MP_LSM9DS1::getAccInterruptStatus(){
    return this->readByte(this->gyroIdentifier, ACC_INT_SRC_REG);
}

uint8_t MP_LSM9DS1::getGyroInterruptStatus(){
    return this->readByte(this->gyroIdentifier, GYRO_INT_SRC_REG);
}

uint8_t MP_LSM9DS1::getMagInterruptStatus(){
    return this->readByte(this->magIdentifier, MAG_INT_SRC_REG);
}

void MP_LSM9DS1::updateGyroMeasurements(){
    //read gyro output values: 16 bit integer in two's complement , byteorder = little (msb is last)
    uint8_t axisRegs[3] = {GYRO_X_REG, GYRO_Y_REG, GYRO_Z_REG};
    for (uint8_t i = 0; i <= 2; i++){
        this->gyroRawMeasurements[i] = this->readByte(this->gyroIdentifier, axisRegs[i]);
        this->gyroRawMeasurements[i] += this->readByte(this->gyroIdentifier, axisRegs[i] + 1) << 8;

        this->gyroMeasurements[i] = ((float) this->gyroRawMeasurements[i]) / 1000.0f * this->gyroScale;
    }
}

void MP_LSM9DS1::updateAccMeasurements(){
    uint8_t axisRegs[3] = {ACC_X_REG, ACC_Y_REG, ACC_Z_REG};
    for (uint8_t i = 0; i <= 2; i++){
        this->accRawMeasurements[i] = this->readByte(this->gyroIdentifier, axisRegs[i]);
        this->accRawMeasurements[i] += this->readByte(this->gyroIdentifier, axisRegs[i] + 1) << 8;

        this->accMeasurements[i] = ((float) this->accRawMeasurements[i]) / 1000.0f * this->accScale;
    }
}

void MP_LSM9DS1::updateMagMeasurements(){
    uint8_t axisRegs[3] = {MAG_X_REG, MAG_Y_REG, MAG_Z_REG};
    for (uint8_t i = 0; i <= 2; i++){
        this->magRawMeasurements[i] = this->readByte(this->magIdentifier, axisRegs[i]);
        this->magRawMeasurements[i] += this->readByte(this->magIdentifier, axisRegs[i] + 1) << 8;

        this->magMeasurements[i] = ((float) this->magRawMeasurements[i]) / 1000.0f * this->magScale;
    }
}

char[] MP_LSM9DS1::getErrorString(int8_t errorCode){
    if (errorCode >= 0) return "No error / Number of bytes written";
    if (errorCode == -1) return "Communication Error with the gyroscope sensor.";
    if (errorCode == -2) return "Communication Error with the magnetometer sensor.";
    if (errorCode == -3) return "I2C communication channel error";
    if (errorCode == -4) return "SPI communication channel error";
    if (errorCode == -5) return "Invalid data format/range (input data out of range)";
    else return "Unknown error";
}

}
