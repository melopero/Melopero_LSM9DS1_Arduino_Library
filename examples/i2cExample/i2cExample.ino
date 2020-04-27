//Author: Leonardo La Rocca
//email: info@melopero.com
//
//In this example it is shown how to configure the device to work with
//the I2C communication protocol.
//First make sure that your connections are setup correctly:
//I2C pinout:
//LSM9DS1 <------> Arduino MKR
//    3v3 <------> VCC
//    SCL <------> SCL (12)
//    SDA <------> SDA (11)
//    GND <------> GND
//
//Note: Do not connect the device to the 5V pin!

#include <Melopero_LSM9DS1.h>

Melopero_LSM9DS1 device;

int waitMillis = 1000;

void setup() {
  Serial.begin(9600);

  //Setup the device to use i2c communication.
  //device.getErrorString returns a readable interpretation for the
  //status returned by most of the functions.
  Serial.print("starting, setup i2c: ");
  Serial.println(device.getErrorString(device.useI2C()));

  //Reset all the settings
  Serial.print("Resetting settings... ");
  Serial.println(device.getErrorString(device.resetSettings()));

  //Setup the output data rates
  Serial.print("Setting Gyro output data rate: ");
  Serial.println(device.setGyroODR(OutputDataRate::Hz_10));
  Serial.print("Setting Accelerometer output data rate: ");
  Serial.println(device.setAccODR(OutputDataRate::Hz_10));
  Serial.print("Setting Magnetometer output data rate: ");
  Serial.println(device.setMagODR(MagnetometerOutputDataRate::Hz_10));
}

void loop() {
  delay(waitMillis);

  //first update the measurements
  device.updateGyroMeasurements();
  device.updateAccMeasurements();
  device.updateMagMeasurements();

  //then print them out
  Serial.println("******** measurements ********");
  printGyroMeasurements();
  Serial.println();
  printAccMeasurements();
  Serial.println();
  printMagMeasurements();
  Serial.println();
}

void printGyroMeasurements(){
  //device.gyroMeasurements is an array of three elements (one for each axis)
  //that contains the last measurements for the gyroscope.
  //These measurements are converted to dps, to read the raw values you
  //can use device.gyroRawMeasurements .
  Serial.print("[Gyro dps (degrees per second)]:");
  Serial.print(" x : ");
  Serial.print(device.gyroMeasurements[0]);
  Serial.print(" y : ");
  Serial.print(device.gyroMeasurements[1]);
  Serial.print(" z : ");
  Serial.print(device.gyroMeasurements[2]);
}

void printAccMeasurements(){
  //device.accMeasurements is an array of three elements (one for each axis)
  //that contains the last measurements for the accelerometer.
  //These measurements are converted to g, to read the raw values you
  //can use device.accRawMeasurements .
  Serial.print("[Accelerometer g (gravity)]:");
  Serial.print(" x : ");
  Serial.print(device.accMeasurements[0]);
  Serial.print(" y : ");
  Serial.print(device.accMeasurements[1]);
  Serial.print(" z : ");
  Serial.print(device.accMeasurements[2]);
}

void printMagMeasurements(){
  //device.magMeasurements is an array of three elements (one for each axis)
  //that contains the last measurements for the magnetometer.
  //These measurements are converted to Gauss, to read the raw values you
  //can use device.magRawMeasurements .
  Serial.print("[Magnetometer G (Gauss)]:");
  Serial.print(" x : ");
  Serial.print(device.magMeasurements[0]);
  Serial.print(" y : ");
  Serial.print(device.magMeasurements[1]);
  Serial.print(" z : ");
  Serial.print(device.magMeasurements[2]);
}
