#include <MP_LSM9DS1.h>

MP_LSM9DS1 device;
int gyroSelect = 10;
int magSelect = 11;
int outputRate = 1;
int waitMillis = 1000 / outputRate;

void setup() {
  Serial.begin(9600);

  //Set the device to use the SPI communication protocol. You have to provide the chip select pins that you use.
  Serial.print("starting, setup SPI: ");
  Serial.println(device.useSPI(gyroSelect, magSelect));
  
  Serial.print("Setting Gyro output data rate: ");
  Serial.println(device.setGyroODR(OutputDataRate::Hz_10));
  Serial.print("Setting Accelerometer output data rate: ");
  Serial.println(device.setAccODR(OutputDataRate::Hz_10));
  Serial.print("Setting Magnetometer output data rate: ");
  Serial.println(device.setMagODR(MagnetometerOutputDataRate::Hz_10));
}

void loop() {
  delay(waitMillis);
  device.updateGyroMeasurements();
  device.updateAccMeasurements();
  device.updateMagMeasurements();

  Serial.println("******** measurements ********");
  printGyroMeasurements();
  Serial.println();
  printAccMeasurements();
  Serial.println();
  printMagMeasurements();
  Serial.println();
}

void printGyroMeasurements(){
  Serial.print("[Gyro]:");
  Serial.print(" x : ");
  Serial.print(device.gyroMeasurements[0]);
  Serial.print(" y : ");
  Serial.print(device.gyroMeasurements[1]);
  Serial.print(" z : ");
  Serial.print(device.gyroMeasurements[2]);
}

void printAccMeasurements(){
  Serial.print("[Accelerometer]:");
  Serial.print(" x : ");
  Serial.print(device.accMeasurements[0]);
  Serial.print(" y : ");
  Serial.print(device.accMeasurements[1]);
  Serial.print(" z : ");
  Serial.print(device.accMeasurements[2]);
}

void printMagMeasurements(){
  Serial.print("[Magnetometer]:");
  Serial.print(" x : ");
  Serial.print(device.magMeasurements[0]);
  Serial.print(" y : ");
  Serial.print(device.magMeasurements[1]);
  Serial.print(" z : ");
  Serial.print(device.magMeasurements[2]); 
}
