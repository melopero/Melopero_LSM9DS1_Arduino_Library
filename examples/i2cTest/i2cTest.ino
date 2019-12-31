#include <MP_LSM9DS1.h>

MP_LSM9DS1 device;
int outputRate = 1;
int waitMillis = 1000 / outputRate;

void setup() {
  Serial.begin(9600);
  Serial.print("starting, setup i2c: ");
  Serial.println(device.useI2C());
  Serial.print("Resetting settings... ");
  Serial.println(device.resetSettings());
 
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
