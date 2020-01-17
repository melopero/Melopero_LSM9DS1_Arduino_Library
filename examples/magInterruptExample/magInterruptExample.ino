//Author: Leonardo La Rocca
//email: info@melopero.com
//
//In this example it is shown how to configure the device to trigger an
//hardware interrupt on the intM pin for the magnetometer. You can use the
//I2C or SPI communication protocol.
//First make sure that your connections are setup correctly:
//I2C pinout:
//LSM9DS1 <------> Arduino MKR
//    3v3 <------> VCC
//    SCL <------> SCL (12)
//    SDA <------> SDA (11)
//    GND <------> GND
//   INTM <------> interruptPin
//Note: You can use the pin you like to listen for the interrupt.
//Note: Do not connect the device to the 5V pin!
//
//SPI pinout:
//LSM9DS1 <------> Arduino MKR
//    3v3 <------> VCC
//    SCL <------> SCK (9)
//    SDA <------> MOSI (8)
//    CSG <------> CS1 (6)
//    CSM <------> CS2 (7)
// SDOG, SDOM <------> MISO (10)
//   INTM <------>  interruptPin
//Note: SDOG and SDOM must be connected both to the same pin (MISO) therefore if
//      you want to use both devices with the SPI protocol you have to connect
//      SDOG and SDOM . (For example on a breadboard).

#include <MP_LSM9DS1.h>

MP_LSM9DS1 device;

boolean interruptOccurred = false;

//This is the pin that will listen for the hardware interrupt.
const byte interruptPin = 1;

void setup() {
  Serial.begin(9600);

  //Setup the device to use i2c communication.
  //device.getErrorString returns a readable interpretation for the
  //status returned by most of the functions.
  //Note: if you want to use SPI you have to call useSPI(gyroCS, magCS)
  //instead of useI2C. gyroCS and magCS are the pin select pins you used
  //(in the pinout above: gyroCS = 6, magCS = 7)
  Serial.print("starting, setup i2c: ");
  Serial.println(device.getErrorString(device.useI2C()));

  //Reset all the settings
  Serial.print("Resetting settings... ");
  Serial.println(device.getErrorString(device.resetSettings()));

  //Next we want to set our output data rate, this will influence the
  //"responsiveness" of the interrupt.
  Serial.print("Setting Magnetometer output data rate: ");
  Serial.println(device.setMagODR(MagnetometerOutputDataRate::Hz_20));

  Serial.print("Setup interrupt : ");
  //This function resets the interrupt settings for the acceleromter,gyroscope and magnetometer
  device.resetInterruptSettings(true, true, true);

  //Now we want to setup our interrupt , we want to fire an hardware interrupt
  //every time the the measure on the x-axis exceeds 0.33 G (is less than -0.33 G or greater than 0.33 G).
  //We simply call the setMagInterrupt function which takes care of this for us.
  //Let's break down the function arguments in order to understand better what is going on.
  //  (float) 0.33f  ---> the threshold value (this gets converted to a raw threshold value (-> documentation))
  //  (bool) true   ---> should the x-axis be detected ?
  //  (bool) false  ---> should the y-axis be detected ?
  //  (bool) false  ---> should the z-axis be detected ?
  //  The next arguments are optional, but since we want to fire an hardware interrupt we need to set them correctly!
  //  (bool) true   ---> should the interrupt signal be HIGH (true) or LOW (false) on active ?
  //  (bool) false  ---> latchInterrupt: should the intM pin remain in the same  state until getMagInterrupt is called?
  //  (bool) true   ---> this flag determines if an hardware interrupt should be fired on the intM pin.
  device.setMagInterrupt(.33f, true, false, false,
                         true, false, true);

  //Next we want to setup our interruptPin to detect the interrupt and to call our
  //interruptHandler function each time an interrupt is triggered.
  pinMode(interruptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin), interruptHandler, RISING);
}

void loop() {
  //Since this is an example let's print out some status registers and measurements
  //so we can see what is going on.

  Serial.print("Int status : ");
  Serial.println(device.getMagInterruptStatus(), BIN);

  Serial.print("Int pin : ");
  Serial.println(digitalRead(interruptPin));

  device.updateMagMeasurements();
  printMagMeasurements();

  if (interruptOccurred){
    Serial.println("****************************");
    Serial.println("*** Interrupt triggered! ***");
    Serial.println("****************************");

    interruptOccurred = false;
  }

  delay(200);
}

void printMagMeasurements(){
  Serial.print("[Magnetometer]:");
  Serial.print(" x : ");
  Serial.print(device.magMeasurements[0]);
  Serial.print(" y : ");
  Serial.print(device.magMeasurements[1]);
  Serial.print(" z : ");
  Serial.print(device.magMeasurements[2]);
  Serial.println();
}

void interruptHandler(){
  interruptOccurred = true;
}
