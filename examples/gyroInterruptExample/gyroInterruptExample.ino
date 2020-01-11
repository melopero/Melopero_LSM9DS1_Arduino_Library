//Author: Leonardo La Rocca
//email: info@melopero.com
//
//In this example it is shown how to configure the device to trigger an
//hardware interrupt on the int1 pin for the gyroscope. You can use the
//I2C or SPI communication protocol.
//First make sure that your connections are setup correctly:
//I2C pinout:
//LSM9DS1 <------> Arduino
//    3v3 <------> 3v3
//    SCL <------> SCL
//    SDA <------> SDA
//    GND <------> GND
//   INT1 <------> 2
//Note: You can use the pin you like to listen for the interrupt.
//Note: Do not connect the device to the 5V pin!
//
//SPI pinout:
//LSM9DS1 <------> Arduino
//    3v3 <------> 3v3
//    SCL <------> 13 (SCL)
//    SDA <------> 11 (MOSI)
//    CSG <------>  9 (CS1)
//    CSM <------> 10 (CS2)
// SDOG, SDOM <------> 12 (MISO)
//   INT1 <------>  2
//Note: SDOG and SDOM must be connected both to the same pin (MISO) therefore if
//      you want to use both devices with the SPI protocol you have to connect
//      SDOG and SDOM . (For example on a breadboard).

#include <MP_LSM9DS1.h>

MP_LSM9DS1 device;

boolean interruptOccurred = false;

//This is the pin that will listen for the hardware interrupt.
const byte interruptPin = 2;

void setup() {
  Serial.begin(9600);

  //Setup the device to use i2c communication.
  //device.getErrorString returns a readable interpretation for the
  //status returned by most of the functions.
  //Note: if you want to use SPI you have to call useSPI(gyroCS, magCS)
  //instead of useI2C. gyroCS and magCS are the pin select pins you used
  //(in the pinout above: gyroCS = 9, magCS = 10)
  Serial.print("starting, setup i2c: ");
  Serial.println(device.getErrorString(device.useI2C()));

  //Next we want to set our output data rate, this will influence the
  //"responsiveness" of the interrupt.
  Serial.print("Setting gyroscope output data rate: ");
  Serial.println(device.setGyroODR(OutputDataRate::Hz_119));

  Serial.print("Setup interrupt : ");
  //This function resets the interrupt settings for the acceleromter,gyroscope and magnetometer
  device.resetInterruptSettings(true, true, true);

  //Now we want to setup our interrupt , we want to fire an hardware interrupt
  //every time the the measure on the x-axis exceeds 90 dps (is less than -90 dps or greater than 90 dps).
  //We simply call the setGyroInterrupt function which takes care of this for us.
  //Let's break down the function arguments in order to understand better what is going on.
  //  (float) 90  ---> the threshold on the x-axis (this gets converted to a raw threshold value (-> documentation))
  //  (bool) true  ---> should the x-axis be detected by the interrupt
  //  (bool) true  ---> should the interrupt be fired when the value is greater (x < -90 or x > 90) or lesser (90 < x < 90) than our threshold
  //  The next six arguments are the same for the y and z axis and since we don't want to detect them we set them to false.
  //  The next arguments are optional, but since we want to fire an hardware interrupt we need to set them correctly!
  //  (bool) false ---> and / or event combination for the interrupts on the three axes
  //  (uint8_t) 0 --> how many samples must the device recognise before triggering the interrupt ? If we set our acceleromter
  //                    to work at 119Hz and set this to 119, an interrupt will be triggered only after one second ( the time the device
  //                    needs to collect 119 samples ) of "triggering" samples.
  //  (bool) false ---> should the device wait for the number of samples specified above before exiting the interrupt
  //  (bool) false ---> (decrement counter...)
  //  (bool) true  ---> this flag determines if an hardware interrupt should be fired on the int1 pin.
  device.setGyroInterrupt(90, true, true, 0, false, false, 0, false, false,
                                        false, 0, false, false, true);

  //Next we want to setup our interruptPin to detect the interrupt and to call our
  //interruptHandler function each time an interrupt is triggered.
  attachInterrupt(digitalPinToInterrupt(interruptPin), interruptHandler, FALLING);
}

void loop() {
  //Since this is an example let's print out some status registers and measurements
  //so we can see what is going on.

  //Prints out the 7 bits that represent the gyroscope interrupt status
  // interrupt occurred | z high | z low | y high | y low | x high | x low
  Serial.print("Int status : ");
  Serial.println(device.getGyroInterruptStatus(), BIN);

  Serial.print("Int pin : ");
  Serial.println(digitalRead(interruptPin));

  device.updateGyroMeasurements();
  printGyroMeasurements();

  if (interruptOccurred){
    Serial.println("****************************");
    Serial.println("*** Interrupt triggered! ***");
    Serial.println("****************************");

    interruptOccurred = false;
  }

  delay(200);

}

void printGyroMeasurements(){
  Serial.print("[Gyroscope]:");
  Serial.print(" x : ");
  Serial.print(device.gyroMeasurements[0]);
  Serial.print(" y : ");
  Serial.print(device.gyroMeasurements[1]);
  Serial.print(" z : ");
  Serial.print(device.gyroMeasurements[2]);
  Serial.println();
}

void interruptHandler(){
  interruptOccurred = true;
}
