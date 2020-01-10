#include <MP_LSM9DS1.h>

MP_LSM9DS1 device;

boolean interruptOccurred = false;
const byte interruptPin = 2; 

void setup() {
  Serial.begin(9600);
  Serial.print("starting, setup i2c: ");
  Serial.println(device.useI2C());

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
                                        
  attachInterrupt(digitalPinToInterrupt(interruptPin), interruptHandler, FALLING);
}

void loop() {
  //Prints out the 7 bits that represent the gyroscope interrupt status
  // interrupt occurred | z high | z low | y high | y low | x high | x low
  Serial.print("int status : ");
  Serial.println(device.getGyroInterruptStatus(), BIN);

  Serial.print("Int pin : ");
  Serial.println(digitalRead(interruptPin));

  delay(200);
  
  if (interruptOccurred){
    device.updateGyroMeasurements();
    printGyroMeasurements();
    Serial.println("Interrupt triggered!");
    Serial.print("Gyroscope interrupt status : ");
    Serial.println(device.getGyroInterruptStatus(), BIN);
    interruptOccurred = false;
  }
    
}

void printGyroMeasurements(){
  Serial.print("[Gyroscope]:");
  Serial.print(" x : ");
  Serial.print(device.gyroMeasurements[0]);
  Serial.print(" y : ");
  Serial.print(device.gyroMeasurements[1]);
  Serial.print(" z : ");
  Serial.print(device.gyroMeasurements[2]);
}

void interruptHandler(){
  interruptOccurred = true;
}
