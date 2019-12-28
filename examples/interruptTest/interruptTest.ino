#include <MP_LSM9DS1.h>

MP_LSM9DS1 device;

boolean interruptOccurred = false;
const byte interruptPin = 2; 

void setup() {
  Serial.begin(9600);
  Serial.print("starting, setup i2c: ");
  Serial.println(device.useI2C());

  Serial.print("Setting Accelerometer output data rate: ");
  Serial.println(device.setAccODR(OutputDataRate::Hz_119));

  Serial.print("Setup interrupt : ");
  //This function resets the interrupt settings for the acceleromter,gyroscope and magnetometer
  device.resetInterruptSettings(true, true, true);
  
  //Now we want to setup our interrupt , we want to fire an hardware interrupt 
  //every time the the measure on the x-axis exceeds 0.5 g (is less than -0.5g or greater than 0.5g).
  //We simply call the setAccInterrupt function which takes care of this for us.
  //Let's break down the function arguments in order to understand better what is going on.
  //  (float) .5f  ---> the threshold on the x-axis (this gets converted to a raw threshold value (-> documentation)) 
  //  (bool) true  ---> should the x-axis be detected by the interrupt
  //  (bool) true  ---> should the interrupt be fired when the value is greater (x < -0.5f or x > 0.5f) or lesser (-0.5f < x < 0.5f) than our threshold
  //  The next six arguments are the same for the y and z axis and since we don't want to detect them we set them to false.
  //  The next arguments are optional, but since we want to fire an hardware interrupt we need to set them correctly!
  //  (bool) false ---> and / or event combination for the interrupts on the three axes
  //  (uint8_t) 100 --> how many samples must the device recognise before triggering the interrupt ? If we set our acceleromter 
  //                    to work at 119Hz and set this to 119, an interrupt will be triggered only after one second ( the time the device 
  //                    needs to collect 119 samples ) of "triggering" samples.
  //  (bool) false ---> should the device wait for the number of samples specified above before exiting the interrupt
  //  (bool) true  ---> this flag determines if an hardware interrupt should be fired on the int1 pin.
  device.setAccInterrupt(.5f, true, false, 0, false, false, 0, false, false,
                                        false, 100, false, true);
                                        
  attachInterrupt(digitalPinToInterrupt(interruptPin), interruptHandler, RISING);
}

void loop() {
  //Prints out the 7 bits that represent the accelerometer interrupt status
  // interrupt occurred | z high | z low | y high | y low | x high | x low
  Serial.print("int status : ");
  Serial.println(device.getAccInterruptStatus(), BIN);

  Serial.print("Int pin : ");
  Serial.println(digitalRead(interruptPin));
  delay(200);
  
  if (interruptOccurred){
    device.updateAccMeasurements();
    printAccMeasurements();
    Serial.println("Interrupt triggered!");
    Serial.print("Accelerometer interrupt status : ");
    Serial.println(device.getAccInterruptStatus(), BIN);
    interruptOccurred = false;
  }
    
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

void interruptHandler(){
  interruptOccurred = true;
}
