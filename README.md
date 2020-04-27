# Melopero LSM9DS1 Arduino Library
A library for interfacing the Melopero LSM9DS1 9-DOF breakout board with Arduino.
<br> If you were looking for the Python3 library for the Raspberry Pi click [HERE](https://github.com/melopero/Melopero_LSM9DS1)

# Melopero LSM9DS1 breakout board
![melopero logo](images/melopero-lsm9ds1-main.jpg?raw=true)

# Pinouts

<table style="width:100%">
  <tr>
    <th>Melopero LSM9DS1</th>
    <th>Description</th>
  </tr>
  <tr>
    <td>3V3</td>
    <td>Input power pin. Apply 3.3V to this pin</td>
  </tr>
  <tr>
    <td>SCL</td>
    <td>I2C or SPI Serial Clock pin</td>
  </tr>
  <tr>
    <td>SDA</td>
    <td>I2C SDA pin or SPI MOSI pin</td>
  </tr>
  <tr>
    <td>GND</td>
    <td>Ground pin</td>
  </tr>
  <tr>
    <td>CSAG</td>
    <td>Accelerometer+Gyro SPI Chip Select</td>
  </tr>
  <tr>
    <td>CSM</td>
    <td>Magnetometer SPI Chip Select</td>
  </tr>
  <tr>
    <td>SDOAG</td>
    <td>Accelerometer+Gyro SPI MISO pin</td>
  </tr>
  <tr>
    <td>SDOM</td>
    <td>Magnetometer SPI MISO pin</td>
  </tr>
  <tr>
    <td>INT1</td>
    <td>Accelerometer+Gyro Interrupt pin</td>
  </tr>
   <tr>
    <td>INTM</td>
    <td>Magnetometer Interrupt pin</td>
  </tr>
  <tr>
    <td>INT2</td>
    <td>Another Interrupt pin for the accelerometer+gyro. <br>This pin is not supported in our library</td>
  </tr>
  <tr>
    <td>DEN</td>
    <td>Gyroscope data enable pin. <br>This pin is not supported in our library</td>
  </tr>
  <tr>
    <td>DRDY</td>
    <td>Magnetometer data ready pin. <br>This pin is not supported in our library</td>
  </tr>
</table>

## Getting Started
### Prerequisites
You will need:
- Arduino IDE, you can download it here: [download python3](https://www.arduino.cc/en/main/software)
- the Melopero LSM9DS1 breakout: [buy here](https://www.melopero.com/shop/sensori/imu/melopero-lsm9ds1-breakout/)

### Connect the sensor to Arduino <br>(use only 3.3V power and logic, do not connect this sensor board directly to 5V)
This sensor communicates over I2C or SPI.
<br><b>I2C connections</b>:
<table style="width:100%">
  <tr>
    <th>Melopero LSM9DS1</th>
    <th>Arduino</th>
  </tr>
  <tr>
    <td>3V3</td>
    <td>VCC</td>
  </tr>
  <tr>
    <td>SCL</td>
    <td>SCL</td>
  </tr>
  <tr>
    <td>SDA</td>
    <td>SDA</td>
  </tr>
  <tr>
    <td>GND</td>
    <td>GND</td>
  </tr>
</table>
<br><b>SPI connections</b>:
<table style="width:100%">
  <tr>
    <th>Melopero LSM9DS1</th>
    <th>Arduino</th>
  </tr>
  <tr>
    <td>3V3</td>
    <td>VCC</td>
  </tr>
  <tr>
    <td>SCL</td>
    <td>SCLK</td>
  </tr>
  <tr>
    <td>SDA</td>
    <td>MOSI</td>
  </tr>
  <tr>
    <td>SDOAG, SDOM</td>
    <td>MISO</td>
  </tr>
  <tr>
    <td>CSAG</td>
    <td>any digital pin</td>
  </tr>
  <tr>
    <td>CSM</td>
    <td>any digital pin</td>
  </tr>
</table>
<br><b>Optional interrupt pins</b>:
<table style="width:100%">
  <tr>
    <th>Melopero LSM9DS1</th>
    <th>Arduino</th>
  </tr>
  <tr>
    <td>INT1 <br>(interrupt pin for Accelerometer and Gyroscope)</td>
    <td>Any pin that supports interrupts</td>
  </tr>
  <tr>
    <td>INTM <br>(interrupt pin for Magnetometer)</td>
    <td>Any pin that supports interrupts</td>
  </tr>

</table>



### Install the library
This library can be installed directly from the Library manager of the Arduino IDE.
<br>Open the Arduino IDE, select <b>Sketch-->Include Library-->Manage Libraries</b>.
<br>Type "melopero lsm9ds1", select the library and click on "Install".
<br>The installation will include some examples, available under <b>File-->Examples-->Melopero LSM9DS1</b>.




## Example using I2C bus
```C++
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
```

## Example using SPI bus
```C++
//Author: Leonardo La Rocca
//email: info@melopero.com
//
//In this example it is shown how to configure the device to work with
//the SPI communication protocol.
//First make sure that your connections are setup correctly:
//SPI pinout:
//LSM9DS1 <------> Arduino MKR
//    3v3 <------> VCC
//    SCL <------> SCK (9)
//    SDA <------> MOSI (8)
//    CSG <------> CS1 (6)
//    CSM <------> CS2 (7)
// SDOG, SDOM <------> MISO (10)
//
//Note: SDOG and SDOM must be connected both to the same pin (MISO) therefore if
//      you want to use both devices with the SPI protocol you have to connect
//      SDOG and SDOM . (For example on a breadboard).
//Note: do not connect the device to the 5V pin!

#include <Melopero_LSM9DS1.h>

Melopero_LSM9DS1 device;

int gyroSelect = 6;
int magSelect = 7;

int waitMillis = 1000;

void setup() {
  Serial.begin(9600);

  //Setup the device to use SPI communication.
  //device.getErrorString returns a readable interpretation for the
  //status returned by most of the functions.
  Serial.print("starting, setup SPI: ");
  Serial.println(device.getErrorString(device.useSPI(gyroSelect, magSelect)));

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
```

### Other examples:
You'll find other examples using the interrupts pins [HERE](https://github.com/melopero/Melopero_LSM9DS1_Arduino_Library/tree/master/examples):


### Attention:

This breakout board is compatible only with 3.3V power and logic.  
