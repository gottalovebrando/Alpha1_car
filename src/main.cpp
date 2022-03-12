#include <Arduino.h>

//*******************************************for IMU******************************************************
#include "I2Cdev.h"
#include "MPU6050.h"
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
//imu global variable
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro(0x68);
//MPU6050 accelgyro(0x69); // <-- use for AD0 high
//for the threshold detection
unsigned long maxRawAcc;
unsigned long minRawAcc;
//*******************************************END setup IMU**************************************************

void setup() {
  Serial.begin(9600);
  Serial.println();
  Serial.println();
  Serial.println(F("Compiled with VSCode and platformIO core 5.2.5")); //@TODO-update this whenever compiling
  Serial.println(F("and library: electroniccats/MPU6050@^0.5.0"));
  Serial.println(F("Alpha 1 code"));
  Serial.println(F("This is alpha 1 version of prototype."));
    /*
   * Version history:
   * V1.0-initial
   * 
   */

  //blink at beginning
   pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);


//Setup IMU*********************************************************************************
//@TODO-figure out why this isn't delcared in scope
//setupIMU();
//IMU setup
// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  accelgyro.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  Serial.println(F("Set on level ground..."));
}

void loop() {
  
}