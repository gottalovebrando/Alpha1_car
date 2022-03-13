#include <Arduino.h>

//*******************************************for IMU******************************************************
// most of this IMU code comes from example sketch listed in references in setup
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu(0x68);

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno, this is
   digital I/O pin 2. Mega 2560 this is pin 21 but this is also SCL pin!
 * ========================================================================= */
//@TODO-this works without interrupt pin and does not seem to need to be interupt#0, figure out why
#define INTERRUPT_PIN 19 // use pin 2 on Arduino Uno & most boards

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}

//*******************************************END setup IMU**************************************************

//*******************************************for GPS******************************************************
#include <TinyGPSPlus.h>
//*******************************************END setup GPS**************************************************

//*******************************************for SD card******************************************************
#include <SPI.h>
#include <SD.h>
//Change this if using a different SD card reader
const int chipSelect = 10;
//*******************************************END setup SD card**************************************************

//*******************************************for motor controller******************************************************

//*******************************************END setup for motor controller**************************************************

//*******************************************for misc setup******************************************************
// keep track of internal LED state
bool blinkState = false;
//the led to light when there is an error
const byte errorLED = 31;
//*******************************************END misc setup**************************************************

void setup()
{
  Serial.begin(9600);
  Serial.println();
  Serial.println();
  Serial.println(F("Compiled with VSCode and platformIO core 5.2.5")); //@TODO-update this whenever compiling
  Serial.println(F("and library: electroniccats/MPU6050@^0.5.0"));
  Serial.println(F("This is code for alpha 1 version of prototype car."));
  /*
   * Version history:
   * V1.0-initial
   *
   * References:
   * @TODO-finish reading this one (good) http://www.starlino.com/imu_guide.html
   * https://dronebotworkshop.com/mpu-6050-level/ (mainly used this one)
   * MPU6050_DMP6.ino- I2C device class (I2Cdev) demonstration Arduino sketch... by Jeff Rowberg <jeff@rowberg.net> (in electroniccats/MPU6050@^0.5.0 example folder)
   * SD Datalooger example sketch modified 9 Apr 2012 by Tom Igoe
   * UNIROI Infrared Remote contorl Experiment sketch (see CD that came with car)
   * 
   * @TODO:
   * Put all strings in F()
   * Figure out why program crashes after reading IMU some times
   */

//*******************************************Misc setup******************************************************
  // blink at beginning
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(errorLED, OUTPUT);
  Serial.println(F("\n"));
  //*******************************************end misc setup******************************************************

  //*******************************************for SD card******************************************************
  Serial.print(F("Initializing SD card..."));

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println(F("Card failed, or not present"));
    digitalWrite(errorLED, HIGH);
    // don't do anything more:
    while (1);
  }
  Serial.println(F("card initialized."));
  Serial.println(F("\n"));
  //*******************************************END setup SD card**************************************************

  //*******************************************for IMU******************************************************
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize device
  Serial.println(F("Initializing IMU..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing IMU connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
//@TODO-add error notify here

  // wait for ready
  Serial.println(F("\nCalibration of DMP will occur in 1 second..."));
  delay(1000);

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  //@TODO-check if these get updated when DSP calibrates
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Looking for interrupts"));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
    digitalWrite(errorLED, HIGH);
  }
  Serial.println(F("\n"));
  //*******************************************END setup IMU**************************************************

  //*******************************************for GPS******************************************************
Serial.println(F("\n"));
  //*******************************************END setup GPS**************************************************

  //*******************************************for motor controller******************************************************
Serial.println(F("\n"));
  //*******************************************END setup for motor controller**************************************************

//@TODO-remove this
delay(1000);
} // end setup()

void errorNotify(String message){
  //this does the standard error notification
  //@TODO-implement
  //@TODO-can you now pass strings??
  digitalWrite(errorLED, HIGH);
  Serial.println(message);
}

boolean getAngle()
{
  // returns false if  dmp failed
  // otherwise this updates the array ypr
  //  if programming failed, don't try to do anything
  if (!dmpReady)
    return false;

  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
  { // Get the Latest packet

    //@TODO-can we delte everything that doesn't update ypr?
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    //@OPT-why does this need yucky floating point math?
    ypr[0] = ypr[0] * 180 / M_PI;
    ypr[1] = ypr[1] * 180 / M_PI;
    ypr[2] = ypr[2] * 180 / M_PI;
  }

  return true;
}

int getGPS()
{
  return 1;
}

boolean saveToSD()
{
    // make a string for assembling the data to log:
  String dataString = "";
// open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    Serial.println(dataString);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
    digitalWrite(errorLED, HIGH);
  }
  return 1;
}

void drive()
{
}

// before loop(), you either neet to put prototypes for functions called or you can just put them ahead of when they are called
// see here:https://stackoverflow.com/questions/18026611/how-do-i-fix-this-error-was-not-declared-in-this-scope

void loop()
{

  
  getGPS();
  saveToSD();
  drive();

  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_BUILTIN, blinkState);

if(getAngle()){
  //@TODO-remove
  Serial.print("ypr\t");
    Serial.print(ypr[0]);
    Serial.print("\t");
    Serial.print(ypr[1]);
    Serial.print("\t");
    Serial.println(ypr[2]);
  delay(500);
}else{
  Serial.println(F("error reading angle"));
  digitalWrite(errorLED, HIGH);
}

} // end loop
