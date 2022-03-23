#include <Arduino.h>
// include main.h to get around having to delclare prototypes @TODO-is there a better way?
#include <main.h>

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
/*
//NOTE-software serial did not work on mega but initialize code included below for reference
//Not all pins on the Mega and Mega 2560 support change interrupts, so only the following can be used for RX:
//10, 11, 12, 13, 14, 15, 50, 51, 52, 53, A8 (62), A9 (63), A10 (64), A11 (65), A12 (66), A13 (67), A14 (68), A15 (69).
//source-https://www.arduino.cc/en/Reference/SoftwareSerial
#include <SoftwareSerial.h>
#define RxPin 15
#define TxPin 14
SoftwareSerial gpsSerial(RxPin, TxPin);
*/
// Default baud of NEO-6M GPS is 9600
int GPSBaud = 9600;
// create the TinyGPSPlus object
TinyGPSPlus gps;
//GNSS Quality indicator- per NEMA 0183 Version 3.01: 0 = Fix not available or invalid, 1 = GPS SPS Mode, fix valid, 2 = Differential GPS, SPS Mode, fix valid, 3 = GPS PPS Mode, fix valid, 4 = Real Time Kinematic. System used in RTK mode with fixed integers, 5 = Float RTK. Satellite system used in RTK mode, floating integers, 6 = Estimated (dead reckoning) Mode, 7 = Manual Input Mode, 8 = Simulator Mode
//@TODO-test if this works, was having trouble before
TinyGPSCustom GPSquality(gps, "GPGGA", 6);
// save raw serial data from GPS to SD card @TODO-implement
bool saveGPSToSD = false;
// send raw serial data from GPS to serial port 2 on mega (9600 baud) @TODO-implement for simulated data
bool saveGPSToSerial2 = true;
//use simulated GPS data instead, for debug purposes
bool simulateGPS = false;
// A sample NMEA stream for simulating
const char *gpsStream =
  "$GPRMC,045103.000,A,3014.1984,N,09749.2872,W,0.67,161.46,030913,,,A*7C\r\n"
  "$GPGGA,045104.000,3014.1985,N,09749.2873,W,1,09,1.2,211.6,M,-22.5,M,,0000*62\r\n"
  "$GPRMC,045200.000,A,3014.3820,N,09748.9514,W,36.88,65.02,030913,,,A*77\r\n"
  "$GPGGA,045201.000,3014.3864,N,09748.9411,W,1,10,1.2,200.8,M,-22.5,M,,0000*6C\r\n"
  "$GPRMC,045251.000,A,3014.4275,N,09749.0626,W,0.51,217.94,030913,,,A*7D\r\n"
  "$GPGGA,045252.000,3014.4273,N,09749.0628,W,1,09,1.3,206.9,M,-22.5,M,,0000*6F\r\n";

//note-double on mega is only 32 bits, not usual 64 bits
//to store GPS Latitude and Longitude, in that order
double latLong[2];
//to store the GPS date, time, and number of satelites in use, in that order
uint32_t dateTimeSats[3];
//to store the Horizontal Dim. of Precision (100ths-i32)
int32_t hdop;
//*******************************************END setup GPS**************************************************

//*******************************************for SD card******************************************************
#include <SPI.h>
#include <SD.h>
// Change this if using a different SD card reader
const int chipSelect = 10;
//*******************************************END setup SD card**************************************************

//*******************************************for motor controller******************************************************
const byte Left_motor_back = 9;
const byte Left_motor_go = 8;
const byte Right_motor_go = 6;
const byte Right_motor_back = 7;
const byte Right_motor_en = 5;
const byte Left_motor_en = 10;
//*******************************************END setup for motor controller**************************************************

//*******************************************for misc setup******************************************************
// keep track of internal LED state
bool blinkState = false; //@TODO-delete this?
// the led to light when there is an error
const byte errorLED = 3;
// light when saved file
const byte sucessLED = 4;
// Set Button pin
const byte keyPin = 13;
// Set BUZZER pin
const byte buzzerPin = 12;

// @TODO-implement this
// Set this to false if not using the USB serial port
// const bool serialEnabled = true;
// set this to output extra messages to serial port, will not work if serialEnabled is false
const bool debugEnabled = true;

//*******************************************END misc setup**************************************************

void setup()
{
  Serial.begin(9600);
  Serial.println();
  Serial.println();
  Serial.println(F("Compiled with VSCode and platformIO core 5.2.5")); //@TODO-update this whenever compiling
  Serial.println(F("and librarys: electroniccats/MPU6050@^0.5.0, mikalhart/TinyGPSPlus@^1.0.3, arduino-libraries/SD@^1.2.4"));
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
   * UNIROI  (see CD that came with car)
   *
   * @TODO-prioritized list:
   * implement error function with beeping and error numbers
   * implement settling function for angle and record max & min
   * save raw GPS data to SD
   * save GPS satelites used
   * implement recording of accelerometer data while driving
   * Make sure power usage is below what Mega's (1870_LD1117S50CTR) 5V regulator can handle (12W)
   * Add voltage measurment (dropout voltage for 1870_LD1117S50CTR) is 1.1@100mA, 1.2V@800mA.
   * Put all strings in F()
   * change driving function definitions from run(int time) to run(unsigned int time) and change to ms
   * Add remote control
   * Figure out why IMU and RTC don't work when on the same I2C bus
   * remove speaker and keyPin button from SPI bus (for UNO)
   * figure out what enable pin is needed for the motor controllers (seems like waste of pin)
   * Figure out why program crashes after reading IMU sometimes
   * Implement thresholding algorthim to determine when to stop and take new data
   * Consider switching to NeoGPS instead of TinyGPSPlus. Apparently it uses 41 bytes of SRAM vs 240. It can use less in more minimal configs
   */

  //*******************************************Misc setup******************************************************
  // blink at beginning
  pinMode(errorLED, OUTPUT);
  pinMode(sucessLED, OUTPUT);
  //@TODO-check with INPUT_PULLUP
  pinMode(keyPin, INPUT);       // Set button as input
  pinMode(buzzerPin, OUTPUT);   // Set buzzer as output
  digitalWrite(keyPin, HIGH);   // Initialize pullups on
  digitalWrite(buzzerPin, LOW); // set buzzer mute

  Serial.println(F("\n"));
  //*******************************************end misc setup******************************************************

  //*******************************************for SD card******************************************************
  Serial.print(F("Initializing SD card..."));

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect))
  {
    Serial.println(F("Card failed, or not present"));
    digitalWrite(errorLED, HIGH);
    // don't do anything more:
    while (1)
      ;
  }
  Serial.println(F("card initialized."));
  //this saves the header the first time its run after this text
  saveToSD("***NEW DATA SESSION***,data_format_ver:1,date:March 2022");
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
  //@TODO-check if these get updated when DMP calibrates
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
    errorNotify(1, true);
  }
  Serial.println(F("\n"));
  //*******************************************END setup IMU**************************************************

  //*******************************************for motor controller******************************************************
  pinMode(Left_motor_go, OUTPUT);
  pinMode(Left_motor_back, OUTPUT);
  pinMode(Right_motor_go, OUTPUT);
  pinMode(Right_motor_back, OUTPUT);
  pinMode(Right_motor_en, OUTPUT);
  pinMode(Left_motor_en, OUTPUT);
  Serial.println(F("\n"));
  //*******************************************END setup for motor controller**************************************************

  //*******************************************for GPS******************************************************
  // Start the software serial port at the GPS's default baud, 1 start bit, 8 data bits, no parity, on stop bit
  //@TODO-consider increasing buffer length from 64 bytes to 82 bytes.
  //82 bytes is max NEMA sentance length (see ref, pg 13)
  //64 bytes is the default serial buffer size (https://internetofhomethings.com/homethings/?p=927)
  //NEMA specifies 10 bits per byte so, @ 9800 its 0.980 bytes/ms and @4800 its 0.480 bytes/ms
  Serial3.begin(GPSBaud,SERIAL_8N1); //SERIAL_8N1 is the default but just in case
  if (saveGPSToSerial2)
      {
        Serial2.begin(9600);
      }
  delay(100);
  Serial.println(F("Waiting for GPS fix..."));
  waitForGPSFix();
  Serial.print(F("We have a fix!"));

  Serial.println(F("\n"));
  //*******************************************END setup GPS**************************************************

  // when ready, do a lamp/buzzer test
  digitalWrite(buzzerPin, HIGH);
  digitalWrite(errorLED, HIGH);
  digitalWrite(sucessLED, HIGH);
  delay(500);
  digitalWrite(buzzerPin, LOW);
  digitalWrite(errorLED, LOW);
  digitalWrite(sucessLED, LOW);
} // end setup()

void errorNotify(byte errorNum, bool haltExe)
{
  Serial.print(F("Encountered error number:"));
  Serial.println(errorNum);
  switch (errorNum)
  {
  case 1:
    Serial.println(F("IMU failed to start, check connections"));
    break;
  case 2:
    Serial.println(F("There was a problem with GPS."));
    break;
  case 3:
    Serial.println(F("Error reading angle."));
    break;
  case 4:
    Serial.println(F("Error writing to SD card."));
  default:
    Serial.println(F("Error number unknown! Halting execution."));
    haltExe = true;
    break;
  }

  if (haltExe)
  {
    Serial.println(F("Fatal error, Halting execution."));
  }else{
    Serial.println(F("Non fatal error, will continue to execute."));
  }

  byte iter = 0;
  while (iter < 2)//just loop twice
  { // loop through this forever if haltExe is true
    for (byte i = 0; i < errorNum; i++)
    { // beep and blink for the error number
      digitalWrite(errorLED, HIGH);
      digitalWrite(buzzerPin, HIGH);
      delay(100);
      digitalWrite(errorLED, LOW);
      digitalWrite(buzzerPin, LOW);
      delay(200);
    }
    delay(500);
    if (!haltExe)
    {
      iter++;
    }
  }
}

void testGPSSim()
{
  //for debugging purposes, @TODO-delete
  int i=0;
  while (i<5)
  {
    i++;
    while (*gpsStream)
    {
      int pos=0;
      gps.encode(*gpsStream+pos);
      Serial.print("gps.encode reading:");
      Serial.println(*gpsStream+pos);
      pos++;
      //gps.encode(*gpsStream++);
      //Serial.print("gps.encode reading:");
      //Serial.println(*gpsStream);
    }
    delay(100);
    

    if (gps.satellites.isValid())
    {
      Serial.print(F("Number of satellites found="));
      Serial.print(gps.satellites.value());
      Serial.print(F(" data age="));
      Serial.print(gps.satellites.age());
      Serial.println(F(" ms."));
    }
    else
    {
      Serial.println(F("gps.satellites.isUpdated() returned false. There may be a problem..."));
    }
    delay(100);
  }
}

//this function reads the GPS data from the serial buffer
void encodeGPSSerial(){

      while (Serial3.available() > 0)
      {
        if (saveGPSToSerial2)
          Serial2.write(Serial3.peek());
        if (saveGPSToSD)
        {
          //@TODO-implement!
          // Serial3.peek()
        }
        gps.encode(Serial3.read());
        //delay(2); //since data could be coming in as slow as 0.48 bytes/ms, do we want to add this? could cause a buffer overflow though
      }
}

//this function prints out the number of satelites found if getting updated data from GPS (not necessarily changed) since last query
void waitForGPSFix()
{//@TODO-figure out if this is the best way to determine if the fix is valid. consider GGA, field 6 as non-zero TinyGPSCustom GPSquality(gps, "GPGGA", 6);
//see Henrique's comment from 2018 (http://arduiniana.org/libraries/tinygpsplus/): "When I lost conection to/restart GPS, and lose the FIX, the funcions isValid and isUpdate still returning true even if the GPS are putting out GPRMC with no data other then date and time ? How culd I know if I really have a valid data ?"
  while (!gps.location.isValid())
  {
    if (!simulateGPS)
    {
      encodeGPSSerial();
    }
    else
    {
      Serial.println(F("USING SIMULATED GPS DATA. Change simulateGPS to false and recompile to change."));
      while (*gpsStream)
      {
        gps.encode(*gpsStream++);
      }
      //@TODO-implement save to SD and write to serial
      //@TODO-how to reset this?
      //*gpsStream = 0;
    }

    if(GPSquality.isUpdated()){
      Serial.print(F("GPS Quality indicator:"));
      Serial.println(GPSquality.value());
    }

    if (gps.satellites.isUpdated())
    {
      Serial.print(F("Number of satellites found="));
      Serial.print(gps.satellites.value());
      Serial.print(F(" age of data="));
      Serial.print(gps.satellites.age());
      Serial.println(F(" ms."));
    }
  } // end of waiting for GPS fix
}

//this function updates the GPS variables (latLong[2], dateTimeSats[3], hdop), returns true if sucessful, will halt program otherwise
bool updateGPS()
{

  bool sucess = false;
  if (debugEnabled)
  {
    Serial.println(F("Updating GPS location info"));
  }
  if (!simulateGPS)
  {
    encodeGPSSerial();
    waitForGPSFix();
  }
  else
  {
    // simulated GPS data @TODO-handle this
  }

  if (debugEnabled)
  {
    if (GPSquality.isValid()){
      Serial.print(F("GPS Quality indicator:"));
      Serial.println(GPSquality.value());
    }
  }

  if (gps.location.isUpdated())
  {
    latLong[0]=gps.location.lat();// Latitude in degrees (double)
    latLong[1]=gps.location.lng();// Longitude in degrees (double)
  }
  else { sucess = false; }

  if (gps.time.isUpdated() && gps.date.isUpdated())
  {
    dateTimeSats[0]=gps.date.value();// Raw date in DDMMYY format (u32)
    dateTimeSats[1]=gps.time.value();// Raw time in HHMMSSCC format (u32)
  }
  else{ sucess = false; }

  if (gps.satellites.isUpdated())
  {
    dateTimeSats[2]=gps.satellites.value();// Number of satellites in use (u32)
  }
  else{ sucess = false; }
  if (gps.hdop.isUpdated())
  {
    hdop=gps.hdop.value(); // Horizontal Dim. of Precision (100ths-i32)
  }
  else{sucess = false;}

  return sucess;
}

boolean updateAngle()
{
  // returns false if  dmp failed
  // otherwise this updates the array ypr
  //  if programming failed, return error
  if (!dmpReady)
    return false;

  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
  { // Get the Latest packet

    //@TODO-can we delte everything that doesn't update ypr?
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    //@TODO-why does this need yucky floating point math?
    ypr[0] = ypr[0] * 180 / M_PI;
    ypr[1] = ypr[1] * 180 / M_PI;
    ypr[2] = ypr[2] * 180 / M_PI;
  }

  return true;
}

//this writes a data string to the SD card, returns true if sucessful, false otherwise. If first time its called, it will write a header for file afte writing whatever data is sent.
boolean saveToSD(String dataString)
{
  static bool writeHeader=true;
  //@TODO-create a unique file name?
  // static String fileName = millis();
  static String fileName = "datalog.txt";
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open(fileName, FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile)
  {
    //provide feedback via buzzer and LED
    digitalWrite(sucessLED, HIGH);
    digitalWrite(buzzerPin, HIGH);
  
    Serial.print(F("Writing this data to SD:"));
    Serial.println(dataString);
    dataFile.println(dataString);
    if(writeHeader){
      String header=F("ms since boot,Latitude (degrees),Longitude (degrees),GPS UTC date(DDMMYY),GPS UTC time(HHMMSSCC),Number of GPS satellites used for fix,GPS Horizontal dilution of precision,pitch (degrees),roll (degrees)");
      Serial.print(F("Writing this data to SD:"));
      Serial.println(header);
      dataFile.println(header);
      writeHeader=false;
    }
    dataFile.close();
    //@TODO-get code to move on instead of using delay
    //delay(100);
    digitalWrite(sucessLED, LOW);
    digitalWrite(buzzerPin, LOW);
  }
  // if the file isn't open, send back an error
  else
  {
    return 0;
  }
  return 1;
}

void drive()
{
  // A preprogrammed driving routine
  // back(10); //back 1s
  // brake(5);//stop 0.5s
  run(10);  // ahead  1s
  brake(1); // stop
  // left(10);//turn left  1s
  // right(10);//turn right 1s
  // spin_right(20); //Right rotation  2s
  // spin_left(20);//left rotation2s
  // brake(5);//stop  0.5s
}
void keyscan()
{
  //@TODO-fix this terribly written code!!
  int val;
  val = digitalRead(keyPin);  // Reads the button ,the level value assigns to val
  while (digitalRead(keyPin)) // When the button is not pressed
  {
    val = digitalRead(keyPin);
  }
  while (!digitalRead(keyPin)) // When the button is pressed
  {
    delay(10);                 // delay 10ms
    val = digitalRead(keyPin); // Reads the button ,the level value assigns to val
    if (val == LOW)            // Double check the button is pressed
    {

      digitalWrite(buzzerPin, HIGH);  // The buzzer sounds
      delay(50);                      // delay 50ms
      while (!digitalRead(keyPin))    // Determine if the button is released or not
        digitalWrite(buzzerPin, LOW); // mute
    }
    else
      digitalWrite(buzzerPin, LOW); // mute
  }
}

// before loop(), you either neet to put prototypes for functions called or you can just put them ahead of when they are called
// see here:https://stackoverflow.com/questions/18026611/how-do-i-fix-this-error-was-not-declared-in-this-scope
void loop()
{
  Serial.println(F("Waiting for keypress..."));
  //@TODO-figure out why this delay is needed
  delay(1000);
  keyscan();//wait for keyPin press
  drive();


  //collect data
  if(updateGPS()){
    //nothing needed if sucessful
  }else{
    errorNotify(2,false);
  }

  // wait for car to stabilize before taking reading
  //@TODO-run experiment to determine minimum time needed to pause or implement thresholding algorithm
  delay(1000);
  // print the angle measurments out
  if (updateAngle())
  { //nothing needed if sucessful
  }
  else
  {
    errorNotify(3,true);
  }

  //@TODO-remove this dynamic memory allocation. This is not good for limited RAM space on microcontrollers!
  //@TODO-check the number of decimal places
  //@TODO-fix the time and date, should have leading zeros (I think)
  String gpsData = String(String(latLong[0],6)+","+String(latLong[1],6)+","+String(dateTimeSats[0])+","+String(dateTimeSats[1])+","+String(dateTimeSats[2])+","+String(hdop,6));
  String angle = String(String(ypr[1],6)+","+String(ypr[2],6));
  String dataString = String(String(millis())+","+gpsData+","+angle);
  if(saveToSD(dataString)){
    //do nothing if sucessful write
  }else{
    errorNotify(4,true);
  };


} // end loop

void run(int time) // advance
{
  // digitalWrite(Left_motor_en,HIGH);  // Left motor enable
  // analogWrite(Left_motor_en,157);
  digitalWrite(Right_motor_en, HIGH); // Right motor enable
  digitalWrite(Right_motor_go, HIGH); // right motor go ahead
  digitalWrite(Right_motor_back, LOW);
  analogWrite(Right_motor_go, 200); // PWM--Pulse Width Modulation(0~255). right motor go speed is 255.
  analogWrite(Right_motor_back, 0);
  digitalWrite(Left_motor_go, HIGH); // set left motor go ahead
  digitalWrite(Left_motor_back, LOW);
  analogWrite(Left_motor_go, 137); // PWM--Pulse Width Modulation(0~255).left motor go speed is 135.
  analogWrite(Left_motor_back, 0);
  delay(time * 100); // Running time can be adjusted
}

void brake(int time) // STOP
{
  digitalWrite(Right_motor_go, LOW); // Stop the right motor
  digitalWrite(Right_motor_back, LOW);
  digitalWrite(Left_motor_go, LOW); // Stop the left motor
  digitalWrite(Left_motor_back, LOW);
  delay(time * 100); // Running time can be adjusted
}

void left(int time) // turn left
{
  digitalWrite(Right_motor_go, HIGH); // right motor go ahead
  digitalWrite(Right_motor_back, LOW);
  analogWrite(Right_motor_go, 190); // PWM--Pulse Width Modulation(0~255) control speed，right motor go speed is 255.
  analogWrite(Right_motor_back, 0);
  digitalWrite(Left_motor_go, LOW); // left motor stop
  digitalWrite(Left_motor_back, LOW);
  analogWrite(Left_motor_go, 0);
  analogWrite(Left_motor_back, 0);
  delay(time * 100);
}
void spin_left(int time) // Left rotation
{
  digitalWrite(Right_motor_go, HIGH); // right motor go ahead
  digitalWrite(Right_motor_back, LOW);
  analogWrite(Right_motor_go, 200); // PWM--Pulse Width Modulation(0~255) control speed ,right motor go speed is 200.
  analogWrite(Right_motor_back, 0);
  digitalWrite(Left_motor_go, LOW); // left motor back off
  digitalWrite(Left_motor_back, HIGH);
  analogWrite(Left_motor_go, 0);
  analogWrite(Left_motor_back, 200); // PWM--Pulse Width Modulation(0~255) control speed,left motor back speed is 200.
  delay(time * 100);
}

void right(int time) // turn right
{
  digitalWrite(Right_motor_go, LOW); // right motor stop
  digitalWrite(Right_motor_back, LOW);
  analogWrite(Right_motor_go, 0);
  analogWrite(Right_motor_back, 0);
  digitalWrite(Left_motor_go, HIGH); // left motor go ahead
  digitalWrite(Left_motor_back, LOW);
  analogWrite(Left_motor_go, 230); // PWM--Pulse Width Modulation(0~255) control speed ,left motor go speed is 255.
  analogWrite(Left_motor_back, 0);
  delay(time * 100);
}

void spin_right(int time) // Right rotation
{
  digitalWrite(Right_motor_go, LOW); // right motor back off
  digitalWrite(Right_motor_back, HIGH);
  analogWrite(Right_motor_go, 0);
  analogWrite(Right_motor_back, 200); // PWM--Pulse Width Modulation(0~255) control speed
  digitalWrite(Left_motor_go, HIGH);  // left motor go ahead
  digitalWrite(Left_motor_back, LOW);
  analogWrite(Left_motor_go, 200); // PWM--Pulse Width Modulation(0~255) control speed
  analogWrite(Left_motor_back, 0);
  delay(time * 100);
}

void back(int time) // back off
{
  digitalWrite(Right_motor_go, LOW); // right motor back off
  digitalWrite(Right_motor_back, HIGH);
  analogWrite(Right_motor_go, 0);
  analogWrite(Right_motor_back, 150); // PWM--Pulse Width Modulation(0~255) control speed
  analogWrite(Right_motor_en, 165);
  digitalWrite(Left_motor_go, LOW); // left motor back off
  digitalWrite(Left_motor_back, HIGH);
  analogWrite(Left_motor_go, 0);
  analogWrite(Left_motor_back, 140); // PWM--Pulse Width Modulation(0~255) control speed
  delay(time * 100);
}