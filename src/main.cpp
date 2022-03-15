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
const byte Left_motor_back=9;       
const byte Left_motor_go=8;         
const byte Right_motor_go=6;       
const byte Right_motor_back=7;   
const byte Right_motor_en=5;      
const byte Left_motor_en=10;      
/*Set Button port*/
const byte key=13;
/*Set BUZZER port*/
const byte beep=12; 
//*******************************************END setup for motor controller**************************************************

//*******************************************for misc setup******************************************************
// keep track of internal LED state
bool blinkState = false;
//the led to light when there is an error
const byte errorLED = 3;
//light when saved file
const byte sucessLED = 4;
//*******************************************END misc setup**************************************************
//@TODO-move this
boolean saveToSD(String);
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
   * @TODO:
   * Put all strings in F()
   * Figure out why program crashes after reading IMU some times
   * remove speaker and key from SPI bus (for UNO)
   * change function definitions from run(int time) to run(unsigned int time) and change to ms
   */



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
  pinMode(Left_motor_go,OUTPUT); 
  pinMode(Left_motor_back,OUTPUT);
  pinMode(Right_motor_go,OUTPUT);
  pinMode(Right_motor_back,OUTPUT);
  pinMode(Right_motor_en,OUTPUT);
  pinMode(Left_motor_en,OUTPUT);
  pinMode(key,INPUT);// Set button as input
  pinMode(beep,OUTPUT);// Set buzzer as output
  digitalWrite(key,HIGH);//Initialize button
  digitalWrite(beep,LOW);// set buzzer mute
Serial.println(F("\n"));
  //*******************************************END setup for motor controller**************************************************


//*******************************************Misc setup******************************************************
  // blink at beginning
  pinMode(errorLED, OUTPUT);
  pinMode(sucessLED, OUTPUT);
 
  digitalWrite(errorLED, HIGH);
  digitalWrite(sucessLED, HIGH);
  delay(1000);
  digitalWrite(errorLED, LOW);
  digitalWrite(sucessLED, LOW);

  saveToSD("***NEW DATA SESSION***");
  saveToSD("time,gps,pitch,roll");
  Serial.println(F("\n"));
  //*******************************************end misc setup******************************************************

} //end setup()

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

boolean saveToSD(String dataString)
{
  
  //@TODO-create a unique file name?
  //@TODO-add header
  //static String fileName = millis();
  static String fileName = "datalog.txt";
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open(fileName, FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
        // print to the serial port too:
        digitalWrite(sucessLED, HIGH);
    Serial.print(F("Writing this data to SD:"));
    Serial.println(dataString);
    dataFile.println(dataString);
    dataFile.close();
    //@TODO-get to move on instead of delay
    delay(100);
    digitalWrite(sucessLED, LOW);

  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
    digitalWrite(errorLED, HIGH);
    return 0;
  }
  return 1;
}

//@TODO-handle prototype functions for drive better
void run(int);
void brake(int);
void drive()
{
  //A preprogrammed driving routine
   //back(10); //back 1s
       //brake(5);//stop 0.5s
       run(10);//ahead  1s
       brake(1);//stop
       //left(10);//turn left  1s
       //right(10);//turn right 1s
       //spin_right(20); //Right rotation  2s
       //spin_left(20);//left rotation2s
       //brake(5);//stop  0.5s 
}
void keyscan()
{
  //@TODO-fix this terribly written code!!
  int val;   
  val=digitalRead(key);// Reads the button ,the level value assigns to val
  while(digitalRead(key))// When the button is not pressed
  {
    val=digitalRead(key);
  }
  while(!digitalRead(key))// When the button is pressed
  {
    delay(10);	//delay 10ms
    val=digitalRead(key);// Reads the button ,the level value assigns to val
    if(val==LOW)  //Double check the button is pressed
    {
       
      digitalWrite(beep,HIGH);//The buzzer sounds
      delay(50);//delay 50ms
      while(!digitalRead(key))	//Determine if the button is released or not
        digitalWrite(beep,LOW);//mute
    }
    else
      digitalWrite(beep,LOW);//mute
  }
}

// before loop(), you either neet to put prototypes for functions called or you can just put them ahead of when they are called
// see here:https://stackoverflow.com/questions/18026611/how-do-i-fix-this-error-was-not-declared-in-this-scope

void loop()
{
  Serial.println(F("Waiting for keypress..."));
  //@TODO-figure out why this delay is needed
  delay(1000);
  //keyscan();//wait for key press
  drive();

  getGPS();

//wait for car to stabilize before taking reading
//@TODO-run experiment to determine minimum time needed to pause or implement thresholding algorithm
delay(500);
//print the angle measurments out
if(getAngle()){
  //@TODO-remove
  Serial.print("R+/L- tilt:");
  //Serial.print("\t");
    //Serial.print(ypr[0]);
    //Serial.print("\t");
    Serial.print(ypr[1]);
    Serial.print('\n');
    Serial.print("forward+/back- tilt:");
    //Serial.print("\t");
    Serial.println(ypr[2]);
  delay(500);
}else{
  Serial.println(F("error reading angle"));
  digitalWrite(errorLED, HIGH);
}

//@TODO-handle SD failure (this returns false when fails)
//@TODO-handle this better (Strings can use ALOT of memory)
    // make a string for assembling the data to log:
    String angle = String(String(ypr[1]) + "," + String(ypr[2]));
    String gpsData = "GPS placeholder";
  String dataString = String(String(millis())+","+gpsData+","+angle);
saveToSD(dataString);

//@TODO-make this work with another LED maybe
// blink LED to indicate activity
  //blinkState = !blinkState;
  //digitalWrite(sucessLED, blinkState);

} // end loop



void run(int time)     // advance
{
 //digitalWrite(Left_motor_en,HIGH);  // Left motor enable
  //analogWrite(Left_motor_en,157);
  digitalWrite(Right_motor_en,HIGH);  // Right motor enable
  digitalWrite(Right_motor_go,HIGH);  // right motor go ahead
  digitalWrite(Right_motor_back,LOW);   
  analogWrite(Right_motor_go,200);//PWM--Pulse Width Modulation(0~255). right motor go speed is 255.
  analogWrite(Right_motor_back,0); 
  digitalWrite(Left_motor_go,HIGH);  // set left motor go ahead
  digitalWrite(Left_motor_back,LOW);
  analogWrite(Left_motor_go,137);//PWM--Pulse Width Modulation(0~255).left motor go speed is 135.
  analogWrite(Left_motor_back,0);
  delay(time * 100);   //Running time can be adjusted 
}

void brake(int time)         //STOP
{
  digitalWrite(Right_motor_go,LOW);//Stop the right motor
  digitalWrite(Right_motor_back,LOW);
  digitalWrite(Left_motor_go,LOW);//Stop the left motor
  digitalWrite(Left_motor_back,LOW);
  delay(time * 100);  //Running time can be adjusted  
}

void left(int time)        //turn left
{
  digitalWrite(Right_motor_go,HIGH);	// right motor go ahead
  digitalWrite(Right_motor_back,LOW);
  analogWrite(Right_motor_go,190); // PWM--Pulse Width Modulation(0~255) control speed，right motor go speed is 255.
  analogWrite(Right_motor_back,0);
  digitalWrite(Left_motor_go,LOW);   // left motor stop
  digitalWrite(Left_motor_back,LOW); 
  analogWrite(Left_motor_go,0); 
  analogWrite(Left_motor_back,0);
  delay(time * 100);	
}
void spin_left(int time)   //Left rotation
{
  digitalWrite(Right_motor_go,HIGH);// right motor go ahead
  digitalWrite(Right_motor_back,LOW);
  analogWrite(Right_motor_go,200);// PWM--Pulse Width Modulation(0~255) control speed ,right motor go speed is 200.
  analogWrite(Right_motor_back,0);
  digitalWrite(Left_motor_go,LOW);   // left motor back off
  digitalWrite(Left_motor_back,HIGH);
  analogWrite(Left_motor_go,0); 
  analogWrite(Left_motor_back,200);// PWM--Pulse Width Modulation(0~255) control speed,left motor back speed is 200.
  delay(time * 100);
}

void right(int time)      //turn right
{
 digitalWrite(Right_motor_go,LOW);   // right motor stop
  digitalWrite(Right_motor_back,LOW);
  analogWrite(Right_motor_go,0); 
  analogWrite(Right_motor_back,0);
  digitalWrite(Left_motor_go,HIGH);// left motor go ahead
  digitalWrite(Left_motor_back,LOW);
  analogWrite(Left_motor_go,230);// PWM--Pulse Width Modulation(0~255) control speed ,left motor go speed is 255.
  analogWrite(Left_motor_back,0);
  delay(time * 100);
}

void spin_right(int time)   //Right rotation
{
  digitalWrite(Right_motor_go,LOW);  // right motor back off
  digitalWrite(Right_motor_back,HIGH);
  analogWrite(Right_motor_go,0); 
  analogWrite(Right_motor_back,200);// PWM--Pulse Width Modulation(0~255) control speed
  digitalWrite(Left_motor_go,HIGH);// left motor go ahead
  digitalWrite(Left_motor_back,LOW);
  analogWrite(Left_motor_go,200);// PWM--Pulse Width Modulation(0~255) control speed 
  analogWrite(Left_motor_back,0);
  delay(time * 100);
}

void back(int time)   //back off 
{
  digitalWrite(Right_motor_go,LOW); //right motor back off
  digitalWrite(Right_motor_back,HIGH);
  analogWrite(Right_motor_go,0);
  analogWrite(Right_motor_back,150);// PWM--Pulse Width Modulation(0~255) control speed
  analogWrite(Right_motor_en,165);
  digitalWrite(Left_motor_go,LOW);  //left motor back off
  digitalWrite(Left_motor_back,HIGH);
  analogWrite(Left_motor_go,0);
  analogWrite(Left_motor_back,140);// PWM--Pulse Width Modulation(0~255) control speed
  delay(time * 100);
}