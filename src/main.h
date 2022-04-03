//https://www.tutorialspoint.com/cprogramming/c_header_files.htm
//a practice in C or C++ is we keep all the constants, macros, system wide global variables, and function prototypes in the header files and include that header file.
#ifndef HEADER_FILE
#define HEADER_FILE

//This function notifies the user of the error number by blink/beep and prints error code to serial, use this to look up what error numbers mean
void errorNotify(byte, bool);

//@TODO-handle prototype functions better
void run(int);
void brake(int);
void waitForGPSFix(void);
void encodeGPSSerial(void);

bool updateGPS(void);

boolean saveToSD(String);

#endif