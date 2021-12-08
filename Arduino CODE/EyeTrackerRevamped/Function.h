/*
   This is the header file that is being used to store the global variables,
   configuration variables, kinematic chain map object and EEPROM logger object.
   These objects and variables are used by almost all the other files and hence
   are stored here.
*/
#ifndef _FUNCTION_H
#define _FUNCTION_H

#include <XBOXONE.h>
// Including the libraries for the IMU - acclerometer
//#include <Adafruit_Sensor.h>
//#include <Adafruit_BNO055.h>
//#include <utility/imumaths.h>

// Including library for the matrix elements in the code
#include <BasicLinearAlgebra.h>

#include <AccelStepper.h>
#include <EEPROM.h>

// state machine enumeration.
enum StateMachineState {
  MenuMode = 0,
  RobotRun = 1,
  EyeCalibrate = 2,
  NeckCalibrate = 3,
  ShoulderCalibrate = 4,
};

// EEPROM address enumeration.
enum PromAddress {
  LXCenter = 12,
  LZCenter = 16,
  RXCenter = 20,
  RZCenter = 24,
  XStepperPosition = 0,
  YStepperPosition = 4,
  ZStepperPosition = 8,
  LastPhiR = 28,
  LastPhiS = 32,
  LastPhiD = 36,
  YawServoCalibrationCenter = 40,
};

// declaring global variables as extern. They are declared
// in the source file for this header.

// BNO IMU object
//extern Adafruit_BNO055 bno;

// Hardware Serial object for comms with API
// Hardware Serial is just a pointer to Serial1.
extern HardwareSerial *SerialTerminal;

// XBOX and USB objects
extern USB Usb;
extern XBOXONE Xbox;

// String array containing the state names.
extern String stateNames[12];

// Declare Servo Pins - check wiring diagram if you have any questions
extern int xLPin;
extern int zLPin;
extern int xRPin;
extern int zRPin;

// Declaring Servo Relat Pin
extern int servoRelay;

// Declaring Shoulder Limit Switches and Stepper Pins - check wiring diagram if you have any questions
// Limit switch pins
extern int xEnd;
extern int yEnd;
extern int zEnd;
// Stepper Pins for X,Y,Z steppers; Dir-Direction;
extern int xDir;
extern int xPulse;
extern int yDir;
extern int yPulse;
extern int zDir;
extern int zPulse;

// Declaring the Neck Stepper Direction and Pulse Pins.

// NECK Stepper Direction and Pulse Pins
extern int frontDir; 
extern int frontPulse; 
extern int backRightDir; 
extern int backRightPulse; 
extern int backLeftDir;  
extern int backLeftPulse; //50 on PCB;
// Declaring neck servo pins
extern int neckServoPin;

#endif
