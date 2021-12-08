/*
   This is the main.ino file for the Arduino.
   This file will instantiate an object of class Robot, that subsequently contains
   the Eye, Shoulder and Neck subsystem classes. Communication with the arduino is
   accomplished through a serial channel-> Serial1 that receives messages from a Windows API.
*/

#include "Robot.h"
#include "Function.h"

/*
   Instantiating the Robot class object.
*/
Robot eyeTrackerRobot;

/*
   Setup() establishes connection with the XBOX controller, Windows API serial comm line,
   Debugging serial comm line, and IMU if available.
*/
void setup() {
  InitializeComponents();
  // initializing Robot Class object
  eyeTrackerRobot.init();
}

/*
   Loop() function will constantly call RunState function from the Robot class.
   This function performs all the decision making for the State Machine.
*/
void loop() {
  eyeTrackerRobot.RunState();
  //Serial.println(eyeTrackerRobot.GetState());
}

/*
   Function to initialize the serial ports, XBOX, BNO objects and turn on
   the servo relay.
*/
void InitializeComponents()
{
  // Debugging Serial
  Serial.begin(115200); //max value

  // Windows API Serial
  Serial1.begin(115200); // should test 230400

  Serial3.begin(1000000);

  // checking for serial connection
  while (!Serial) {
    ;
  }
  Serial.println("Serial On");

  // checking for IMU-accelerometer connection
//  if (!bno.begin())
//  {
//    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
//  }
//  bno.setExtCrystalUse(true);   // prepping the IMU

  // Checking for XBOX connection
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.print(F("\r\nXBOX USB Library Started"));

  // Setting and writing pin mode for servoRelay.
  pinMode(servoRelay, OUTPUT);
  digitalWrite(servoRelay, LOW); //Servo enable using relay - using that to prevent jitter when arduino starts
}
