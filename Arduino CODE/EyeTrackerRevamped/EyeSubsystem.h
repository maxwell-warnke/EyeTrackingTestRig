/*
   Eye subsystem class consists of all of the Servo objects that make up the 2 eyes of the robot.
   The class encapsulates all servos used by both eyes,
   and the center locations in microseconds for those servos. The class has public member functions
   to calibrate the eye subsystem and parallax both eyes to a single point on the screen.
*/

#ifndef _EYESUBSYSTEM_H
#define _EYESUBSYSTEM_H

//DYNAMIXEL DEFINITIONS
#define CW_ANGLE_LIMIT_ADDR         6
#define CCW_ANGLE_LIMIT_ADDR        8
#define ANGLE_LIMIT_ADDR_LEN        2
#define OPERATING_MODE_ADDR_LEN     2
#define TORQUE_ENABLE_ADDR          24
#define TORQUE_ENABLE_ADDR_LEN      1
#define LED_ADDR                    25
#define LED_ADDR_LEN                1
#define GOAL_POSITION_ADDR          30
#define GOAL_POSITION_ADDR_LEN      2
#define PRESENT_POSITION_ADDR       36
#define PRESENT_POSITION_ADDR_LEN   2
#define TIMEOUT 10    //default communication timeout 10ms

//#include <Servo.h>
#include <EEPROM.h>
#include <Dynamixel2Arduino.h>
#include "KinematicChain.h"
#include "Function.h"

/*
   #defining constants
*/

/*
   This enumeration is used in servo calibration to determine which servo motor, on which side
   is being calibrated
   X- horizontal
   Z- vertical
*/
enum EyeMotor {
  rightZ = 1,
  leftZ = 2,
  rightX = 3,
  leftX = 4,
};
extern EyeMotor calibrationMotor;

/*
   Eye class declaration consisting of private and public member functions.
   Private - Servo objects, calibration motor and center locations
   Public functions - initialization function, calibration function and parallaxing function for eyes.
*/
class Eyes
{
  private:
    // Servo objects for left and right eyes
    Dynamixel2Arduino *dxl; //NEW

    /*
       Dynamixel constants
    */
    uint8_t turn_on = 1;
    uint8_t turn_off = 0;
    const uint8_t DXL_DIR_PIN =    4;

    const uint8_t DXL_ID_YAW_L = 1;           //ID NUMBER FOR LEFT YAW SERVO
    const uint8_t DXL_ID_PITCH_L = 2;         //ID NUMBER FOR LEFT PITCH SERVO
    const uint8_t DXL_ID_YAW_R = 3;           //ID NUMBER FOR RIGHT YAW SERVO
    const uint8_t DXL_ID_PITCH_R = 4;         //ID NUMBER FOR RIGHT PITCH SERVO

    const float DXL_PROTOCOL_VERSION = 1.0;
    uint16_t goalPosition = 0;
//    uint16_t goalAngle = 0;

    // declaring counts per degree for dynamixel objects
    float countsPerDegree;

    // center locations in microseconds for the servo objects.
    uint16_t lXCenter, lZCenter, rXCenter, rZCenter;
    
    // this function makes the eyes parallax or look at a single point on the screen
    void parallax(BLA::Matrix<4> leftDotPos, BLA::Matrix<4> rightDotPos);

  protected:
    /*
      T - will be the frame of reference located at the center of the top of the neck.
      D - will be the frame of reference at the center of the eye mechanisms,
      with Z up, X to the right and Y along the axis of the straight 90-degree eyes
      L - will be the left eye's frame of reference with the origin at the center of rotation
      and is aligned with the D frame when the 90-degree angle is commanded
      R - will be the right eye's frame of reference with the origin at the center of rotation
      and is aligned with D frame when the 90-degree angle is commanded
    */
    BLA::Matrix<4, 4> gTD, gDL, gDR, gLT, gRT;

  public:
    //constructor
    Eyes();
    //initializer
    void init();
    //calibration function
    void CalibrateEyes(char eyeCalCommand, BLA::Matrix<4, 4> gLS, BLA::Matrix<4, 4> gRS);
    // functions to write and read variables to/from EEPROM
    void WriteEyeCalibrationVariablesToProm();
    void ReadEyeCalibrationVariablesFromProm();
    // public function called by external objects to parallax eyes
    void ParallaxEyesToPos(BLA::Matrix<4> screenDotPos, BLA::Matrix<4, 4> gLS, BLA::Matrix<4, 4> gRS);
    // functions to get the inverse transformation matrices for the eye subsystem
    BLA::Matrix<4, 4> GetInverseLeftEyeTransformation();
    BLA::Matrix<4, 4> GetInverseRightEyeTransformation();
};

#endif
