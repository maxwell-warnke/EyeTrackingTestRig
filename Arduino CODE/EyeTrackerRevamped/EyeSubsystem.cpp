#include "EyeSubsystem.h"

/*
   variable that indicates which motor is being calibrated.
*/
EyeMotor calibrationMotor = rightZ;

/*
   Eye class constructor.
*/
Eyes::Eyes()
{
}

/*
   Initialization function for eye subsystem. Attaches the servo objects
   to their respective pins and sets default value for the center locations.
*/
void Eyes::init()
{
  this->dxl = new Dynamixel2Arduino(Serial3, DXL_DIR_PIN);

  this->dxl->begin(1000000);
  this->dxl->setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  /*
     Intializing dynamixel objects.
  */
  // Turn off torque when configuring items in EEPROM area LEFT YAW
  if (dxl->write(DXL_ID_YAW_L, TORQUE_ENABLE_ADDR, (uint8_t*)&turn_off , TORQUE_ENABLE_ADDR_LEN, TIMEOUT))
    Serial.println("DYNAMIXEL Torque off for left yaw");
  else
    Serial.println("Error: Torque off failed for left yaw");

  // Turn off torque when configuring items in EEPROM area LEFT PITCH
  if (dxl->write(DXL_ID_PITCH_L, TORQUE_ENABLE_ADDR, (uint8_t*)&turn_off , TORQUE_ENABLE_ADDR_LEN, TIMEOUT))
    Serial.println("DYNAMIXEL Torque off for left pitch");
  else
    Serial.println("Error: Torque off failed for left pitch");

  // Turn off torque when configuring items in EEPROM area RIGHT YAW
  if (dxl->write(DXL_ID_YAW_R, TORQUE_ENABLE_ADDR, (uint8_t*)&turn_off , TORQUE_ENABLE_ADDR_LEN, TIMEOUT))
    Serial.println("DYNAMIXEL Torque off for left yaw");
  else
    Serial.println("Error: Torque off failed for right yaw");

  // Turn off torque when configuring items in EEPROM area RIGHT PITCH
  if (dxl->write(DXL_ID_PITCH_R, TORQUE_ENABLE_ADDR, (uint8_t*)&turn_off , TORQUE_ENABLE_ADDR_LEN, TIMEOUT))
    Serial.println("DYNAMIXEL Torque off for left pitch");
  else
    Serial.println("Error: Torque off failed for right pitch");

  // Set to Joint Mode LEFT YAW
  //  if(dxl->write(DXL_ID_YAW_L, CW_ANGLE_LIMIT_ADDR, (uint8_t*)&goalPosition1, ANGLE_LIMIT_ADDR_LEN, TIMEOUT)
  //        && dxl.write(DXL_ID_YAW_L, CCW_ANGLE_LIMIT_ADDR, (uint8_t*)&goalPosition2, ANGLE_LIMIT_ADDR_LEN, TIMEOUT))
  //    Serial.println("Set operating mode");
  //  else
  //    Serial.println("Error: Set operating mode failed for left yaw");

  // Set to Joint Mode Left PITCH
  //  if(dxl->write(DXL_ID_PITCH_L, CW_ANGLE_LIMIT_ADDR, (uint8_t*)&goalPosition1, ANGLE_LIMIT_ADDR_LEN, TIMEOUT)
  //        && dxl.write(DXL_ID_YAW_L, CCW_ANGLE_LIMIT_ADDR, (uint8_t*)&goalPosition2, ANGLE_LIMIT_ADDR_LEN, TIMEOUT))
  //    Serial.println("Set operating mode");
  //  else
  //    Serial.println("Error: Set operating mode failed for left yaw");

  //SECTION TO TURN ON TORQUE (ENABLE MOVEMENT)
  // Turn on torque LEFT YAW
  if (dxl->write(DXL_ID_YAW_L, TORQUE_ENABLE_ADDR, (uint8_t*)&turn_on, TORQUE_ENABLE_ADDR_LEN, TIMEOUT))
    Serial.println("Torque on for left yaw");
  else
    Serial.println("Error: Torque on failed for left yaw");

  // Turn on torque LEFT PITCH
  if (dxl->write(DXL_ID_PITCH_L, TORQUE_ENABLE_ADDR, (uint8_t*)&turn_on, TORQUE_ENABLE_ADDR_LEN, TIMEOUT))
    Serial.println("Torque on for left PITCH");
  else
    Serial.println("Error: Torque on failed for left PITCH");

  // Turn on torque RIGHT YAW
  if (dxl->write(DXL_ID_YAW_R, TORQUE_ENABLE_ADDR, (uint8_t*)&turn_on, TORQUE_ENABLE_ADDR_LEN, TIMEOUT))
    Serial.println("Torque on for right yaw");
  else
    Serial.println("Error: Torque on failed for right yaw");

  // Turn on torque RIGHT PITCH
  if (dxl->write(DXL_ID_PITCH_R, TORQUE_ENABLE_ADDR, (uint8_t*)&turn_on, TORQUE_ENABLE_ADDR_LEN, TIMEOUT))
    Serial.println("Torque on for right PITCH");
  else
    Serial.println("Error: Torque on failed for right pitch");

  // 1500 microseconds is the default center for servo objects from class Servo.h
  this->lXCenter = 2048;
  this->lZCenter = 2048;
  this->rXCenter = 2048;
  this->rZCenter = 2048;

  // setting up the default values for microSecondsPerDegree (empirically determined)
  this->countsPerDegree = 34.1333333;
  
  // writing all servos to middle position
  this->dxl->write(DXL_ID_YAW_L, GOAL_POSITION_ADDR, (uint8_t*)&lXCenter, GOAL_POSITION_ADDR_LEN, TIMEOUT); //writes to left yaw
  this->dxl->write(DXL_ID_PITCH_L, GOAL_POSITION_ADDR, (uint8_t*)&lZCenter, GOAL_POSITION_ADDR_LEN, TIMEOUT);//writes to left pitch
  this->dxl->write(DXL_ID_YAW_R, GOAL_POSITION_ADDR, (uint8_t*)&rXCenter, GOAL_POSITION_ADDR_LEN, TIMEOUT);//writes to right yaw
  this->dxl->write(DXL_ID_PITCH_R, GOAL_POSITION_ADDR, (uint8_t*)&rZCenter, GOAL_POSITION_ADDR_LEN, TIMEOUT);//writes to right pitch

  // setting initial values to the eye subsystem transformation matrices
  this->gTD = KinematicChain::xform(0, 0, 0, (-0.004), (0.0636), (0.0856)); // units in meters
  this->gDL = KinematicChain::xform(0, 0, 0, (-0.03475), 0, 0);
  this->gDR = KinematicChain::xform(0, 0, 0, (0.03475), 0, 0);

  this->gLT = gDL.Inverse() * gTD.Inverse();
  this->gRT = gDR.Inverse() * gTD.Inverse();

  //SerialTerminal->println("Finished setting up eyes");
}

/*
   Parallax function to make eye servos look at
   desired coordinate position. Left and right dot positions indicate the desired
   position on the screen in the frame of reference of the left and right eyes, that the eyes need to look at.
*/
void Eyes::parallax(BLA::Matrix<4> leftDotPos, BLA::Matrix<4> rightDotPos)
{
  // calculating angles of rotation for the left eye
  float alphaLeft = atan2(leftDotPos(1), leftDotPos(0)) * (180 / M_PI);
  float betaLeft = atan2(leftDotPos(2), sqrt((leftDotPos(0) * leftDotPos(0)) + (leftDotPos(1) * leftDotPos(1)))) * (180 / M_PI);

  // calculating angles of rotation for the right eye
  float alphaRight  = atan2(rightDotPos(1), rightDotPos(0)) * (180 / M_PI);
  float betaRight = atan2(rightDotPos(2), sqrt((rightDotPos(0) * rightDotPos(0)) + (rightDotPos(1) * rightDotPos(1)))) * (180 / M_PI);

  // converting degrees to counts
  alphaLeft = alphaLeft * this->countsPerDegree;
  betaLeft = betaLeft * this->countsPerDegree;
  alphaRight = alphaRight * this->countsPerDegree;
  betaRight = betaRight * this->countsPerDegree;

  this->goalPosition = this->lXCenter + static_cast<uint16_t>(alphaLeft);
  this->dxl->write(DXL_ID_YAW_L, GOAL_POSITION_ADDR, (uint8_t*)&goalPosition, GOAL_POSITION_ADDR_LEN, TIMEOUT); //writes to left yaw
  
  this->goalPosition = this->lZCenter + static_cast<uint16_t>(betaLeft);
  this->dxl->write(DXL_ID_PITCH_L, GOAL_POSITION_ADDR, (uint8_t*)&goalPosition, GOAL_POSITION_ADDR_LEN, TIMEOUT);//writes to left pitch
  
  this->goalPosition = this->rXCenter + static_cast<uint16_t>(alphaRight);
  this->dxl->write(DXL_ID_YAW_R, GOAL_POSITION_ADDR, (uint8_t*)&goalPosition, GOAL_POSITION_ADDR_LEN, TIMEOUT);//writes to right yaw
  
  this->goalPosition = this->rZCenter - static_cast<uint16_t>(betaRight);
  this->dxl->write(DXL_ID_PITCH_R, GOAL_POSITION_ADDR, (uint8_t*)&goalPosition, GOAL_POSITION_ADDR_LEN, TIMEOUT);//writes to right pitch

  delay(50);
}

/*
   Gets the inverse transformation that goes from center of left eye to top of neck.
*/
BLA::Matrix<4, 4> Eyes::GetInverseLeftEyeTransformation()
{
  this->gLT = gDL.Inverse() * gTD.Inverse();
  return this->gLT;
}

/*
   Gets the inverse transformation that goes from the center of right eye to top of neck.
*/
BLA::Matrix<4, 4> Eyes::GetInverseRightEyeTransformation()
{
  this->gRT = gDL.Inverse() * gTD.Inverse();
  return this->gRT;
}

/*
   Function to write the calibration variables
   for Eye Servos to EEPROM.
*/
void Eyes::WriteEyeCalibrationVariablesToProm()
{
  PromAddress eeAddress = LXCenter;

  EEPROM.put(eeAddress, this->lXCenter);
  // incrementing the address variable by 4 because
  // it is the size of a floating point data type
  eeAddress = (PromAddress)((int)eeAddress + 4);
  EEPROM.put(eeAddress, this->lZCenter);
  eeAddress = (PromAddress)((int)eeAddress + 4);
  EEPROM.put(eeAddress, this->rXCenter);
  eeAddress = (PromAddress)((int)eeAddress + 4);
  EEPROM.put(eeAddress, this->rZCenter);

  //SerialTerminal->println("Eye Calibration Variables written to Prom");
}

/*
   Function to read the calibration variables
   for Eye Servos from EEPROM.
*/
void Eyes::ReadEyeCalibrationVariablesFromProm()
{
  PromAddress eeAddress = LXCenter;
  long int servoCenter = 0.0;

  EEPROM.get(eeAddress, servoCenter);
  this->lXCenter = servoCenter;
  eeAddress = (PromAddress)((int)eeAddress + 4);

  EEPROM.get(eeAddress, servoCenter);
  this->lZCenter = servoCenter;
  eeAddress = (PromAddress)((int)eeAddress + 4);

  EEPROM.get(eeAddress, servoCenter);
  this->rXCenter = servoCenter;
  eeAddress = (PromAddress)((int)eeAddress + 4);

  EEPROM.get(eeAddress, servoCenter);
  this->rZCenter = servoCenter;

  //SerialTerminal->println("Eye Calibration Variables read from Prom");
}

/*
   Function to make the eyes parallax to a single point
   on the screen. Parameter screenDotPos gives the coordinates of
   the desired position.
*/
void Eyes::ParallaxEyesToPos(BLA::Matrix<4> screenDotPos, BLA::Matrix<4, 4> gLS, BLA::Matrix<4, 4> gRS)
{
  BLA::Matrix<4> leftDotPos = gLS * screenDotPos;
  BLA::Matrix<4> rightDotPos = gRS * screenDotPos;

  // calling private helper function to write to servos.
  this->parallax(leftDotPos, rightDotPos);
}

/*
   Function to calibrate the eye servos. The function takes eyeCalCommand
   as a parameter, which is the signal transmitted by the Windows API.
   This character is used as input to the calibration sequence.
*/
void Eyes::CalibrateEyes(char eyeCalCommand, BLA::Matrix<4, 4> gLS, BLA::Matrix<4, 4> gRS)
{
  // Dot position array corresponding to desired screen location
  // set to zero for calibration to calibrate to center of screen.
  BLA::Matrix<4> screenDotPos = {0, 0, 0, 1};

  if (eyeCalCommand == '1' || eyeCalCommand == '2' || eyeCalCommand == '3' || eyeCalCommand == '4') {
    // converting integer to enumeration
    int motorChoice = eyeCalCommand - '0';
    calibrationMotor = static_cast<EyeMotor>(motorChoice);
  }

  // switch case to calibrate the center location of respective eye servo.
  // the center locations are incremented/decremented by 5 microseconds for each press.
  switch (calibrationMotor)
  {
    case (rightZ) : {
        if (eyeCalCommand == 'u') {
          this->rZCenter -= 1;
        }
        if (eyeCalCommand == 'd') {
          this->rZCenter += 1;
        }
        break;
      }
    case (leftZ) : {
        if (eyeCalCommand == 'u') {
          this->lZCenter += 1;
        }
        if (eyeCalCommand == 'd') {
          this->lZCenter -= 1;
        }
        break;
      }
    case (rightX) : {
        if (eyeCalCommand == 'u') {
          this->rXCenter += 1;
        }
        if (eyeCalCommand == 'd') {
          this->rXCenter -= 1;
        }
        break;
      }
    case (leftX) : {
        if (eyeCalCommand == 'u') {
          this->lXCenter += 1;
        }
        if (eyeCalCommand == 'd') {
          this->lXCenter -= 1;
        }
        break;
      }
    default : break;
  }

  this->ParallaxEyesToPos(screenDotPos, gLS, gRS);
}
