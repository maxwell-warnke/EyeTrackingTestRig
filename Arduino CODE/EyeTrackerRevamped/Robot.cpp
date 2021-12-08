/*
   Robot source file containing definitions for all function declarations in
   header file.
*/

#include "Robot.h"

/*
   Robot class constructor.
*/
Robot::Robot()
{
  this->robotState = MenuMode;
  this->screenDotPos = {0, 0, 0, 1};
  SerialTerminal->println("Created Robot Object");
}

Robot::~Robot()
{
}

/*
   Initialization function for the robot class that
   will create the robot eye, shoulder and neck objects.
*/
void Robot::init()
{
  this->robotEyes.init();
  this->robotShoulder.init();
  //this->robotNeck.init();

  this->robotEyes.ReadEyeCalibrationVariablesFromProm();

  // reading the last shoulder position from prom and updating the kinematic chain
  this->robotShoulder.ReadShoulderPositionFromProm();

  // reading the last neck position from prom and updating the kinematic chain
  //this->robotNeck.ReadNeckPositionFromProm();
  //this->robotNeck.SetLastNeckPosition();

  this->updateKinematicChain();
}

/*
   Function to get a command from the SerialTerminal
   that obtains commands from the windows API
*/
char Robot::getSerialCommand()
{
  char command = ' ';
  if (SerialTerminal->available()) {
    command = SerialTerminal->read();
  }
  return command;
}

/*
   Function that will be used to update the kinematic chain instance.
   Calling this function will recalculate the entire kinematic chain for the robot
   going from the eyes to the screen. Therefore this function must be called whenever the robot has moved.
*/
void Robot::updateKinematicChain()
{
//  this->gLS = this->robotEyes.GetInverseLeftEyeTransformation() * this->robotNeck.GetInverseNeckTransformation() * this->robotShoulder.GetInverseShoulderTransformation();
//  this->gRS = this->robotEyes.GetInverseRightEyeTransformation() * this->robotNeck.GetInverseNeckTransformation() * this->robotShoulder.GetInverseShoulderTransformation();
  
  this->gLS = this->robotEyes.GetInverseLeftEyeTransformation() *  this->robotShoulder.GetInverseShoulderTransformation();
  this->gRS = this->robotEyes.GetInverseRightEyeTransformation() * this->robotShoulder.GetInverseShoulderTransformation();
}

/*
   Function to obtain serial communication signals from Windows API.
   Resembles the idle state of the machine. s command is sent to change the state
   to achieve the desired functionality.
*/
void Robot::runMenuModeState()
{
  char command = this->getSerialCommand();

  if (command == 'm') { //returns current mode (for now should always be menuMode)
    SerialTerminal->println("Mode: " + stateNames[this->GetState()]);
  }

  else if (command == 's') {
    int stateNumber = SerialTerminal->parseInt();
    this->SetState(stateNumber);
  }
}

/*
   Function to move the desired subsystem to their desired positions.
   This function is called immediately after the g code is sent from the API or serial communication.
*/
void Robot::setRobotPosition(char command)
{
  switch (command) {
    case ('e'): {
        float goX = SerialTerminal->parseFloat();
        float goY = SerialTerminal->parseFloat();
        this->screenDotPos(0) = goX;
        this->screenDotPos(2) = goY;

        break;
      }
    case ('s'): {
        float goX = SerialTerminal->parseFloat();
        float goY = SerialTerminal->parseFloat();
        float goZ = SerialTerminal->parseFloat();

        this->robotShoulder.MoveShoulderToPosition(-goX, -goY, -goZ);
        break;
      }
//    case ('n'): {
//        float phiR = SerialTerminal->parseFloat();
//        float phiS = SerialTerminal->parseFloat();
//        float phiD = SerialTerminal->parseFloat();
//
//        this->robotNeck.MoveNeckManually(phiR, phiS, phiD);        
//        break;
//      }
    default: break;
  }
  this->updateKinematicChain();
}

/*
   Function to get the current position of the desired subsystem. This function is called
   immediately after the p command is sent from the API or serial communication.
*/
void Robot::getRobotPosition(char command)
{
  switch (command) {
    case ('e'): {
        SerialTerminal->print(this->screenDotPos(0));
        SerialTerminal->print(", ");
        SerialTerminal->println(this->screenDotPos(2));

        break;
      }
    case ('s'): {
        SerialTerminal->print(this->robotShoulder.GetShoulderPosition('x'));
        SerialTerminal->print(", ");
        SerialTerminal->print(this->robotShoulder.GetShoulderPosition('y'));
        SerialTerminal->print(", ");
        SerialTerminal->println(this->robotShoulder.GetShoulderPosition('z'));

        break;
      }
//    case ('n'): {
//        SerialTerminal->print(this->robotNeck.GetLastNeckPosition("PhiR"));
//        SerialTerminal->print(", ");
//        SerialTerminal->print(this->robotNeck.GetLastNeckPosition("PhiS"));
//        SerialTerminal->print(", ");
//        SerialTerminal->println(this->robotNeck.GetLastNeckPosition("PhiD"));
//        break;
//      }
    default: break;
  }
}

/*
   Function that executes the RunRobot state. This function can be used to set the positions
   of the desired subsystem of the robot i.e. obtain manual control of the desired subsystem
   and can also be used to get the positions of the desired subsystem.
*/
void Robot::runRobotRunState()
{
  char command = this->getSerialCommand();

  if (command == 'g') {
    char newCommand[1];
    SerialTerminal->readBytes(newCommand, 1);

    this->setRobotPosition(newCommand[0]);
  }

  else if (command == 'p') {
    char newCommand[1];
    SerialTerminal->readBytes(newCommand, 1);

    this->getRobotPosition(newCommand[0]);
  }

  else if (command == 'm') {
    this->robotShoulder.WriteShoulderPositionToProm();
//
//    this->robotNeck.WriteNeckPositionToProm();

    this->SetState(static_cast<int>(MenuMode));
  }
//
//  this->robotNeck.RunSteppers();
  this->robotEyes.ParallaxEyesToPos(this->screenDotPos, this->gLS, this->gRS);
}

/*
   Function to obtain calibration commands from API and
   subsequently pass message onto Eye subsystem class.
*/
void Robot::runEyeCalibrationState()
{
  char eyeCalCommand = this->getSerialCommand();

  // calling robotEyes object to perform calibration of eyes.
  this->robotEyes.CalibrateEyes(eyeCalCommand, this->gLS, this->gRS);

  // if calibration has been stopped, return to MenuMode;
  if (eyeCalCommand == 'm')
  {
    this->robotEyes.WriteEyeCalibrationVariablesToProm();
    this->SetState(static_cast<int>(MenuMode));
  }
}

/*
   Shoulder Home state will home the steppers. Homing the steppers will include moving the
   steppers till they hit the limit switches. Hitting the limit switches will reset the zero locations
   of the steppers.
*/
void Robot::runShoulderCalibrationState()
{
  // Home all steppers function will return false when steppers have finished homing
  bool calibrating = this->robotShoulder.HomeShoulder();

  char command = this->getSerialCommand();


  if (calibrating == false)
  {
    this->updateKinematicChain();
    this->robotShoulder.WriteShoulderPositionToProm();
    this->SetState(static_cast<int>(MenuMode));
  }
}

/*
   Function to obtain calibration commands for the neck from the API
   and subsequently pass them onto the neck object.
*/
//void Robot::runNeckCalibrationState()
//{
//  char neckCalCommand = this->getSerialCommand();
//
//  // calling robotEyes object to perform calibration of eyes.
//  this->robotNeck.CalibrateNeck(neckCalCommand);
//
//  // if calibration has been stopped, return to MenuMode;
//  if (neckCalCommand == 'm')
//  {
//    this->updateKinematicChain();
//
//    // writing last neck position variables to prom
//    this->robotNeck.WriteNeckPositionToProm();
//    this->SetState(static_cast<int>(MenuMode));
//  }
//}

/*
   Function to set the state for the Robot.
*/
void Robot::SetState(int stateNumber)
{
  this->robotState = static_cast<StateMachineState>(stateNumber);
  //SerialTerminal->println("Set New State");
}

/*
   Function to get the current state of the Robot.
*/
int Robot::GetState()
{
  return static_cast<int>(this->robotState);
  //SerialTerminal->println("Current State is " + String(this->robotState));
}

/*
   Function to determine the state of the robot, and run the
   corresponding action of the state.
*/
void Robot::RunState()
{
  switch (this->robotState)
  {
    case (MenuMode):
      {
        (this)->runMenuModeState();
        break;
      }
    case (EyeCalibrate):
      {
        (this)->runEyeCalibrationState();
        break;
      }
    case (ShoulderCalibrate):
      {
        (this)->runShoulderCalibrationState();
        break;
      }
//    case (NeckCalibrate):
//      {
//        (this)->runNeckCalibrationState();
//        break;
//      }
    case (RobotRun):
      {
        (this)->runRobotRunState();
        break;
      }
    default :
      {
        break;
      }
  }
}
