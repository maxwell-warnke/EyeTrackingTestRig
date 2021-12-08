/*
   This is the header file for the robot class. The robot class is the logic engine of the robot.
   The robot class is responsible for holding the eyes, shoulder and neck objects.
   It also sets the state of the robot and has additional private-public functions
   to run the corresponding states.
*/
#ifndef _ROBOT_H
#define _ROBOT_H

//#include "NeckClass.h"
#include "EyeSubsystem.h"
#include "Shoulder.h"
#include <BasicLinearAlgebra.h>

/*
   Class declaration for robot class.
*/
class Robot
{
    /*
       Private objects consist of the state, eyes, shoulder and neck objects.
       Also desired gaze coordinates.
       Private functions consist of the different functions implemented by each state.
    */
  private:
    StateMachineState robotState;
    Eyes robotEyes;
    Shoulder robotShoulder;
    //RobotNeck robotNeck;
    //gaze coordinates
    BLA::Matrix<4> screenDotPos;

    char getSerialCommand();

    void updateKinematicChain();

    void setRobotPosition(char command);
    void getRobotPosition(char command);

    void runMenuModeState();
    void runRobotRunState();

    void runEyeCalibrationState();
    void runShoulderCalibrationState();
    //void runNeckCalibrationState();

  protected:
    /*
        S - will be the screen frame of reference with the zero coordinate at the center of the screen,
          X will be to the right, Z up, and Y into the screen
        L - will be the left eye's frame of reference with the origin at the center of rotation
        and is aligned with the D frame when the 90-degree angle is commanded
        R - will be the right eye's frame of reference with the origin at the center of rotation
        and is aligned with D frame when the 90-degree angle is commanded
    */
    BLA::Matrix<4, 4> gLS, gRS;


  public:
    Robot();
    ~Robot();
    void init();
    void SetState(int stateNumber);
    int GetState();
    void RunState();
};

#endif
