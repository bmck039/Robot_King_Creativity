#ifndef QuadrupedRobot_h
#define QuadrupedRobot_h
#include "Arduino.h"
#include "Servo.h"

class QuadrupedRobot {
    public:
        //Initializer
        QuadrupedRobot();

        //attaches motors and stands
        void initialize();

        //moves the motors to stand
        void stand();

        //Executes an input function incrementally over the course of moveTime milliseconds. the input function f should be a lambda function that takes an integer parameter that controls the execution progress
        template<typename T> void executeFunctionOverTime(int moveTime, T&& f);

        //moves a particular joint to a specified angle
        void moveJoint(int legNum, int jointNum, int angle);

        //moves all hips to the specified angle over the course of moveTime ms
        void moveHips(int angle, int moveTime);

        //moves all knees to the specified angle over the course of moveTime ms
        void moveKnees(int angle, int moveTime);

        //moves all ankles to the specified angle over the course of moveTime ms
        void moveAnkles(int angle, int moveTime);

        //moves hips and knees to the specified angles and moves the ankles to a calculated angle to keep them pointing straight down
        void moveAligned(int hipAngle, int kneeAngle);

        //checks to see if a particular joint type is at a specified angle
        bool indexIsAtAngle(int i, int angle);

        //moves a particular leg to the specified angles over the course of moveTime ms.
        void moveLeg(int i, int legAngles[], int moveTime);

        //moves all joints to the specified angles over the course of moveTime ms.
        void moveJoints(int moveAngles[4][3], int moveTime);

        //moves the joints to a safe position
        void safePosition();

        //sets the internal calibration array
        void setCalibration(int offsetArray[4][3]);

        //runs a Serial-interactive loop to adjust the calibration array. Sets the zero-point of each motor. Hips should point at right-angles to the body, knees should point fully vertical, and ankles should be at right-angles to the legs
        void calibrate();

        //positions the leg in such a way that the tip of the leg occupies the coordinates (x,y,z) millimeters in space in reference to the leg joint
        void positionFromCoordinates(int x, int y, int z);

        //returns all 12 leg servo angles in an array
        int getCurrentPosition();

        //time between motor moves
        int delayTime;

        //default time that a move takes
        int defaultMoveTime;

    private:

        //internal function to transform an angle into local coordinates for a particular motor
        int correctAngle(int legNum, int jointNum, int angle);

        //internal function to move a particular joint index to an angle over moveTime ms.
        void moveIndexOverTime(int i, int angle, int moveTime);

        //internal function to move a particular joint index to an angle immediately
        void moveIndex(int i, int angle);
        Servo motors[4][3];

        //internal array to keep track of the angles each motor is set to
        int setAngles[4][3];

        //internal array to keep track of calibration values
        int calibrationArray[4][3];

        //Private calculation for aligning all coordinate planes for the leg position function
        int xSign();

        //Private calculation for aligning all coordinate planes for the leg position function
        int ySign();

        //leg length
        int segmentLLength;

        //Base length
        int segmentBLength;

        //Claw length
        int segmentCLength;
};

#endif