#ifndef QuadrupedRobot_h
#define QuadrupedRobot_h
#include "Arduino.h"
#include "Servo.h"

class QuadrupedRobot {
    public:
        QuadrupedRobot();
        void initialize();
        void stand();
        template<typename T> void executeFunctionOverTime(int moveTime, T&& f);
        void moveJoint(int legNum, int jointNum, int angle);
        void moveHips(int angle, int moveTime);
        void moveKnees(int angle, int moveTime);
        void moveAnkles(int angle, int moveTime);
        void moveAligned(int hipAngle, int kneeAngle);
        bool indexIsAtAngle(int i, int angle);
        void moveLeg(int i, int legAngles[], int moveTime);
        void moveJoints(int moveAngles[4][3], int moveTime);
        void safePosition();
        void setCalibration(int offsetArray[4][3]);
        void calibrate();
        int delayTime;
        int defaultMoveTime;
        int setAngles[4][3];
        signed int calibrationArray[4][3];

    private:
        int correctAngle(int legNum, int jointNum, int angle);
        void moveIndexOverTime(int i, int angle, int moveTime);
        void moveIndex(int i, int angle);
        Servo motors[4][3];
};

#endif