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
        void moveJoint(char legNum, char jointNum, int angle);
        void moveHips(int angle, int moveTime);
        void moveKnees(int angle, int moveTime);
        void moveAnkles(int angle, int moveTime);
        void moveAligned(int hipAngle, int kneeAngle);
        bool indexIsAtAngle(char i, int angle);
        void moveLeg(char i, int legAngles[], int moveTime);
        void moveJoints(int moveAngles[4][3], int moveTime);
        void safePosition();
        void setCalibration(signed char offsetArray[4][3]);
        void calibrate();
        char delayTime;
        int defaultMoveTime;
        int setAngles[4][3];
        signed char calibrationArray[4][3];

    private:
        char correctAngle(char legNum, char jointNum, int angle);
        void moveIndexOverTime(char i, int angle, int moveTime);
        void moveIndex(char i, int angle);
        Servo motors[4][3];
};

#endif