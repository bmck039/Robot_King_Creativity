#ifndef QuadrupedRobot_h
#define QuadrupedRobot_h
#include "Arduino.h"
#include "Servo.h"

class QuadrupedRobot {
    public:
        QuadrupedRobot();
        void initialize();
        void stand();
        template<typename T> void executeFunctionOverTime(char moveTime, T&& f);
        void moveJoint(char legNum, char jointNum, char angle);
        void moveHips(char angle, char moveTime);
        void moveKnees(char angle, char moveTime);
        void moveAnkles(char angle, char moveTime);
        void moveAligned(char hipAngle, char kneeAngle);
        bool indexIsAtAngle(char i, char angle);
        void moveLeg(char i, char legAngles[], char moveTime);
        void moveJoints(char moveAngles[4][3], char moveTime);
        void safePosition();
        void setCalibration(char offsetArray[4][3]);
        void calibrate();
        char delayTime;
        int defaultMoveTime;
        char setAngles[4][3];
        char calibrationArray[4][3];

    private:
        char correctAngle(char legNum, char jointNum, char angle);
        void moveIndexOverTime(char i, char angle, char moveTime);
        void moveIndex(char i, char angle);
        Servo motors[4][3];
};

#endif