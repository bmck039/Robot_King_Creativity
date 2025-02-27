#ifndef QuadrupedRobot_h
#define QuadrupedRobot_h
#include "Arduino.h"
#include "Servo.h"

class QuadrupedRobot {
    public:
        QuadrupedRobot();
        void initialize();
        void stand();
        void moveJoint(int legNum, int jointNum, int angle);
        void moveHips(int angle);
        void moveKnees(int angle);
        void moveAnkles(int angle);
        void QuadrupedRobot::moveAligned(int hipAngle, int kneeAngle)
        bool indexIsAtAngle(int i, int angle);
        int delayTime;
    private:
        int correctAngle(int legNum, int jointNum, int angle);
        void moveIndex(int i, int angle);
        Servo motors[4][3];
        
};

#endif