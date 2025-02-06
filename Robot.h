#ifndef Robot_h
#define Robot_h
#include "Arduino.h"
#include "Servo.h"

class Robot {
    public:
        Robot(int numLegs, int numJoints);
        void initialize();
        void stand();
        void moveJoint(int legNum, int jointNum, int angle);
        void moveHips(int angle);
        void moveKnees(int angle);
        void moveAnkles(int angle);
        bool indexIsAtAngle(int i, int angle);
        int delayTime;
    private:
        int correctAngle(int legNum, int jointNum, int angle);
        void moveIndex(int i, int angle);
        Servo motors[][];
        
}

#endif