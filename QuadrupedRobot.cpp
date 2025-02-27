#include "Arduino.h"
#include "QuadrupedRobot.h"
#include <Servo.h>

QuadrupedRobot::QuadrupedRobot() {
    // int numLegs = 4;
    // int numJoints = 3;
    // int QuadrupedRobot::numLegs = numLegs;
    // int QuadrupedRobot::numJoints = numJoints;
    QuadrupedRobot::delayTime = 500;
}

void QuadrupedRobot::initialize() {
    QuadrupedRobot::moveHips(45);
    QuadrupedRobot::moveKnees(0);
    QuadrupedRobot::moveAnkles(90);
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 3; j++) {
            QuadrupedRobot::motors[i][j].attach(4*j + i);
        }
    }
    QuadrupedRobot::stand();
}

void QuadrupedRobot::stand() {

    if(! QuadrupedRobot::indexIsAtAngle(0, 45)) {
        QuadrupedRobot::moveHips(45);
        // delay(QuadrupedRobot::delayTime);
    } 

    if(! QuadrupedRobot::indexIsAtAngle(1, 0)) {
        QuadrupedRobot::moveKnees(0);
        // delay(QuadrupedRobot::delayTime);
    } 
    if(! QuadrupedRobot::indexIsAtAngle(2, 90)) {
        QuadrupedRobot::moveAnkles(90);
        // delay(QuadrupedRobot::delayTime);
    }
    QuadrupedRobot::moveKnees(90);
}

bool QuadrupedRobot::indexIsAtAngle(int i, int angle) {
    bool isAtAngle = true;
    for (int j = 0; j < 4 && isAtAngle; j++)
    {
        int setAngle = QuadrupedRobot::motors[j][i].read();
        int correctedAngle = QuadrupedRobot::correctAngle(j, i, angle);
        isAtAngle = isAtAngle && (setAngle == correctedAngle);
    }
    return isAtAngle;
    
}

void QuadrupedRobot::moveIndex(int i, int angle) {
    for(int j = 0; j < 4; j++) {
        QuadrupedRobot::moveJoint(j, i, angle);
    }
    delay(QuadrupedRobot::delayTime);
}

void QuadrupedRobot::moveHips(int angle) {
    QuadrupedRobot::moveIndex(0, angle);
}

void QuadrupedRobot::moveKnees(int angle) {
    QuadrupedRobot::moveIndex(1, angle);
}

void QuadrupedRobot::moveAnkles(int angle) {
    QuadrupedRobot::moveIndex(2, angle);
}

int QuadrupedRobot::correctAngle(int legNum, int jointNum, int angle) {
    int correctedAngle;
    if(legNum == 0 or legNum == 2) {
        correctedAngle = 180 - angle;
    } else {
        correctedAngle = angle;
    }
    switch (jointNum) {
        case 0:
            if(correctedAngle > 135) {
                correctedAngle = 135; // safeguard to prevent legs crashing into the body
            }
            break;
        case 1:
            correctedAngle = 180 - correctedAngle; //for knees: 0 is horizontal
            break;
        
        default:
            break;
    }
    return correctedAngle;
}

void QuadrupedRobot::moveJoint(int legNum, int jointNum, int angle) {
    int correctedAngle = QuadrupedRobot::correctAngle(legNum, jointNum, angle);
    QuadrupedRobot::motors[legNum][jointNum].write(correctedAngle);
}