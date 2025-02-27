#include "Arduino.h"
#include "QuadrupedRobot.h"
#include <Servo.h>

QuadrupedRobot::QuadrupedRobot() {
    // int numLegs = 4;
    // int numJoints = 3;
    // int QuadrupedRobot::numLegs = numLegs;
    // int QuadrupedRobot::numJoints = numJoints;
    QuadrupedRobot::delayTime = 500;
    QuadrupedRobot::defaultMoveTime = 100;
}

void QuadrupedRobot::initialize() {
    QuadrupedRobot::moveHips(45, 1);
    QuadrupedRobot::moveKnees(0, 1);
    QuadrupedRobot::moveAnkles(0, 1);
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 3; j++) {
            QuadrupedRobot::motors[i][j].attach(4*j + i);
        }
    }
    QuadrupedRobot::stand();
}

void QuadrupedRobot::stand() {

    QuadrupedRobot::moveHips(45, 1000);
    QuadrupedRobot::moveKnees(0, 1000);
    QuadrupedRobot::moveAnkles(0, 1000);
    QuadrupedRobot::moveKnees(90, 1000);
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
        case 2:
            correctedAngle = 180 - correctedAngle; //for knees: 90 is horizontal
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

template<typename T> void QuadrupedRobot::executeFunctionOverTime(int moveTime, T&& f) {
    long moveStartTime = millis();
    long stepTime = 0;
    for(int i = 0; i <= moveTime; i += stepTime) {
        stepTime = millis() - moveStartTime;
        f(i);
    }
}

void QuadrupedRobot::moveIndexOverTime(int i, int angle, int moveTime) {

    int startAngle[] = {0, 0, 0, 0};
    for(int j = 0; j < 4; j++) {
        int setAngle = QuadrupedRobot::motors[j][i].read();
        startAngle[j] = setAngle;
    }
    auto function = [=](int t) {
        for(int j = 0; j < 4; j++) {
           int moveAngle = map(t, moveTime, startAngle[j], angle);
           moveAngle = constrain(moveAngle, startAngle[j], angle);
           QuadrupedRobot::moveJoint(j, i, moveAngle);
        }
    };

    QuadrupedRobot::executeFunctionOverTime(moveTime, function);
}

void QuadrupedRobot::moveIndex(int i, int angle) {
    for(int j = 0; j < 4; j++) {
        QuadrupedRobot::moveJoint(j, i, angle);
    }
    // delay(QuadrupedRobot::delayTime);
}

void QuadrupedRobot::moveHips(int angle, int moveTime) {
    QuadrupedRobot::moveIndexOverTime(0, angle, moveTime);
}

void QuadrupedRobot::moveKnees(int angle, int moveTime) {
    QuadrupedRobot::moveIndexOverTime(1, angle, moveTime);
}

void QuadrupedRobot::moveAnkles(int angle, int moveTime) {
    QuadrupedRobot::moveIndexOverTime(2, angle, moveTime);
}

void QuadrupedRobot::moveAligned(int hipAngle, int kneeAngle) {
    QuadrupedRobot::moveHips(hipAngle, QuadrupedRobot::defaultMoveTime);
    QuadrupedRobot::moveKnees(kneeAngle, QuadrupedRobot::defaultMoveTime);
    QuadrupedRobot::moveAnkles(90-kneeAngle, QuadrupedRobot::defaultMoveTime);
}