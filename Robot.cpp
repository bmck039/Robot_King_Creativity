#include "Arduino.h"
#include "Robot.h"
#include <Servo.h>

Robot::Robot() {
    Servo Robot::motors[4][3];
    int Robot::numLegs = 4;
    int Robot::numJoints = 3;
    int Robot::delayTime = 500;
}

void Robot::initialize() {
    Robot::moveHips(45);
    Robot::moveKnees(0);
    Robot::moveAnkles(90);
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 3; j++) {
            legs[i][j].attach(4*j + i);
        }
    }
    Robot::stand();
}

void Robot::stand() {

    if(! Robot::indexIsAtAngle(2, 45)) {
        Robot::moveAnkles(45);
        delay(Robot::delayTime);
    } 
    if(! Robot::indexIsAtAngle(2, 90)) {
        Robot::moveAnkles(90);
        delay(Robot::delayTime);
    }
    Robot::moveKnees(90);
}

bool Robot::indexIsAtAngle(int i, int angle) {
    bool isAtAngle = true;
    for (int j = 0; j < 4 && isAtAngle; j++)
    {
        int setAngle = motors[j][i].read();
        int correctedAngle = Robot::correctAngle(j, i, angle);
        isAtAngle = isAtAngle && (setAngle == correctedAngle);
    }
    return isAtAngle;
    
}

void Robot::moveIndex(int i, int angle) {
    for(int j = 0; j < Robot::numLegs) {
        Robot::moveJoint(j, i, angle);
    }
}

void Robot::moveHips(int angle) {
    Robot::moveIndex(0, angle);
}

void Robot::moveKnees(int angle) {
    Robot::moveIndex(1, angle);
}

void Robot::moveAnkles(int angle) {
    Robot::moveIndex(2, angle);
}

int Robot::correctAngle(int legNum, int jointNum, int angle) {
    int correctedAngle;
    if(legNum == 0 or legNum == 3) {
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
            correctedAngle = 180 - correctedAngle; //for knees: 0 will be vertical
            break;
        
        default:
            break;
    }
    return correctedAngle;
}

void Robot::moveJoint(int legNum, int jointNum, int angle) {
    int correctedAngle = Robot::correctAngle(legNum, jointNum, angle);
    motors[legNum][jointNum].write(correctedAngle);
}