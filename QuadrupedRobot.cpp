#include "Arduino.h"
#include "QuadrupedRobot.h"
#include <Servo.h>

QuadrupedRobot::QuadrupedRobot() {
    // int numLegs = 4;
    // int numJoints = 3;
    // int QuadrupedRobot::numLegs = numLegs;
    // int QuadrupedRobot::numJoints = numJoints;
    QuadrupedRobot::delayTime = 10;
    QuadrupedRobot::defaultMoveTime = 250;
}

void QuadrupedRobot::initialize() {
    QuadrupedRobot::moveHips(90, 1);
    QuadrupedRobot::moveKnees(0, 1);
    QuadrupedRobot::moveAnkles(90, 1);
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 3; j++) {
            QuadrupedRobot::motors[i][j].attach(4*j + i);
        }
    }
    // delay(1000);
    QuadrupedRobot::stand();
}

void QuadrupedRobot::stand() {

    QuadrupedRobot::moveHips(45, 1000);
    delay(defaultMoveTime);
    QuadrupedRobot::moveKnees(0, 1000);
    delay(defaultMoveTime);
    QuadrupedRobot::moveAnkles(0, 1000);
    delay(defaultMoveTime);
    QuadrupedRobot::moveKnees(120, 1000);
    delay(defaultMoveTime);
}

void QuadrupedRobot::moveLeg(int i, int hipAngle, int kneeAngle, int ankleAngle, int moveTime) {
    int legSetAngles[] = {hipAngle, kneeAngle, ankleAngle};
    int startAngle[] = {0, 0, 0};
    for(int j = 0; j < 3; j++) {
        startAngle[j] = setAngles[i][j];
    }
    
    auto function = [=](int t) {     
        for(int j = 0; j < 3; j++) {
           int moveAngle = map(t, 0, moveTime, startAngle[j], legSetAngles[j]);
           if(startAngle[j] < legSetAngles[j]) {
                moveAngle = constrain(moveAngle, startAngle[j], legSetAngles[j]);
           } else {
                moveAngle = constrain(moveAngle, legSetAngles[j], startAngle[j]);
           }
           QuadrupedRobot::moveJoint(i, j, moveAngle);
        }
    };

    QuadrupedRobot::executeFunctionOverTime(moveTime, function);
}

bool QuadrupedRobot::indexIsAtAngle(int i, int angle) {
    bool isAtAngle = true;
    for (int j = 0; j < 4 && isAtAngle; j++) {
        int setAngle = QuadrupedRobot::motors[j][i].read();
        int correctedAngle = QuadrupedRobot::correctAngle(j, i, angle);
        isAtAngle = isAtAngle && (setAngle == correctedAngle);
    }
    return isAtAngle;
    
}

int QuadrupedRobot::correctAngle(int legNum, int jointNum, int angle) {
    int correctedAngle;
    if(angle > 135 && jointNum == 0) {
        angle = 135; // safeguard to prevent legs crashing into the body
    }
    if(legNum == 0 or legNum == 2) {
        correctedAngle = 180 - angle;
    } else {
        correctedAngle = angle;
    }
    switch (jointNum) {
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
    setAngles[legNum][jointNum] = angle;
    int correctedAngle = QuadrupedRobot::correctAngle(legNum, jointNum, angle);
    QuadrupedRobot::motors[legNum][jointNum].write(correctedAngle);
    // delay(delayTime);
}

template<typename T> void QuadrupedRobot::executeFunctionOverTime(int moveTime, T&& f) {
    int moveStartTime = millis();
    int currentTime = 0;
    while(currentTime <= moveTime) {
        currentTime = millis() - moveStartTime;
        f(currentTime);
    }
    f(moveTime);
}

void QuadrupedRobot::moveIndexOverTime(int i, int angle, int moveTime) {
    int startAngle[] = {0, 0, 0, 0};
    for(int j = 0; j < 4; j++) {
        startAngle[j] = setAngles[j][i];
    }
    auto function = [=](int t) {     
        for(int j = 0; j < 4; j++) {
           int moveAngle = map(t, 0, moveTime, startAngle[j], angle);
           if(startAngle[j] < angle) {
                moveAngle = constrain(moveAngle, startAngle[j], angle);
           } else {
                moveAngle = constrain(moveAngle, angle, startAngle[j]);
           }
           QuadrupedRobot::moveJoint(j, i, moveAngle);
        }
    };
    if(! QuadrupedRobot::indexIsAtAngle(i, angle)) {
        QuadrupedRobot::executeFunctionOverTime(moveTime, function);
    }
    
    // delay(moveTime);
}

void QuadrupedRobot::moveJoints(int hipAngles[], int kneeAngles[], int ankleAngles[], int moveTime) {
    int startAngles[4][3];
    int motorAngles[4][3];
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 3; j++) {
            startAngles[i][j] = setAngles[i][j];
        }
        motorAngles[i][0] = hipAngles[i];
        motorAngles[i][1] = kneeAngles[i];
        motorAngles[i][2] = ankleAngles[i];
    }

    auto function = [=](int t) {
        for(int i = 0; i < 4; i++) {
            for(int j = 0; j < 3; j++) {
                int angle = motorAngles[i][j];
                int moveAngle = map(t, 0, moveTime, startAngles[i][j], angle);
                if(startAngles[i][j] < angle) {
                        moveAngle = constrain(moveAngle, startAngles[i][j], angle);
                } else {
                        moveAngle = constrain(moveAngle, angle, startAngles[i][j]);
                }
                QuadrupedRobot::moveJoint(i, j, moveAngle);
            }
        }
    };
    executeFunctionOverTime(moveTime, function);
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
    int hipAngles[4];
    int kneeAngles[4];
    int ankleAngles[4];
    for(int i = 0; i < 4; i++) {
        hipAngles[i] = hipAngle;
        kneeAngles[i] = kneeAngle;
        ankleAngles[i] = kneeAngle - 90;
    }
    QuadrupedRobot::moveJoints(hipAngles, kneeAngles, ankleAngles, QuadrupedRobot::defaultMoveTime);
}