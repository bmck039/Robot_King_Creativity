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
    for(char i = 0; i < 4; i++) {
        for(char j = 0; j < 3; j++) {
            QuadrupedRobot::motors[i][j].attach(4*j + i);
        }
    }
    // delay(1000);
    QuadrupedRobot::stand();
}

void QuadrupedRobot::stand() {

    QuadrupedRobot::safePosition();
    delay(defaultMoveTime);
    QuadrupedRobot::moveKnees(90, 1000);
    delay(defaultMoveTime);
}

void QuadrupedRobot::moveLeg(char i, int legAngles[], int moveTime) {
    int moveAngles[4][3];
    memcpy(&moveAngles[0][0], &setAngles[0][0], sizeof(setAngles[0][0]) * 4 * 3);
    for(char j = 0; j < 3; j++) {
        moveAngles[i][j] = legAngles[j];
    }

    QuadrupedRobot::moveJoints(moveAngles, moveTime);
}

bool QuadrupedRobot::indexIsAtAngle(char i, int angle) {
    bool isAtAngle = true;
    for (char j = 0; j < 4 && isAtAngle; j++) {
        int setAngle = QuadrupedRobot::motors[j][i].read();
        char correctedAngle = QuadrupedRobot::correctAngle(j, i, angle);
        isAtAngle = isAtAngle && (setAngle == correctedAngle);
    }
    return isAtAngle;
    
}

char QuadrupedRobot::correctAngle(char legNum, char jointNum, int angle) {
    int correctedAngle;
    angle += calibrationArray[legNum][jointNum];
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
    correctedAngle = (correctedAngle, 0, 180);
    return correctedAngle;
}

void QuadrupedRobot::setCalibration(signed char offsetArray[4][3]) {
    memcpy(&calibrationArray[0][0], &offsetArray[0][0], sizeof(offsetArray[0][0]) * 4 * 3);
}

void QuadrupedRobot::calibrate() {
    Serial.begin(9600);
    String arrayResult;
    for(char i = 0; i < 4; i++) {
        for(char j = 0; j < 3; j++) {

            char currentOffset = 0;
            bool notEmpty = true;
            while(notEmpty) {
                signed char result = Serial.parseInt();
                notEmpty = !(result == -1);
                if(notEmpty) {
                    currentOffset += result;
                }
                QuadrupedRobot::calibrationArray[i][j] = currentOffset;
                QuadrupedRobot::moveJoint(i, j, 0);
            }
            arrayResult.concat(currentOffset);
            arrayResult.concat("\t");
        }
        arrayResult.concat("\n");
    }
    Serial.println(arrayResult);
    Serial.end();
}

void QuadrupedRobot::moveJoint(char legNum, char jointNum, int angle) {
    setAngles[legNum][jointNum] = angle;
    char correctedAngle = QuadrupedRobot::correctAngle(legNum, jointNum, angle);
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

void QuadrupedRobot::moveIndexOverTime(char i, int angle, int moveTime) {
    int moveAngles[4][3];
    memcpy(&moveAngles[0][0], &setAngles[0][0], sizeof(setAngles[0][0]) * 4 * 3);
    for(char j = 0; j < 4; j++) {
        moveAngles[j][i] = angle;
    }
    if(! QuadrupedRobot::indexIsAtAngle(i, angle)) {
        QuadrupedRobot::moveJoints(moveAngles, moveTime);
    }
    
    // delay(moveTime);
}

void QuadrupedRobot::moveJoints(int moveAngles[4][3], int moveTime) {
    int startAngles[4][3];
    memcpy(&startAngles[0][0], &setAngles[0][0], sizeof(setAngles[0][0]) * 4 * 3);
    auto function = [=](int t) {
        for(char i = 0; i < 4; i++) {
            for(char j = 0; j < 3; j++) {
                int angle = moveAngles[i][j];
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

void QuadrupedRobot::moveIndex(char i, int angle) {
    for(char j = 0; j < 4; j++) {
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
    int moveAngles[4][3];
    for(char i = 0; i < 4; i++) {
        moveAngles[i][0] = hipAngle;
        moveAngles[i][1] = kneeAngle;
        moveAngles[i][2] = kneeAngle - 90;
    }
    QuadrupedRobot::moveJoints(moveAngles, QuadrupedRobot::defaultMoveTime);
}

void QuadrupedRobot::safePosition() {
    int setAngles[4][3] = {{45, 0, 0}, {45, 0, 0}, {45, 0, 0}, {45, 0, 0}};
    QuadrupedRobot::moveJoints(setAngles, 1000);
}