#include "Arduino.h"
#include "QuadrupedRobot.h"
#include <Servo.h>
#include <math.h>

// QuadrupedRobot::width = 70;
// QuadrupedRobot::

//Initializer
QuadrupedRobot::QuadrupedRobot() {
    // int numLegs = 4;
    // int numJoints = 3;
    // int QuadrupedRobot::numLegs = numLegs;
    // int QuadrupedRobot::numJoints = numJoints;
    QuadrupedRobot::delayTime = 10;
    QuadrupedRobot::defaultMoveTime = 250;
    //Standardized in millimeters
    
    QuadrupedRobot::segmentLLength = 65; //Leg
    QuadrupedRobot::segmentBLength = 33; //Base
    QuadrupedRobot::segmentCLength = 63; //Claw
}

QuadrupedRobot::QuadrupedRobot(int legLength, int baseLength, int clawLength) {
    //Standardized in millimeters
    QuadrupedRobot::segmentLLength = legLength; //Leg
    QuadrupedRobot::segmentBLength = baseLength; //Base
    QuadrupedRobot::segmentCLength = clawLength; //Claw

    QuadrupedRobot::delayTime = 10;
    QuadrupedRobot::defaultMoveTime = 250;
}

//attaches motors and stands
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

//moves the motors to stand
void QuadrupedRobot::stand() {

    QuadrupedRobot::safePosition();
    delay(defaultMoveTime);
    QuadrupedRobot::moveKnees(90, 1000);
    delay(defaultMoveTime);
}

//moves a particular leg to the specified angles over the course of moveTime ms.
void QuadrupedRobot::moveLeg(int i, int legAngles[], int moveTime) {
    int moveAngles[4][3];
    memcpy(&moveAngles[0][0], &setAngles[0][0], sizeof(setAngles[0][0]) * 4 * 3);
    for(int j = 0; j < 3; j++) {
        moveAngles[i][j] = legAngles[j];
    }

    QuadrupedRobot::moveJoints(moveAngles, moveTime);
}

//checks to see if a particular joint type is at a specified angle
bool QuadrupedRobot::indexIsAtAngle(int i, int angle) {
    bool isAtAngle = true;
    for (int j = 0; j < 4 && isAtAngle; j++) {
        int setAngle = QuadrupedRobot::motors[j][i].read();
        int correctedAngle = QuadrupedRobot::correctAngle(j, i, angle);
        isAtAngle = isAtAngle && (setAngle == correctedAngle);
    }
    return isAtAngle;
    
}


//internal function to transform an angle into local coordinates for a particular motor
int QuadrupedRobot::correctAngle(int legNum, int jointNum, int angle) {
    int correctedAngle;
    angle += QuadrupedRobot::calibrationArray[legNum][jointNum];
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
    correctedAngle = constrain(correctedAngle, 0, 180);
    return correctedAngle;
}

//sets the internal calibration array
void QuadrupedRobot::setCalibration(int offsetArray[4][3]) {
    memcpy(&QuadrupedRobot::calibrationArray[0][0], &offsetArray[0][0], sizeof(offsetArray[0][0]) * 4 * 3);
}

//runs a Serial-interactive loop to adjust the calibration array. Sets the zero-point of each motor. Hips should point at right-angles to the body, knees should point fully vertical, and ankles should be at right-angles to the legs
void QuadrupedRobot::calibrate() {
    Serial.begin(9600);
    String arrayResult = String("{");
    for(int i = 0; i < 4; i++) {
        arrayResult.concat("{");
        for(int j = 0; j < 3; j++) {
            int currentOffset = 0;
            bool notEmpty = true;
            Serial.print("Calibrating Leg Number ");
            Serial.print(i);
            Serial.print(" and Motor Number ");
            Serial.println(j);
            QuadrupedRobot::moveJoint(i, j, 0);
            while(notEmpty) {
                int result = Serial.parseInt();
                notEmpty = !(result == -1);
                if(notEmpty) {
                    currentOffset += result;
                }
                QuadrupedRobot::calibrationArray[i][j] = currentOffset;
                QuadrupedRobot::moveJoint(i, j, 0);
            }
            arrayResult.concat(currentOffset);
            if(j != 2) {
                arrayResult.concat(", ");
            }
        }
        arrayResult.concat("}");
        if(i != 3) {
            arrayResult.concat(", ");
        }
    }
    arrayResult.concat("}");
    Serial.print("Calibration Array: ");
    Serial.println(arrayResult);
    Serial.end();
}

//moves a particular joint to a specified angle
void QuadrupedRobot::moveJoint(int legNum, int jointNum, int angle) {
    setAngles[legNum][jointNum] = angle;
    int correctedAngle = QuadrupedRobot::correctAngle(legNum, jointNum, angle);
    QuadrupedRobot::motors[legNum][jointNum].write(correctedAngle);
    // delay(delayTime);
}

//Executes an input function incrementally over the course of moveTime milliseconds. the input function f should be a lambda function that takes an integer parameter that controls the execution progress
template<typename T> void QuadrupedRobot::executeFunctionOverTime(int moveTime, T&& f) {
    int moveStartTime = millis();
    int currentTime = 0;
    while(currentTime <= moveTime) {
        currentTime = millis() - moveStartTime;
        f(currentTime);
    }
    f(moveTime);
}

//internal function to move a particular joint index to an angle over moveTime ms.
void QuadrupedRobot::moveIndexOverTime(int i, int angle, int moveTime) {
    int moveAngles[4][3];
    memcpy(&moveAngles[0][0], &setAngles[0][0], sizeof(setAngles[0][0]) * 4 * 3);
    for(int j = 0; j < 4; j++) {
        moveAngles[j][i] = angle;
    }
    if(! QuadrupedRobot::indexIsAtAngle(i, angle)) {
        QuadrupedRobot::moveJoints(moveAngles, moveTime);
    }
    
    // delay(moveTime);
}

//moves all joints to the specified angles over the course of moveTime ms.
void QuadrupedRobot::moveJoints(int moveAngles[4][3], int moveTime) {
    int startAngles[4][3];
    memcpy(&startAngles[0][0], &setAngles[0][0], sizeof(setAngles[0][0]) * 4 * 3);
    auto function = [=](int t) {
        for(int i = 0; i < 4; i++) {
            for(int j = 0; j < 3; j++) {
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

//internal function to move a particular joint index to an angle immediately
void QuadrupedRobot::moveIndex(int i, int angle) {
    for(int j = 0; j < 4; j++) {
        QuadrupedRobot::moveJoint(j, i, angle);
    }
    // delay(QuadrupedRobot::delayTime);
}

//moves all hips to the specified angle over the course of moveTime ms
void QuadrupedRobot::moveHips(int angle, int moveTime) {
    QuadrupedRobot::moveIndexOverTime(0, angle, moveTime);
}

//moves all knees to the specified angle over the course of moveTime ms
void QuadrupedRobot::moveKnees(int angle, int moveTime) {
    QuadrupedRobot::moveIndexOverTime(1, angle, moveTime);
}

//moves all ankles to the specified angle over the course of moveTime ms
void QuadrupedRobot::moveAnkles(int angle, int moveTime) {
    QuadrupedRobot::moveIndexOverTime(2, angle, moveTime);
}

//moves hips and knees to the specified angles and moves the ankles to a calculated angle to keep them pointing straight down
void QuadrupedRobot::moveAligned(int hipAngle, int kneeAngle) {
    int moveAngles[4][3];
    for(int i = 0; i < 4; i++) {
        moveAngles[i][0] = hipAngle;
        moveAngles[i][1] = kneeAngle;
        moveAngles[i][2] = kneeAngle - 90;
    }
    QuadrupedRobot::moveJoints(moveAngles, QuadrupedRobot::defaultMoveTime);
}

//moves the joints to a safe position
void QuadrupedRobot::safePosition() {
    int setAngles[4][3] = {{45, 0, 0}, {45, 0, 0}, {45, 0, 0}, {45, 0, 0}};
    QuadrupedRobot::moveJoints(setAngles, 1000);
}

void QuadrupedRobot::positionFromCoordinates(int legNum, int x, int y, int z) {
    // int xsign = QuadrupedRobot::xSign(legNum);
    // int ysign = QuadrupedRobot::ySign(legNum);
    // x = (xsign*x) - QuadrupedRobot::width/2;
    // y = (ysign*y) - QuadrupedRobot::height/2;
    // if(){

    // }
    // if(){

    // }
    // if(){

    // }
    float pi = 3.14159;
    int hipAngle = atan2(x, y) * 180 / pi;
    if(hipAngle < 0) { hipAngle += 360; }
    float R = (1.0*x)/sin(hipAngle * pi / 180);
    float r = R - QuadrupedRobot::segmentBLength;
    int &c = QuadrupedRobot::segmentCLength;
    int &l = QuadrupedRobot::segmentLLength;

    float squareRoot = sqrt((2*pow(c, 2))*(pow(l, 2) + pow(r, 2) + pow(z, 2)) + (2*pow(l, 2)) * (pow(r, 2) + pow(z, 2)) - 2*pow(r,2) * pow(z, 2) - pow(c, 4) - pow(l, 4) - pow(r, 4) - pow(z, 4));
    int kneeAngle = -2 * atan2((2*l*r + squareRoot), pow(c, 2) - 2*l*z - pow(l, 2) - pow(r, 2) - pow(z, 2)) * 180 / pi;
    int ankleAngle = 2 * atan2((-2*c*l - squareRoot), (pow(c, 2) + pow(l, 2) - pow(r, 2) - pow(z, 2))) * 180 / pi;

    if(kneeAngle < 0) { kneeAngle += 360; }
    if(ankleAngle < -calibrationArray[legNum][2]) { ankleAngle += 360; }

    int angles[] = {hipAngle, kneeAngle, ankleAngle};
    QuadrupedRobot::moveLeg(legNum, angles, QuadrupedRobot::defaultMoveTime);
    Serial.begin(9600);
    Serial.println(hipAngle);
    Serial.println(kneeAngle);
    Serial.println(ankleAngle);
    Serial.end();
}

QuadrupedRobot::AngleArray QuadrupedRobot::getCurrentPosition() {
    struct QuadrupedRobot::AngleArray angles;
    // angles.array = QuadrupedRobot::setAngles;
    memcpy(&angles.array[0][0], &setAngles[0][0], sizeof(setAngles[0][0]) * 4 * 3);
    return angles;
}