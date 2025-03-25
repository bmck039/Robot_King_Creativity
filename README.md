Robot designed and coded by Ben McKallip and Collin Cook under advising of Jacob Schrum and Steven Alexander.
This library is still a work in progress but is designed to be used with:
* an Arduino Nano microcontroller
* Tower Pro 9g Servos: https://tinyurl.com/yfc457us
* Breakout board: https://tinyurl.com/ykw8mreh
  
The robot will run with any 7.2+ Volt battery with 1.5 Amps current, battery should be wired through the breakout board VIN port.
This library contains many methods to make addressing each leg and each servo much easier, adds methods for moving the robot as well.
Inverse kinematics and related calculations are done within this library

Documentation:
```cpp
//Initializer
QuadrupedRobot();

//Initializer
QuadrupedRobot(int legLength, int baseLength, int clawLength);

//attaches motors and stands
void initialize();

//moves the motors to stand
void stand();

//Executes an input function incrementally over the course of moveTime milliseconds. the input function f should be a lambda function that takes an integer parameter that controls the execution progress
template<typename T> void executeFunctionOverTime(int moveTime, T&& f);

//moves a particular joint to a specified angle
void moveJoint(int legNum, int jointNum, int angle);

//moves all hips to the specified angle over the course of moveTime ms
void moveHips(int angle, int moveTime);

//moves all knees to the specified angle over the course of moveTime ms
void moveKnees(int angle, int moveTime);

//moves all ankles to the specified angle over the course of moveTime ms
void moveAnkles(int angle, int moveTime);

//moves hips and knees to the specified angles and moves the ankles to a calculated angle to keep them pointing straight down
void moveAligned(int hipAngle, int kneeAngle);

//checks to see if a particular joint type is at a specified angle
bool indexIsAtAngle(int i, int angle);

//moves a particular leg to the specified angles over the course of moveTime ms.
void moveLeg(int i, int legAngles[], int moveTime);

//moves all joints to the specified angles over the course of moveTime ms.
void moveJoints(int moveAngles[4][3], int moveTime);

//moves the joints to a safe position
void safePosition();

//sets the internal calibration array
void setCalibration(int offsetArray[4][3]);

//runs a Serial-interactive loop to adjust the calibration array. Sets the zero-point of each motor. Hips should point at right-angles to the body, knees should point fully vertical, and ankles should be at right-angles to the legs
void calibrate();

//positions the leg in such a way that the tip of the leg occupies the coordinates (x,y,z) millimeters in space in reference to the leg joint
void positionFromCoordinates(int legNum, int x, int y, int z);

//returns all 12 leg servo angles in an array
AngleArray getCurrentPosition();

//time between motor moves
int delayTime;

//default time that a move takes
int defaultMoveTime;
```

Still in Beta, work in progress
