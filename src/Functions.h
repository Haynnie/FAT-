#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <Evo.h>
EVOX1 evo;
EvoMotor leftMotor(M2, EV3MediumMotor, true); //change pins accordingly
EvoMotor rightMotor(M3, EV3MediumMotor);
// EvoMotor leftMotor(M1, EV3MediumMotor); //change pins accordingly
// EvoMotor rightMotor(M2, EV3MediumMotor, true);
EvoMotorPair robot(&leftMotor, &rightMotor);


void initEvo(){
    Serial.begin(115200);

    // Initialize EVO
    evo.begin();
    leftMotor.begin();
    rightMotor.begin();
    robot.coast();
    leftMotor.flipEncoderDirection(false);
    rightMotor.flipEncoderDirection(false);
    // leftMotor.flipEncoderDirection(true);
    // rightMotor.flipEncoderDirection(true);
    evo.flipDisplayOrientation(true);
}

void initDis(){
    evo.writeLineToDisplay("Program Start", 0, true, false);
    evo.writeToDisplay("Battery:", 0, 10);
    evo.writeToDisplay(evo.getBattery(), 50, 10, false, true);
    evo.writeToDisplay((String(leftMotor.getCount()) + ' ' + String(rightMotor.getCount())).c_str(), 0, 20, false, true);
    // evo.writeToDisplay((String(K[0]) + ' ' + String(K[1]) + ' ' + String(K[2])+ ' ' + String(K[3])).c_str(), 0, 40, false, true);
}

void resetMotors(){
    leftMotor.resetAngle();
    rightMotor.resetAngle();
    leftMotor.resetCount();
    rightMotor.resetCount();
}

#endif