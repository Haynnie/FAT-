#include "Acceleration.h"

#define PROGRAM 1

#if PROGRAM == 1
extern EVOX1 evo;
extern EvoMotor leftMotor;
extern EvoMotor rightMotor;
extern EvoMotorPair robot;

float targetDistance = 3000.0f;

void setup(){
    initEvo();
    initDis();    
    evo.playTone(300, 100);
    evo.waitForBump(100); // Wait for start
    resetMotors();
}

sPD scurve(100000.0f, 10000000.0f);
void loop() {
    scurve.sMotion(targetDistance, sPD::HOLD);

    evo.writeLineToDisplay((String(leftMotor.getCount()) + ' ' + String(rightMotor.getCount())).c_str(), 0, true, true);
    delay(100000);
}


#elif PROGRAM == 2
#include <Arduino.h>
#include "LegoTech.h"

LegoLPF2 motor(Serial1);

void setup() {
    motor.begin();
    motor.setMode(LegoLPF2::MODE_POWER);
}

void loop() {
    motor.setPower(50);
    motor.poll();
    delay(100);
}

#endif
