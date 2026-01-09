#ifndef ACCELERATION_H
#define ACCELERATION_H

#include <math.h>
#include <algorithm> // For std::abs, std::min, std::max
#include "Functions.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
// #include <../lib/Teleplot.h>
// Teleplot teleplot("127.0.0.1");

#ifndef constrain
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#endif




extern EVOX1 evo;
extern EvoMotor leftMotor;
extern EvoMotor rightMotor;
extern EvoMotorPair robot;


class test {
    private:  
        float minVel = 2000.0f;
        float maxVel = 4000.0;
        float jerkPerA = 0.4f;   
        float jerkPerD = 0.4f;
        float ki = 0.0f;
        

    public:        
        float vel = minVel, maxAcc, Acc = 0.0f, maxJerk, JerkAccDist, JerkDcDist;
        float curDist, totalDist, start_cruiseDist, cruiseDist, slow_cruiseDist, AccDist, DcDist = 0.0f;
                
        
        float error, lastError;
        float lDeg, rDeg;
         
        int lspd, rspd;
        int dir, adj;

    xTaskHandle mot = NULL;

    test(float mxA = 8000.0f, float mxJ = 500000.0f){
        maxAcc = mxA;
        maxJerk = mxJ;
    };


    enum stopMode{
        COAST,
        BRAKE,
        HOLD
    };


    int param(float totalDistance){
        int motion;
        totalDist = totalDistance;
        if (totalDist < 600){
            AccDist = 0; DcDist = 0;
            cruiseDist = 0; slow_cruiseDist = 0; 
            JerkAccDist = 0; JerkDcDist = 0;
            motion = 0;
        }
        else if (totalDist< 1800){
            AccDist = 0.2 * totalDist;
            DcDist = 0.3 * totalDist;
            cruiseDist = 0.1 * totalDist; 

            start_cruiseDist = 0.5*(totalDist - (AccDist + DcDist+cruiseDist));
            slow_cruiseDist = start_cruiseDist;
            JerkAccDist = (jerkPerA)*AccDist;
            JerkDcDist = (jerkPerD)*DcDist;            
            motion = 1;         
        }
        else{
            AccDist = 0.5 * totalDist;
            DcDist = 0.3 * totalDist;
            cruiseDist = 0.2 * totalDist; 

            start_cruiseDist = 0.5*(totalDist - (AccDist + DcDist+cruiseDist));
            slow_cruiseDist = start_cruiseDist;
            JerkAccDist = (jerkPerA)*AccDist;
            JerkDcDist = (jerkPerD)*DcDist;            
            motion = 1;
        }
        return motion;
    };

    void sCalculate(void* spd_){
        test* instance = static_cast<test*>(spd_);

        for (;;){
            TickType_t lwt = xTaskGetTickCount();
            const TickType_t freq = pdMS_TO_TICKS(10);
            const float dt = 0.010;
        }

    };

    void sMotion(float totalDistance, stopMode _stop, float kp = 1.5, float kd = 10.0, int speed = 1500, float aj = 200000.0f, float bj = 10000.0f, float cj = 100000.0f, float dj = 5000.0f){
        int type = param(totalDistance);

        float curDist = 0.0f;
        unsigned long currentTime;
        float dt = 0.0000f;
        vel = minVel;
        dir = (leftMotor.getCount()>=0)?1:-1;
        unsigned long lastTime = millis();


        while (curDist <= (totalDist - 10.0f)){
            currentTime = millis();
            dt = (currentTime - lastTime)/1000.0f;
            lastTime = currentTime;

            lDeg = fabs(leftMotor.getCount());
            rDeg = fabs(rightMotor.getCount());
            curDist = (lDeg + rDeg)/2.0f;
            error = rDeg - lDeg;

            
            // --- NO ACCEL ---
            if(type == 0){
                vel = speed;
            }
            else if(curDist < start_cruiseDist){
                Acc = 0;
            }
            // --- ACCELERATION PHASE ---
           else if (curDist < start_cruiseDist + JerkAccDist){
                Acc = constrain(Acc+(aj*dt), 0, maxAcc);
            }
            else if(curDist < (start_cruiseDist + AccDist - JerkAccDist)){
            }
            else if(curDist < start_cruiseDist + AccDist){
                Acc = constrain(Acc-(bj*dt), 0, maxAcc);               
            }


            // --- FAST CRUISE PHASE ---
            else if(curDist < (start_cruiseDist + cruiseDist + AccDist)){
                Acc = 0.0f;
            }


            // --- DECELERATION PHASE ---
            else if(curDist < (totalDist - slow_cruiseDist - 0.6 * DcDist)){
                Acc = constrain(Acc-(cj*dt), -maxAcc, 0);
            }
            else if(curDist < (totalDist - JerkDcDist - slow_cruiseDist)){
            }
            else if(curDist < (totalDist - cruiseDist*0.3f)){
                Acc = constrain(Acc+(dj*dt), -maxAcc, 0);
        
            }

            vel = constrain(vel+(Acc*dt), minVel, maxVel);
            adj = (kp * 10000 * error *dt) + ((kd*100)*(error - lastError)/dt) + (ki * (error - lastError) * dt * 0.66);
            lspd = constrain(vel + adj, minVel, maxVel);
            rspd = constrain(vel - adj, minVel, maxVel);
            leftMotor.run(dir*lspd);
            rightMotor.run(dir*rspd);
            lastError = error;
            Serial.println(String(error) + "  " + String(vel) + "  " + String(Acc) + "  " + String(lDeg) + "  " + String(rDeg));
        }

        stopMotors(_stop);
    };

    void stopMotors(stopMode mode) {
        if (mode == COAST) {
            leftMotor.run(0); 
            rightMotor.run(0);
        } 
        else if (mode == BRAKE) {
            leftMotor.brake();
            rightMotor.brake();
        }
        else if (mode == HOLD) {
            leftMotor.hold();
            rightMotor.hold();
        }
    };
};


class sPD {
    private:  
        float minVel = 2500.0f;
        float maxVel = 4000.0;
        float jerkPerA = 0.5f;   
        float jerkPerD = 0.4f;
        
        std::atomic<float> vel{0.0f};
        std::atomic<float> curDist{0.0f};
        

    public:        
        float maxAcc, Acc = 0.0f, maxJerk, JerkAccDist, JerkDcDist;
        float totalDist, start_cruiseDist, cruiseDist, slow_cruiseDist, AccDist = 0.0f, DcDist = 0.0f;
                
        float aj, bj, cj, dj;
        float error, lastError;
        float lDeg, rDeg;
        float kp, kd, ki = 0.0f;     
        int lspd, rspd;
        int dir, adj, type;
        float dt = 0.001;

    xTaskHandle mot = NULL;

    sPD(float mxA = 8000.0f, float mxJ = 500000.0f){
        maxAcc = mxA;
        maxJerk = mxJ;
    };

    enum stopMode{
        COAST,
        BRAKE,
        HOLD
    };

int param(float totalDistance) {
        totalDist = totalDistance;
        if (totalDist < 1000) {
            AccDist = DcDist = cruiseDist = slow_cruiseDist = JerkAccDist = JerkDcDist = 0;
            return 0;
        }
        
        float accRatio = (totalDist < 1800) ? 0.3f : 0.5f;
        AccDist = accRatio * totalDist;
        DcDist = 0.3f * totalDist;
        cruiseDist = (totalDist < 1800) ? 0.1f : 0.2f;

        start_cruiseDist = 0.3f * (totalDist - (AccDist + DcDist + cruiseDist));
        slow_cruiseDist = 0.7f * (totalDist - (AccDist + DcDist + cruiseDist));
        JerkAccDist = jerkPerA * AccDist;
        JerkDcDist = jerkPerD * DcDist;            
        return 1;
    }

    void sMotion(float totalDistance, stopMode _stop, float kp = 5.0/*30*/, float kd = 20/*10*/, int baseSpd = 3000, float aj_ = 4.0f, float bj_ = 3.0f, float cj_ = 1.0f, float dj_ = 3.0f) {
        type = param(totalDistance);
        if (totalDist<1000) type = 0;
        curDist = 0.0f; vel = minVel; lastError = 0;
        aj = aj_; bj = bj_; cj = cj_; dj = dj_;
        dir = (leftMotor.getCount() >= 0) ? 1 : -1;
        
        uint64_t lastTime = esp_timer_get_time();

        while (curDist < (totalDist - 5.0f)) { 
            uint64_t currentTime = esp_timer_get_time();
            dt = (currentTime - lastTime) / 1000000.0f;
            lDeg = fabs(leftMotor.getCount());
            rDeg = fabs(rightMotor.getCount());
            curDist = (lDeg + rDeg) / 2.0f;
            error = rDeg - lDeg;

            if (type == 0) vel = baseSpd;
            else if (curDist < start_cruiseDist) Acc = 0;
            else if (curDist < start_cruiseDist + JerkAccDist) 
                Acc = constrain(Acc + (aj * dt), 0, maxAcc);
            else if (curDist < start_cruiseDist + AccDist - JerkAccDist) { /* Constant Acc */ }
            else if (curDist < start_cruiseDist + AccDist) 
                Acc = constrain(Acc - (bj * dt), 0, maxAcc);
            else if (curDist < start_cruiseDist + cruiseDist + AccDist) Acc = 0.0f;
            else if (curDist < totalDist - slow_cruiseDist - 0.6f * DcDist) 
                Acc = constrain(Acc - (cj * dt), -maxAcc, 0);
            else if (curDist < totalDist - JerkDcDist - slow_cruiseDist) { kp = 1; kd = 1; }
            else if (curDist < totalDist - cruiseDist * 0.3f) 
                Acc = constrain(Acc + (dj * dt), -maxAcc, 0); 
                       
            vel = constrain(vel + (Acc * dt), 1500, maxVel);
            float adj = (kp * 100 * error * dt) + ((kd * 1) * (error - lastError) / dt);
            
            // leftMotor.move(dir * constrain(vel + adj, 0, maxVel));
            // rightMotor.move(dir * constrain(vel - adj, 0, maxVel));
            leftMotor.move(dir * constrain(vel - adj, 0, maxVel));
            rightMotor.move(dir * constrain(vel + adj, 0, maxVel));
            
            Serial.println(String(error) + " " + String(vel));

            lastError = error;

        }
        stopMotors(_stop);
    }

    void stopMotors(stopMode mode) {
        if (mode == COAST) {
            leftMotor.run(0); 
            rightMotor.run(0);
        } 
        else if (mode == BRAKE) {
            leftMotor.brake();
            rightMotor.brake();
        }
        else if (mode == HOLD) {
            leftMotor.hold();
            rightMotor.hold();
            // delay(500);
            // leftMotor.brake(), rightMotor.brake();
            
            // // leftMotor.run(0); 
            // // rightMotor.run(0);
        }
    };
};


#endif