//
// Created by REBAI Omar on 18/10/2023.
//

#include "SimplePID.h"
#include <Arduino.h>
//#include <util/atomic.h>
SimplePID::SimplePID() : kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0) {
    prevT = 0;
    posPrev = 0;
    pos_i = 0;
    velocity_i = 0;
    prevT_i = 0;
    v1Filt = 0;
    v1Prev = 0;
}

void SimplePID::setParams(float kpIn, float kdIn, float kiIn, float umaxIn) {
    kp = kpIn;
    kd = kdIn;
    ki = kiIn;
    umax = umaxIn;
}

void SimplePID::evalu(int value, int target, float deltaT, int &pwr, int &dir) {
    int e = target - value;

    float dedt = (e - eprev) / deltaT;

    eintegral += e * deltaT;

    float u = kp * e + kd * dedt + ki * eintegral;

    pwr = (int)(fabs(u));
    if (pwr > umax) {
        pwr = (int)umax;
    }

    dir = 1;
    if (u < 0) {
        dir = -1;
    }

    eprev = e;
}

void SimplePID::evaluSpeed(float target, float deltaT, int &pwr, int &dir) {
    int pos = 0;
    noInterrupts();
    pos = pos_i;
    interrupts();


    unsigned long currT = micros();
    float deltaTime = (float)(currT - prevT) / 1.0e6F;
    float velocity1 = (pos - posPrev) / deltaTime;
    posPrev = pos;
    prevT = currT;

    // Convert count/s to RPM
    //float v1 = velocity1 / 102.0F * 60.0F;
    float v1 =  velocity1 / 204.0F * 60.0F;

    // Low-pass filter (25 Hz cutoff)
    v1Filt = 0.854F * v1Filt + 0.0728F * v1 + 0.0728F * v1Prev;
    v1Prev = v1;

    float e = target - v1Filt;

    // Derivative
    float dedt = (e - eprev) / deltaT;
    eintegral += e * deltaT;
    float u = kp * e + kd * dedt + ki * eintegral;

    // Set the motor speed and direction
    dir = 1;
    if (u < 0) {
        dir = -1;
    }
    pwr = (int)fabs(u);
    if (pwr > umax) {
        pwr = (int)umax;
    }

    // Store the previous error
    eprev = e;
}




volatile int SimplePID::getPosI() const {
    return pos_i;
}

void SimplePID::setPosI(volatile int posI) {
    pos_i = posI;
}




