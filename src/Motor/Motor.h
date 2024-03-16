//
// Created by Ayman Rebai on 3/16/24.
//

#ifndef UNTITLED_MOTOR_H
#define UNTITLED_MOTOR_H
#include "../KalmanFilter/Kalman.h"

class Motor {
public:
    Motor(int pinA, int pulsesPerRev);
    void update();
    void static incrementEncoderTicksRight();
    float currentRPM = -1; // Public RPM variable

private:
    static volatile long rightEncoderTicks;
    unsigned long lastUpdateTime;
    int encoderPinA;
    int pulsesPerRevolution;
    static const unsigned long sampleTimeMs = 50; // Sample time in milliseconds
    static Motor* instance;
};

#endif //UNTITLED_MOTOR_H
