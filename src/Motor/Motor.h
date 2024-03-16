//
// Created by Ayman Rebai on 3/16/24.
//

#ifndef UNTITLED_MOTOR_H
#define UNTITLED_MOTOR_H
#include "../KalmanFilter/Kalman.h"
#include <PID_v1.h>

class Motor {
public:
    Motor(int pinA, int pulsesPerRev, int pinForward, int pinBackward);
    void update();
    void static incrementEncoderTicksRight();
    double currentRPM = -1; // Public RPM variable
    void setTargetRPM(double targetRpm);
    void startPID();
    double getCurrentRPM() const { return currentRPM; }

private:
    static volatile long rightEncoderTicks;
    unsigned long lastUpdateTime;
    int encoderPinA;
    int pinForward;
    int pinBackward;
    int pulsesPerRevolution;
    static const unsigned long sampleTimeMs = 50; // Sample time in milliseconds
    static Motor* instance;

    // PID Control variables
    double targetRPM; // Target RPM set by the user
    double outputSignal; // Output signal to control the motor (e.g., PWM value)
    PID rpmPID; // PID controller object

};

#endif //UNTITLED_MOTOR_H
