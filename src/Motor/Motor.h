//
// Created by Ayman Rebai on 3/16/24.
//

#ifndef UNTITLED_MOTOR_H
#define UNTITLED_MOTOR_H
#include "../KalmanFilter/Kalman.h"
//#include <PID_v1.h>
#include "../QuickPID/QuickPID.h"
class Motor {
public:
    Motor(int pinA, int pinB, int pulsesPerRev, int pinForward, int pinBackward);
    void update();
    void incrementEncoderTicks();

    float currentRPM = -1; // Public RPM variable
    void setTargetRPM(float targetRpm);
    void startPID();
    void Compute();

    void SetPid(float Kp, float Ki, float Kd);

private:
    int encoderPinA, encoderPinB;
    int pulsesPerRevolution=204;
    static volatile long encoderTicks;
    static volatile bool lastEncoderAState;
    static volatile bool lastEncoderBState;
    static bool direction;

    int pinForward;
    int pinBackward;

    void readEncoder();
    static const unsigned long sampleTimeMs = 50; // Sample time in milliseconds
    static Motor* instance;

    // PID Control variables
    float targetRPM{}; // Target RPM set by the user
    float outputSignal{}; // Output signal to control the motor (e.g., PWM value)
    QuickPID rpmPID; // PID controller object



    unsigned long lastUpdateTime;
};

#endif //UNTITLED_MOTOR_H
