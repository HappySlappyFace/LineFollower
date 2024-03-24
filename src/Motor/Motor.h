//
// Created by Ayman Rebai on 3/16/24.
//
#define PI 3.1415926535897932384626433832795
#ifndef UNTITLED_MOTOR_H
#define UNTITLED_MOTOR_H
#include "../KalmanFilter/Kalman.h"
//#include <PID_v1.h>
#include "../QuickPID/QuickPID.h"
class Motor {
public:
    Motor();
    Motor(int pinA, int pinB, int pinForward, int pinBackward);
    static void printCurrentRPM();
    float currentRPM = -1; // Public RPM variable
    void startPID();
    void SetPid(float Kp, float Ki, float Kd);
    long getTotalEncoderTicks() const;
    void followLine(float lineError, float baseSpeed);
    static const int MAX_MOTORS = 2;
    static Motor* instances[MAX_MOTORS];


    float pidInput;  // This will be your positionError
    float pidOutput=0; // This will be the control output from PID to your motor
    void setTargetRPM(float target);
    void calculateRPM();
    void applyControlOutput(float pidOut) const;
    void readEncoder();
    static void applyMotorsOutput(float left,float right);
    static void resetMotorsPID();



private:
    int encoderPinA, encoderPinB, pinForward, pinBackward;
    int pulsesPerRevolution=204;
    volatile long totalEncoderTicks=0;
    volatile bool lastEncoderAState;
    volatile bool lastEncoderBState;

    // PID Control variables
    float targetRPM; // Target RPM set by the user
    float outputRPM;
    QuickPID rpmPID; // PID controller object
    float positionError; // This should be updated in your readEncoder or a dedicated method


    float Kp=1;
    float Ki=0;
    float Kd=0;
    float wheelDiameterCm=6.5;
    float wheelCircumferenceCm=wheelDiameterCm*(float)PI;
    void resetPID();


    float targetSpeed;
    float calculateLineAdjustment(float lineError);
    long encoderTicks;
    float speedError;
    Kalman rpmFilter;


};

#endif //UNTITLED_MOTOR_H
