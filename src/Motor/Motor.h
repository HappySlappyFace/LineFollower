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
    Motor(int pinA, int pinB, int pinForward, int pinBackward);
    void update();
    void incrementEncoderTicks();
    float currentRPM = -1; // Public RPM variable
    void startPID();
    void SetPid(float Kp, float Ki, float Kd);
    float calculateDistanceTraveled() const;
    void setTargetPosition(float positionTicks);
    long getTotalEncoderTicks() const;
    int distanceInCmToTicks(float distanceInCm) const;
    void followLine(float lineError, float baseSpeed);
    static void printInstances();
    static const int MAX_MOTORS = 2;
    static Motor* instances[MAX_MOTORS];

//    volatile bool debugFlag = false;
//    volatile long debugEncoderTicks = 0;
//    volatile long debugTotalEncoderTicks = 0;

    float pidInput;  // This will be your positionError
    float pidOutput=0; // This will be the control output from PID to your motor
    float pidSetpoint; // This is your target position
//    bool debugCurrentA;
//    bool debugCurrentB;
    void setTargetRPM(float target);
    void calculateRPM();

private:
    int encoderPinA, encoderPinB, pinForward, pinBackward;
    int pulsesPerRevolution=204;
    volatile long totalEncoderTicks=0;
    volatile bool lastEncoderAState;
    volatile bool lastEncoderBState;
    bool direction;

    void readEncoder();
    // PID Control variables
    float targetRPM{}; // Target RPM set by the user
    float outputSignal{}; // Output signal to control the motor (e.g., PWM value)
    QuickPID rpmPID; // PID controller object
    float positionError; // This should be updated in your readEncoder or a dedicated method
    float controlOutput; // PID output

    unsigned long lastUpdateTime;
    float targetPosition;
    float Kp=1;
    float Ki=0;
    float Kd=0;
    float wheelDiameterCm=6.5;
    float wheelCircumferenceCm=wheelDiameterCm*PI;

    void applyControlOutput(float pidOut) const;


    float targetSpeed;

    float calculateLineAdjustment(float lineError);



    long encoderTicks;
    float speedError;
    Kalman rpmFilter;
};

#endif //UNTITLED_MOTOR_H
