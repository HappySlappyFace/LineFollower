//
// Created by Ayman Rebai on 3/16/24.
//

#include "Motor.h"
#include <Arduino.h>

static Kalman rpmFilter(25, 10, 3, 0);
Motor* Motor::instances[MAX_MOTORS] = {nullptr}; // Initialize all elements to nullptr


Motor::Motor(int pinA, int pinB,int pinForward, int pinBackward):
        pinForward(pinForward), pinBackward(pinBackward),
        rpmPID(&pidInput, &pidOutput, &pidSetpoint, Kp, Ki, Kd, QuickPID::Action::direct)
{
    encoderPinA = pinA;
    encoderPinB = pinB;
    pinMode(encoderPinA, INPUT_PULLUP);
    pinMode(encoderPinB, INPUT_PULLUP);
    pinMode(pinForward, OUTPUT);
    pinMode(pinBackward, OUTPUT);
    lastUpdateTime = millis();
    bool instanceAdded = false;
    for (int i = 0; i < MAX_MOTORS; ++i) {
        if (instances[i] == nullptr) {
            instances[i] = this;
            instanceAdded = true;
            break;
        }
    }
    if (!instanceAdded) {
        Serial.println("Error: Too many motor instances created!");
    }
    startPID(); // Start the PID controller
}

void Motor::printInstances() {
    for (int i = 0; i < MAX_MOTORS; ++i) {
        if(instances[i]==nullptr){Serial.println("null!");}
        if (instances[i] != nullptr) {
            Serial.print("Motor instance ");
            Serial.print(i);
            Serial.print(" has encoder pins A: ");
            Serial.print(instances[i]->encoderPinA);
            Serial.print(", B: ");
            Serial.println(instances[i]->encoderPinB);
        }
    }
}
void Motor::incrementEncoderTicks() {
    readEncoder();

//    if(this==instances[0]){
//        digitalWrite(2, !digitalRead(2));
//    }
//    if(this==instances[1]){
//        digitalWrite(0, !digitalRead(0));
//    }
}
void Motor::readEncoder() {

    bool currentA = digitalRead(encoderPinA);
    bool currentB = digitalRead(encoderPinB);
    debugTotalEncoderTicks = totalEncoderTicks;
    debugFlag = true;
    debugCurrentA=currentA;
    debugCurrentB=currentB;
//    Serial.println((String)currentA+"\t"+(String)currentB+"\t"+(String)totalEncoderTicks+"\t");
    if (lastEncoderAState != currentA) { // If A changed
        if (currentA == currentB) {
            totalEncoderTicks++;
        } else {
            totalEncoderTicks--;
        }
    } else if (lastEncoderBState != currentB) { // If B changed
        if (currentA != currentB) {
            totalEncoderTicks++;
        } else {
            totalEncoderTicks--;
        }
    }

    lastEncoderAState = currentA;
    lastEncoderBState = currentB;
}

void Motor::update() {
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - lastUpdateTime;
    if (elapsedTime >= 50) { //50 is the sample time
        noInterrupts();
        long currentTicks = totalEncoderTicks;
        interrupts();

        // Update PID input with the current position error
        pidInput = targetPosition - currentTicks; // positionError

        rpmPID.Compute();
        // Apply the control output to the motor
        applyControlOutput(pidOutput);

        lastUpdateTime = currentTime;
    }
}

void Motor::followLine(int lineError) {
    // Update PID input with the error from IR sensors
    pidInput = lineError;

    // Assuming the PID setpoint is already set to 0 during initialization
    rpmPID.Compute();

    // Apply the control output to correct the line following error
    applyControlOutput(pidOutput);
}
void Motor::applyControlOutput(float pidOut) const {
    // Assuming pidOut ranges from -255 to 255
    // Forward direction
    if (pidOut > 1) {
        analogWrite(pinForward, static_cast<int>(pidOut));
        digitalWrite(pinBackward, LOW);
    }
        // Backward direction
    else if (pidOut < -1) {
        analogWrite(pinBackward, -static_cast<int>(pidOut)); // Make pidOut positive
        digitalWrite(pinForward, LOW);
    }
        // Stop the motor if pidOut is 0
    else {
        digitalWrite(pinForward, LOW);
        digitalWrite(pinBackward, LOW);
    }
}

float Motor::calculateDistanceTraveled() const {
    const int ticksPerRevolution = pulsesPerRevolution/* number of encoder ticks per wheel revolution */;
    return ((float)totalEncoderTicks / (float)ticksPerRevolution) * wheelCircumferenceCm;
}
int Motor::distanceInCmToTicks(float distanceInCm) const{
    // Calculate the wheel's circumference based on its diameter
    return (int)((distanceInCm / wheelCircumferenceCm) * (float)pulsesPerRevolution);
}

void Motor::SetPid(float Kp, float Ki, float Kd){
    rpmPID.SetTunings(Kp,Ki,Kd);
}
void Motor::setTargetPosition(long positionTicks) {
    targetPosition = positionTicks;
}



void Motor::startPID() {
    rpmPID.SetMode(QuickPID::Control::automatic);
    rpmPID.SetSampleTimeUs(5000);
    rpmPID.SetOutputLimits(-255, 255); // Allow reverse control
    rpmPID.SetControllerDirection(QuickPID::Action::direct);
    rpmPID.Initialize();
}

long Motor::getTotalEncoderTicks() const {
    return totalEncoderTicks;
}
