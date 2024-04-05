//
// Created by Ayman Rebai on 3/16/24.
//

#include "Motor.h"
#include <Arduino.h>


Motor* Motor::instances[MAX_MOTORS] = {nullptr}; // Initialize all elements to nullptr


Motor::Motor(int pinA, int pinB,int pinForward, int pinBackward):
        pinForward(pinForward), pinBackward(pinBackward),
        rpmPID(&pidInput, &pidOutput, &outputRPM, Kp, Ki, Kd, QuickPID::Action::direct),
        rpmFilter(25, 10, 3, 0)
{
    encoderPinA = pinA;
    encoderPinB = pinB;
    pinMode(encoderPinA, INPUT_PULLUP);
    pinMode(encoderPinB, INPUT_PULLUP);
    pinMode(pinForward, OUTPUT);
    pinMode(pinBackward, OUTPUT);
    bool instanceAdded = false;
    for (auto & instance : instances) {
        if (instance == nullptr) {
            instance = this;
            instanceAdded = true;
            break;
        }
    }
    if (!instanceAdded) {
        Serial.println("Error: Too many motor instances created!");
    }
    startPID(); // Start the PID controller
}

Motor::Motor() : rpmFilter(rpmFilter) {}

void Motor::readEncoder() {

    bool currentA = digitalRead(encoderPinA);
    bool currentB = digitalRead(encoderPinB);
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

void Motor::calculateRPM() {
        noInterrupts();
        long ticks = totalEncoderTicks;
        totalEncoderTicks = 0;
        interrupts();
        currentRPM = rpmFilter.update((float)ticks * 60.0/5);
}
void Motor::printCurrentRPM() {
    Serial.println((String)instances[0]->currentRPM+"\t"+(String)instances[1]->currentRPM);
}

void Motor::resetMotorsPID(){
    instances[0]->resetPID();
    instances[1]->resetPID();
}
void Motor::enforceSetpoint(float Set){
    outputRPM=Set;
}
void Motor::enforceMotorsSetpoint(float leftSet, float rightSet){
//    instances[0]->rpmPID.Reset();
//    instances[1]->rpmPID.Reset();
    instances[0]->enforceSetpoint(leftSet);
    instances[1]->enforceSetpoint(rightSet);
    instances[0]->rpmPID.Compute();
    instances[1]->rpmPID.Compute();
//    instances[0]->outputRPM=leftSet;
//    instances[1]->outputRPM=rightSet;
    instances[0]->applyControlOutput(instances[0]->outputRPM);
    instances[1]->applyControlOutput(instances[1]->outputRPM);
}
void Motor::resetPID(){
    rpmPID.Reset();
    rpmPID.SetMode(QuickPID::Control::manual);
    rpmPID.SetOutputLimits(-1, 1); // Allow reverse control

//    pidOutput=0;
//    pidInput=0;
//    totalEncoderTicks=0;
//    encoderTicks=0;
//    outputRPM=0;
    rpmPID.Compute();
}
void Motor::followLine(float lineError, float baseSpeed) {
    setTargetRPM(baseSpeed);
    pidInput = -currentRPM;
    outputRPM=targetRPM+lineError;
    rpmPID.Compute();


    if(this==instances[0]){
        Serial.print("R:\t"+(String)pidInput+"\t"+(String)pidOutput+"\t"+(String)outputRPM+"\t"+(String)currentRPM+"\t\t"+(String)lineError+"\t\t");
        applyControlOutput(outputRPM);
    }
    if(this==instances[1]){
        Serial.println("L:\t"+(String)pidInput+"\t"+(String)pidOutput+"\t"+(String)outputRPM+"\t"+(String)currentRPM);
        applyControlOutput(-outputRPM);
    }


    // Apply the control output to adjust motor speed

//    applyControlOutput(baseSpeed+lineAdjustment);
}
void Motor::applyMotorsOutput(float left,float right){
    instances[0]->applyControlOutput(left);
    instances[1]->applyControlOutput(right);
}
void Motor::applyControlOutput(float pidOut) const {
    // Assuming pidOut ranges from -255 to 255
    // Forward direction
    if (pidOut > 1) {
        analogWrite(pinForward, (int)pidOut);
        digitalWrite(pinBackward, LOW);
    }
        // Backward direction
    else if (pidOut < -1) {
        analogWrite(pinBackward, -(int)(pidOut)); // Make pidOut positive
        digitalWrite(pinForward, LOW);
    }
        // Stop the motor if pidOut is 0
    else {
        digitalWrite(pinForward, LOW);
        digitalWrite(pinBackward, LOW);
    }
}
void Motor::SetPid(float Kp, float Ki, float Kd){
    rpmPID.SetTunings(Kp,Ki,Kd);
}

void Motor::startPID() {
    rpmPID.SetMode(QuickPID::Control::automatic);
    rpmPID.SetSampleTimeUs(50000);
    rpmPID.SetOutputLimits(-255, 255); // Allow reverse control
    rpmPID.SetControllerDirection(QuickPID::Action::direct);
    rpmPID.Initialize();
}
long Motor::getTotalEncoderTicks() const {
    return totalEncoderTicks;
}
float Motor::calculateLineAdjustment(float lineError) {
    const float errorToSpeedScale = 1.0f;

    // Calculate the speed adjustment
    // The scaling factor determines the sensitivity of the adjustment to the line error
    float speedAdjustment = lineError * errorToSpeedScale;

    // Optionally, limit the speed adjustment to a maximum value to prevent too drastic changes
    float maxSpeedAdjustment = 50; // Example value; adjust based on your robot's capabilities
    speedAdjustment = constrain(speedAdjustment, -maxSpeedAdjustment, maxSpeedAdjustment);

    return speedAdjustment;
}

void Motor::setTargetRPM(float target) {
    targetRPM=target;
}


