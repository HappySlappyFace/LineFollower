//
// Created by Ayman Rebai on 3/16/24.
//

#include "Motor.h"
#include <Arduino.h>

static Kalman rpmFilter(25, 10, 3, 0);
Motor* Motor::instance = nullptr;
volatile long Motor::encoderTicks = 0;
volatile bool Motor::lastEncoderAState = LOW;
volatile bool Motor::lastEncoderBState = LOW;
bool Motor::direction= false;


Motor::Motor(int pinA, int pinB, int pulsesPerRev,int pinForward, int pinBackward):
        pinForward(pinForward), pinBackward(pinBackward),
        rpmPID(&currentRPM, &outputSignal, &targetRPM)
{
    Motor::encoderPinA = pinA;
    Motor::encoderPinB = pinB;
    pinMode(encoderPinA, INPUT_PULLUP);
    pinMode(encoderPinB, INPUT_PULLUP);
    pinMode(pinForward, OUTPUT);
    pinMode(pinBackward, OUTPUT);
    lastUpdateTime = millis();
    instance = this;
    startPID(); // Start the PID controller
}
void Motor::incrementEncoderTicks() {
    readEncoder();
}
void Motor::readEncoder() {
    unsigned long currentTime = millis();
    bool currentA = digitalRead(encoderPinA);
    bool currentB = digitalRead(encoderPinB);
    digitalWrite(2, currentA); // Example of direct state output for debugging
//    Serial.println(encoderTicks);
    // Determine direction and tick count
    if (lastEncoderAState != currentA) { // If A changed
        if (currentA == currentB) {
            encoderTicks++;
            totalEncoderTicks++;
        } else {
            encoderTicks--;
            totalEncoderTicks--;
        }
    } else if (lastEncoderBState != currentB) { // If B changed
        if (currentA != currentB) {
            encoderTicks++;
            totalEncoderTicks++;
        } else {
            encoderTicks--;
            totalEncoderTicks--;
        }
    }

    lastEncoderAState = currentA;
    lastEncoderBState = currentB;
}

void Motor::update() {
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - lastUpdateTime;
    if (elapsedTime >= sampleTimeMs) {
        noInterrupts();
        long ticks = encoderTicks; // Use instance-specific tick count
        encoderTicks = 0; // Reset tick count for the next period
        interrupts();
        // Calculate RPM. The direction can be factored into calculations if needed.
        float rpm = ((float)ticks / pulsesPerRevolution) * (60000.0 / (float)elapsedTime);
        currentRPM = rpmFilter.update(rpm); // Apply Kalman filter or any smoothing technique
        lastUpdateTime = currentTime;
    }
}
float Motor::calculateDistanceTraveled() const {
    const float wheelCircumferenceCm = 66;
    const int ticksPerRevolution = pulsesPerRevolution/* number of encoder ticks per wheel revolution */;
    return (totalEncoderTicks / (float)ticksPerRevolution) * wheelCircumferenceCm;
}

void Motor::SetPid(float Kp, float Ki, float Kd){
    rpmPID.SetTunings(Kp,Ki,Kd);
}
void Motor::Compute(){
    // Compute PID output
    rpmPID.Compute();
    if (outputSignal > 0) {
        analogWrite(pinForward, outputSignal);
        digitalWrite(pinBackward, LOW);
    } else {
        analogWrite(pinBackward, -outputSignal); // Assuming outputSignal can be negative
        digitalWrite(pinForward, LOW);
    }
//    Serial.println((String)currentRPM+" "+(String)outputSignal);
}

void Motor::setTargetRPM(float targetRpm) {
    this->targetRPM = targetRpm;
}

void Motor::startPID() {
    rpmPID.SetMode(QuickPID::Control::automatic);
    rpmPID.SetSampleTimeUs(5000);
    rpmPID.SetOutputLimits(-255, 255); // Allow reverse control
    rpmPID.SetControllerDirection(QuickPID::Action::direct);
    rpmPID.Initialize();
}
