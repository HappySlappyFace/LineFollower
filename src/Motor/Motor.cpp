//
// Created by Ayman Rebai on 3/16/24.
//

#include "Motor.h"
#include <Arduino.h>

static Kalman rpmFilter(25, 10, 3, 0);
Motor* Motor::instance = nullptr;
volatile long Motor::rightEncoderTicks = 0;

Motor::Motor(int pinA, int pulsesPerRev,int pinForward, int pinBackward):
        encoderPinA(pinA), pulsesPerRevolution(pulsesPerRev),
        pinForward(pinForward), pinBackward(pinBackward),
        rpmPID(&currentRPM, &outputSignal, &targetRPM, 2.0, 5.0, 1.0, DIRECT)
{
    pinMode(encoderPinA, INPUT_PULLUP);
    pinMode(pinForward, OUTPUT);
    pinMode(pinBackward, OUTPUT);
    lastUpdateTime = millis();
    instance = this;
    startPID(); // Start the PID controller
    rpmPID.SetTunings(3.6,0,0);
}
void Motor::incrementEncoderTicksRight() {
//    digitalWrite(2, !digitalRead(2));
    Motor::rightEncoderTicks++;
}
void Motor::update() {
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - lastUpdateTime;
    if (elapsedTime >= sampleTimeMs) {
        noInterrupts();
        long ticks = Motor::rightEncoderTicks;
        Motor::rightEncoderTicks = 0; // Reset tick count for the next period
        interrupts();
        float rpm = ((float)ticks / (float)pulsesPerRevolution) * (60000.0 / (float)elapsedTime);
        currentRPM = rpmFilter.update(rpm); // Apply Kalman filter.

        rpmPID.Compute(); // Compute PID output
        if (outputSignal > 0) {
            analogWrite(pinForward, outputSignal);
            digitalWrite(pinBackward, LOW);
        } else {
            analogWrite(pinBackward, -outputSignal); // Assuming outputSignal can be negative
            digitalWrite(pinForward, LOW);
        }
        Serial.println((String)rpm+" "+(String)outputSignal);
//        Serial.println(outputSignal);

        lastUpdateTime = currentTime;
    }
}

void Motor::setTargetRPM(double targetRpm) {
    this->targetRPM = targetRpm;
}

void Motor::startPID() {
    rpmPID.SetMode(AUTOMATIC);
    rpmPID.SetSampleTime(10);
    rpmPID.SetOutputLimits(-255, 255); // Allow reverse control
}
