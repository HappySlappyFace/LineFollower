//
// Created by Ayman Rebai on 3/16/24.
//

#include "Motor.h"
#include <Arduino.h>

static Kalman rpmFilter(25, 10, 3, 0);
Motor* Motor::instance = nullptr;
volatile long Motor::rightEncoderTicks = 0;

Motor::Motor(int pinA, int pulsesPerRev): encoderPinA(pinA), pulsesPerRevolution(pulsesPerRev) {
    pinMode(encoderPinA, INPUT_PULLUP);
    lastUpdateTime = millis();
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
        currentRPM = rpmFilter.update(rpm); // Apply Kalman filter
        lastUpdateTime = currentTime;
    }
}
