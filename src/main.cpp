#include <Arduino.h>
#include "Motor/Motor.h"

void setup() {
    Serial.begin(9600);
    pinMode(22,INPUT);
    pinMode(2,OUTPUT);
    pinMode(33,OUTPUT);
    analogWrite(33,255);
    digitalWrite(2,HIGH);
    attachInterrupt(digitalPinToInterrupt(22),[]() { Motor::incrementEncoderTicksRight(); }, CHANGE);
}

Motor rightMotor(22, 110); // Example: encoderPinA = 22, PulsesPerRevolution = 110
void loop() {
    rightMotor.update();
    Serial.println(rightMotor.currentRPM);
    delay(25); // Adjust based on your application's needs
}
