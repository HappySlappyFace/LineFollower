//
// Created by Ayman Rebai on 3/16/24.
//

#include "Motor.h"
#include <Arduino.h>

static Kalman rpmFilter(25, 10, 3, 0);
Motor* Motor::instances[MAX_MOTORS] = {nullptr}; // Initialize all elements to nullptr


Motor::Motor(int pinA, int pinB,int pinForward, int pinBackward):
        pinForward(pinForward), pinBackward(pinBackward),
        rpmPID(&currentRPM, &targetRPM, &pidOutput, Kp, Ki, Kd, QuickPID::Action::direct)
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
        noInterrupts();
        long currentTicks = totalEncoderTicks;
        interrupts();

        // Update PID input with the current position error
        pidInput = (targetPosition - currentTicks); // positionError
//        Serial.println(pidInput);
        calculateRPM();
        rpmPID.Compute();
        // Apply the control output to the motor
        applyControlOutput(pidOutput);
}
void Motor::calculateRPM() {
    static unsigned long lastUpdateTime = 0;
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - lastUpdateTime;

    if (elapsedTime >= 10) { // Calculate RPM every second
        noInterrupts();
        long ticks = encoderTicks;
        encoderTicks = totalEncoderTicks; // Reset tick count for the next measurement period
        interrupts();

        currentRPM = (ticks / (float)pulsesPerRevolution) * 60.0 / (elapsedTime / 1000.0);
        lastUpdateTime = currentTime;
    }
}
void Motor::followLine(float lineError, float baseSpeed) {
//    pidInput = lineError;
//    setTargetPosition()

    // Then, use the encoder feedback to adjust towards this new target speed
    noInterrupts();
    long currentTicks = totalEncoderTicks;
    interrupts();

    float speedError = baseSpeed - (float)currentTicks;
    float lineAdjustment = calculateLineAdjustment(lineError); // Define this function based on lineError

    pidInput = speedError + lineAdjustment;

    rpmPID.Compute(); // Adjust the PID controller to regulate speed instead of position
    Serial.println((String)lineAdjustment+"\t"+(String)targetSpeed+"\t"+(String)pidOutput);

    // Apply the control output to adjust motor speed
    applyControlOutput(pidOutput);
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
void Motor::setTargetPosition(float positionTicks) {
    targetPosition = positionTicks;
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

void Motor::setTargetRPM(float targetRPM) {
    this->targetRPM=targetRPM;
}
