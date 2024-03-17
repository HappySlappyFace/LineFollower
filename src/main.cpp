#include <Arduino.h>
#include "Motor/Motor.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
Motor* rightMotor;
Motor* leftMotor;
TaskHandle_t partie1;
void updateMotorTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1);
    for (;;) {
        rightMotor->update();
        leftMotor->update();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void partie1Task(void *pvParameters) {
    for(;;){
        rightMotor->SetPid(0.5,4,0);
        leftMotor->SetPid(0.5,4,0);
        int distanceToTravel=rightMotor->distanceInCmToTicks(25);
        rightMotor->setTargetPosition(distanceToTravel);
        leftMotor->setTargetPosition(distanceToTravel);
        vTaskDelay(pdMS_TO_TICKS(10)); // Run every 1ms
    }
//    rightMotor->setTargetRPM(0);
    vTaskDelete(partie1);
}
void readEncodersTask(void *pvParameters) {
    for(;;){
        Serial.print((String)rightMotor->getTotalEncoderTicks()+"\t"+ (String)rightMotor->calculateDistanceTraveled()+"\t");
//        Serial.println((String)leftMotor->getTotalEncoderTicks()+"\t"+ (String)leftMotor->calculateDistanceTraveled());
        vTaskDelay(pdMS_TO_TICKS(10)); // Run every 1ms
    }
}

void debugTask(void *pvParameters) {
    for(;;) {
        if (rightMotor->debugFlag) {
            rightMotor->debugFlag = false; // Reset flag
            Serial.print("A: ");
            Serial.print(rightMotor->debugEncoderTicks);
            Serial.print("\tB: ");
            Serial.print(rightMotor->debugTotalEncoderTicks);
            Serial.println();
        }
        if (leftMotor->debugFlag) {
            leftMotor->debugFlag = false; // Reset flag
            Serial.print("A: ");
            Serial.print(leftMotor->debugEncoderTicks);
            Serial.print("\tB: ");
            Serial.print(leftMotor->debugTotalEncoderTicks);
            Serial.println();
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Run this task at a safe, low frequency
    }
}

void setup() {
    rightMotor = new Motor (22,23, 204,33,32); // Example: encoderPinA = 22, PulsesPerRevolution = 110
    leftMotor = new Motor (18,19, 204,25,26); // Example: encoderPinA = 22, PulsesPerRevolution = 110
    Serial.begin(9600);
    pinMode(22,INPUT);
    pinMode(23,INPUT);
    pinMode(18,INPUT);
    pinMode(19,INPUT);
    pinMode(33,OUTPUT);
    pinMode(32,OUTPUT);
    pinMode(26,OUTPUT);
    pinMode(25,OUTPUT);

    pinMode(2,OUTPUT);
    pinMode(0,OUTPUT);

    attachInterrupt(digitalPinToInterrupt(22), [](){ Motor::instances[0]->incrementEncoderTicks(); }, CHANGE);
    attachInterrupt(digitalPinToInterrupt(18), [](){ Motor::instances[1]->incrementEncoderTicks(); }, CHANGE);

    xTaskCreate(updateMotorTask, "Update Motor", 4096, NULL, 3, NULL);
    xTaskCreate(partie1Task, "Partie1", 2048, NULL, 1, &partie1);
//    xTaskCreate(debugTask, "ReadEncoders", 2048, NULL, 1, NULL);
}

void loop() {

}
