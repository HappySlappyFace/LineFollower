#include <Arduino.h>
#include "Motor/Motor.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
Motor* rightMotor;
Motor* leftMotor;

void updateMotorTask(void *pvParameters) {
    for (;;) {
        rightMotor->update();
        rightMotor->Compute();
//        leftMotor->update();
//        leftMotor->Compute();
        vTaskDelay(pdMS_TO_TICKS(3)); // Run every 1ms
    }
}
int i=0;
int sense=0;
void readRpmTask(void *pvParameters) {
    for (;;) {

        vTaskDelay(pdMS_TO_TICKS(2)); // Run every 1ms
    }
}
void serialPrintTask(void *pvParameters) {
    for (;;) {
        Serial.println(rightMotor->currentRPM);
        vTaskDelay(pdMS_TO_TICKS(50)); // Print every 1000ms (1 second)
    }
}
void updateLedTask(void *pvParameters) {
    for (;;) {
        digitalWrite(2, rightMotor->currentRPM<20?HIGH:LOW);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
void partie1Task(void *pvParameters) {
    for (;;) {
        if(sense==0){
            i++;
            if(i>=600){sense=1;}
        }
        else{
            i--;
            if(i<=-600){sense=0;}
        }
        rightMotor->setTargetRPM(40);
//        leftMotor->update();
//        leftMotor->Compute();
        vTaskDelay(pdMS_TO_TICKS(8)); // Run every 1ms
    }
}
void setup() {
    rightMotor = new Motor (22,23, 204,33,32); // Example: encoderPinA = 22, PulsesPerRevolution = 110
//    leftMotor = new Motor (22,23, 110,26,25); // Example: encoderPinA = 22, PulsesPerRevolution = 110
    Serial.begin(9600);
    pinMode(22,INPUT);
    pinMode(23,INPUT);
    pinMode(2,OUTPUT);
    pinMode(33,OUTPUT);
    pinMode(32,OUTPUT);
    pinMode(26,OUTPUT);
    pinMode(25,OUTPUT);

    digitalWrite(2,HIGH);
    attachInterrupt(digitalPinToInterrupt(22), [](){ rightMotor->incrementEncoderTicks(); }, CHANGE);
    //attachInterrupt(digitalPinToInterrupt(34), [](){ leftMotor->incrementEncoderTicks(); }, CHANGE);

    rightMotor->SetPid(5,0,0);
    rightMotor->setTargetRPM(40);
//    leftMotor->SetPid(0.375,0.25,0.00001);

    xTaskCreate(updateMotorTask, "Update Motor", 2048, NULL, 3, NULL);
    //xTaskCreate(partie1Task, "Update Motor", 2048, NULL, 1, NULL);

}

void loop() {

}
