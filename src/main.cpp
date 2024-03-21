#include <Arduino.h>
#include "Motor/Motor.h"
#include "IRSensorArray/IRSensorArray.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
Motor* rightMotor;
Motor* leftMotor;

//int sensorWeights[] = {-4, -3, -2, -1, 0, 1, 2, 3, 4};
int sensorWeights[] = {  -2, 0, 2 };
//int pins[]={12,14,27,26,25,33,32,35,34};
int pins[]={26,25,32};//temporary middle pins until i fix the the IR sensors
IRSensorArray irSensorArray(pins, sensorWeights, sizeof(pins) / sizeof(pins[0]));


#define leftMotorBack 19
#define leftMotorFront 21
#define rightMotorFront 13
#define rightMotorBack 4
TaskHandle_t partie1;

void updateMotorTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1);
    for (;;) {
        float lineError = irSensorArray.calculateError();
        rightMotor->update();
        leftMotor->update();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void partie1Task(void *pvParameters) {
    for(;;){
//        int distanceToTravel=rightMotor->distanceInCmToTicks(25);
        int distanceToTravel=-250;
        leftMotor->setTargetPosition(distanceToTravel);
        rightMotor->setTargetPosition(distanceToTravel);
        vTaskDelay(pdMS_TO_TICKS(10)); // Run every 1ms
    }
//    rightMotor->setTargetRPM(0);
    vTaskDelete(partie1);
}

void lineFollowerTask(void *pvParameters) {
    for(;;){
//        int distanceToTravel=rightMotor->distanceInCmToTicks(25);
        float lineError = irSensorArray.calculateError();
        leftMotor->setTargetPosition(-300);
        rightMotor->setTargetPosition(-300);
        vTaskDelay(pdMS_TO_TICKS(1)); // Run every 1ms
    }
}
void readEncodersTask(void *pvParameters) {
    for(;;){
        Serial.print((String)rightMotor->getTotalEncoderTicks()+"\t"+ (String)rightMotor->calculateDistanceTraveled()+"\t"+(String)rightMotor->pidOutput+"\t");
        Serial.println((String)leftMotor->getTotalEncoderTicks()+"\t"+ (String)leftMotor->calculateDistanceTraveled()+"\t"+(String)leftMotor->pidOutput);
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
void startupTask(void *pvParameters){
    vTaskDelay(pdMS_TO_TICKS(200));
}

void readIRTask(void *pvParameters){
    for(;;){
        for(int pin : pins){
            Serial.print((String)analogRead(pin)+"\t");
        }
        Serial.println();
        vTaskDelay(pdMS_TO_TICKS(25));
    }
}
void setup() {
//    xTaskCreate(startupTask, "Delay everything for 200ms", 2048, NULL, 10, NULL);

    Serial.begin(9600);
    pinMode(22,INPUT);
    pinMode(23,INPUT);
    pinMode(16,INPUT);
    pinMode(17,INPUT);
    pinMode(rightMotorFront,OUTPUT);
    pinMode(rightMotorBack,OUTPUT);
    pinMode(leftMotorBack,OUTPUT);
    pinMode(leftMotorFront,OUTPUT);
    digitalWrite(rightMotorFront,LOW);
    digitalWrite(rightMotorBack,LOW);
    digitalWrite(leftMotorBack,LOW);
    digitalWrite(leftMotorFront,LOW);
    rightMotor = new Motor (23,22,rightMotorFront,rightMotorBack);
    leftMotor = new Motor (17,16,leftMotorFront,leftMotorBack);
    for(int pin : pins){
        digitalWrite(pin,LOW);
        pinMode(pin,INPUT);
    }
    rightMotor->SetPid(0.65,3.5,0); //set k to 0.9
    leftMotor->SetPid(0.65,3.5,0);


    attachInterrupt(digitalPinToInterrupt(22), [](){ Motor::instances[0]->incrementEncoderTicks(); }, CHANGE);
    attachInterrupt(digitalPinToInterrupt(17), [](){ Motor::instances[1]->incrementEncoderTicks(); }, CHANGE);
    xTaskCreate(updateMotorTask, "Update Motor", 4096, NULL, 3, NULL);
    xTaskCreate(lineFollowerTask, "Partie1", 2048, NULL, 1, &partie1);
//    xTaskCreate(readIRTask, "ReadEncoders", 2048, NULL, 1, NULL);
}

void loop() {

}
