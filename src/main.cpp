#include <Arduino.h>
#include "Motor/Motor.h"
#include "IRSensorArray/IRSensorArray.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <QTRSensors.h>
////#include "esp_wifi.h"
////#include <WiFi.h>
Motor* leftMotor;
Motor* rightMotor;
//
int sensorWeights[] = {-4, -3, -2, -1, 0, 1, 2, 3, 4};
//int sensorWeights[] = { -3, -2, -1, 0, 1, 2, 3};
////int sensorWeights[] = {  -2, 0, 2 };
int pins[]={12,14,27,26,25,33,32,35,34};
QTRSensors qtr;
const uint8_t SensorCount = sizeof(pins) / sizeof(pins[0]);
uint16_t sensorValues[SensorCount];

IRSensorArray irSensorArray(sensorValues, sensorWeights, SensorCount);
//IRSensorArray ir(sensorValues,sensorWeights,SensorCount);
//
//#include <esp32-hal-adc.h>

//
#define rightMotorBack 19
#define rightMotorFront 21
#define leftMotorFront 13
#define leftMotorBack 4
TaskHandle_t partie1;

void updateMotorTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(3);
    for (;;) {
        float lineError = irSensorArray.calculateError();
//        leftMotor->setTargetRPM(100);
//        rightMotor->setTargetRPM(100);
        leftMotor->followLine(lineError,100);
        rightMotor->followLine(lineError,100);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void partie1Task(void *pvParameters) {
    for(;;){
        int distanceToTravel=leftMotor->distanceInCmToTicks(50);
//        int distanceToTravel=-250;
        rightMotor->setTargetPosition(distanceToTravel);
        leftMotor->setTargetPosition(distanceToTravel);
        vTaskDelay(pdMS_TO_TICKS(10)); // Run every 1ms
    }
//    leftMotor->setTargetRPM(0);
//    vTaskDelete(partie1);
}

void lineFollowerTask(void *pvParameters) {
    const int baseSpeed = 100; // This should be adjusted based on your robot's design
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(2); // Adjust frequency as needed
    while (true) {
//        float lineError = irSensorArray.calculateError();
        uint16_t position = qtr.readLineBlack(sensorValues);
        float lineError=map(position,0,8000,-baseSpeed,baseSpeed);
        // Call followLine for each motor with the calculated line error
        rightMotor->followLine(-lineError, baseSpeed); // Assuming rightMotor needs to slow down when error is positive
        leftMotor->followLine(lineError, baseSpeed); // Assuming leftMotor needs to slow down when error is negative
//        Serial.println();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
//void readEncodersTask(void *pvParameters) {
//    for(;;){
//        Serial.print((String)leftMotor->getTotalEncoderTicks()+"\t"+ (String)leftMotor->calculateDistanceTraveled()+"\t"+(String)leftMotor->pidOutput+"\t");
//        Serial.println((String)rightMotor->getTotalEncoderTicks()+"\t"+ (String)rightMotor->calculateDistanceTraveled()+"\t"+(String)rightMotor->pidOutput);
//        vTaskDelay(pdMS_TO_TICKS(10)); // Run every 1ms
//    }
//}
//void debugTask(void *pvParameters) {
//    for(;;) {
//        if (leftMotor->debugFlag) {
//            leftMotor->debugFlag = false; // Reset flag
//            Serial.print("A: ");
//            Serial.print(leftMotor->debugEncoderTicks);
//            Serial.print("\tB: ");
//            Serial.print(leftMotor->debugTotalEncoderTicks);
//            Serial.println();
//        }
//        if (rightMotor->debugFlag) {
//            rightMotor->debugFlag = false; // Reset flag
//            Serial.print("A: ");
//            Serial.print(rightMotor->debugEncoderTicks);
//            Serial.print("\tB: ");
//            Serial.print(rightMotor->debugTotalEncoderTicks);
//            Serial.println();
//        }
//        vTaskDelay(pdMS_TO_TICKS(100)); // Run this task at a safe, low frequency
//    }
//}

void startupTask(void *pvParameters){
    vTaskDelay(pdMS_TO_TICKS(200));
}

void readIRTask(void *pvParameters){
    for(;;){
        uint16_t position = qtr.readLineBlack(sensorValues);
        for (uint8_t i = 0; i < SensorCount; i++)
        {
            Serial.print(sensorValues[i]);
            Serial.print('\t');
        }
        Serial.println(position);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
void readRPMTask(void *pvParameters){
    for(;;){
        rightMotor->calculateRPM();
        leftMotor->calculateRPM();
        leftMotor->update();
        rightMotor->update();
        Serial.println();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void setup() {
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){12,14,27,26,25,33,32,35,34}, SensorCount);
    pinMode(2, OUTPUT);
    pinMode(0, OUTPUT);
    digitalWrite(2, HIGH);
    for (uint16_t i = 0; i < 69; i++)
    {
        analogWrite(leftMotorBack,105);
        analogWrite(rightMotorFront,105);
        qtr.calibrate();
    }
    digitalWrite(leftMotorBack,LOW);
    digitalWrite(rightMotorFront,LOW);
    digitalWrite(2, LOW);
//    xTaskCreate(startupTask, "Delay everything for 200ms", 2048, NULL, 9, NULL);
    Serial.begin(9600);
//    esp_err_t results = esp_wifi_stop();
    pinMode(22,INPUT);
    pinMode(23,INPUT);
    pinMode(16,INPUT);
    pinMode(17,INPUT);
    pinMode(leftMotorFront,OUTPUT);
    pinMode(leftMotorBack,OUTPUT);
    pinMode(rightMotorBack,OUTPUT);
    pinMode(rightMotorFront,OUTPUT);
    digitalWrite(leftMotorFront,LOW);
    digitalWrite(leftMotorBack,LOW);
    digitalWrite(rightMotorBack,LOW);
    digitalWrite(rightMotorFront,LOW);
    leftMotor = new Motor (23,22,leftMotorFront,leftMotorBack);
    rightMotor = new Motor (17,16,rightMotorFront,rightMotorBack);
    for(int pin : pins){
//        digitalWrite(pin,LOW);
        pinMode(pin,INPUT);
    }
//    analogReadResolution(12);
//    analogSetWidth(12);
//    analogSetClockDiv(1);
//    analogSetAttenuation(ADC_11db);
//    analogSetPinAttenuation(14, ADC_2_5db);
    leftMotor->SetPid(.7,0,0); //set k to 0.9
    rightMotor->SetPid(.7,0,0);


    attachInterrupt(digitalPinToInterrupt(22), [](){ Motor::instances[0]->incrementEncoderTicks(); }, CHANGE);
    attachInterrupt(digitalPinToInterrupt(17), [](){ Motor::instances[1]->incrementEncoderTicks(); }, CHANGE);
//    xTaskCreate(updateMotorTask, "Update Motor", 4096, NULL, 3, NULL);
//    xTaskCreate(lineFollowerTask, "Partie1", 4096, NULL, 4, NULL);

    xTaskCreate(readRPMTask, "RPMReading", 2048, NULL, 2, NULL);
//    xTaskCreate(readIRTask, "ReadEncoders", 4096, NULL, 3, NULL);
//    xTaskCreate(readIRTask, "ReadEncoders", 4096, NULL, 3, NULL);
}

void loop() {

}