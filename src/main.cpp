#include <Arduino.h>
#include "Motor/Motor.h"
#include "IRSensorArray/IRSensorArray.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <QTRSensors.h>
Motor* leftMotor;
Motor* rightMotor;
//Motor Motors;
int sensorWeights[] = {-1, -2, -4, -3, 0, 4, 3, 2, 1};
int pins[]={12,14,27,26,25,33,32,35,34};
unsigned long lastTurnTriggerTimer=0;
QTRSensors qtr;
const uint8_t SensorCount = sizeof(pins) / sizeof(pins[0]);
uint16_t sensorValues[SensorCount];

IRSensorArray irSensorArray(sensorValues, sensorWeights, SensorCount);
//IRSensorArray ir(sensorValues,sensorWeights,SensorCount);
//
//#include <esp32-hal-adc.h>

//
#define leftMotorBack 19
#define leftMotorFront 21
#define rightMotorFront 13
#define rightMotorBack 4
TaskHandle_t partie1;

[[noreturn]] void updateMotorTask(void *pvParameters) {
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


[[noreturn]] void lineFollowerTask(void *pvParameters) {
    const int baseSpeed = 100; // This should be adjusted based on your robot's design
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(2);
    for(;;) {

        uint16_t position = qtr.readLineBlack(sensorValues);
        long lineError=map(position,0,8000,-baseSpeed*0.85,baseSpeed*0.85);
//        if(sensorValues[8]>400 &&sensorValues[7]>400){
//            long currentTime=millis();
////            if(currentTime-lastTurnTriggerTimer>1000){
//                while((millis()-currentTime<40)){
//                    digitalWrite(2,HIGH);
//                    Motor::applyMotorsOutput(-baseSpeed,baseSpeed);
//                }
////            }
//            lastTurnTriggerTimer=millis();
//            Motor::applyMotorsOutput(0,0);
//            Motor::resetMotorsPID();
//            digitalWrite(2,LOW);
//
//        }
        rightMotor->followLine(lineError, -baseSpeed); // Assuming rightMotor needs to slow down when error is positive
        leftMotor->followLine(lineError, baseSpeed); // Assuming leftMotor needs to slow down when error is negative

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}


void startupTask(void *pvParameters){
    vTaskDelay(pdMS_TO_TICKS(200));
}

[[noreturn]] void readIRTask(void *pvParameters){
    for(;;){
        uint16_t position = qtr.readLineBlack(sensorValues);
        for (unsigned short sensorValue : sensorValues)
        {
            Serial.print(sensorValue);
            Serial.print('\t');
        }
        Serial.println(position);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

[[noreturn]] void readRPMTask(void *pvParameters){
    for(;;){
        leftMotor->calculateRPM();

        rightMotor->calculateRPM();
//        leftMotor->update();
//        rightMotor->update();
//        Serial.println();
        vTaskDelay(pdMS_TO_TICKS(25));
    }
}
void rightIncrementEncoderTicks() {
    rightMotor->readEncoder();
}
void leftIncrementEncoderTicks() {
    leftMotor->readEncoder();
}

int debug=0;
void setup() {
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){12,14,27,26,25,33,32,35,34}, SensorCount);
    pinMode(2, OUTPUT);
    pinMode(0, OUTPUT);
    digitalWrite(2, HIGH);
    if (!debug){
        for (uint16_t i = 0; i < 69; i++)
        {
            analogWrite(leftMotorBack,105);
            analogWrite(rightMotorFront,105);
            qtr.calibrate();
        }
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
    leftMotor = new Motor (16,17,leftMotorFront,leftMotorBack);
    rightMotor = new Motor (22,23,rightMotorFront,rightMotorBack);
    for(int pin : pins){
        pinMode(pin,INPUT);
    }
    leftMotor->SetPid(5,5,0); //set k to 0.9
    rightMotor->SetPid(5,5,0);


    attachInterrupt(digitalPinToInterrupt(22), rightIncrementEncoderTicks, CHANGE);
    attachInterrupt(digitalPinToInterrupt(17), leftIncrementEncoderTicks, CHANGE);
//    xTaskCreate(updateMotorTask, "Update Motor", 4096
    xTaskCreate(lineFollowerTask, "LineFollowerTask", 4096, nullptr, 4, nullptr);
    xTaskCreate(readRPMTask, "RPMReading", 2048, nullptr, 2, nullptr);
//    xTaskCreate(readIRTask, "ReadEncoders", 4096, NULL, 3, NULL);
//    xTaskCreate(readIRTask, "ReadEncoders", 4096, NULL, 3, NULL);
}

void loop() {

}