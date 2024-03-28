#include <Arduino.h>
#include "Motor/Motor.h"
#include "IRSensorArray/IRSensorArray.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <QTRSensors.h>
Motor* leftMotor;
Motor* rightMotor;
//Motor Motors;
int sensorWeights[] = {-1, -2, -4, -3, 0, 3, 4, 2, 1};
//int pins[]={12,14,27,26,25,33,32,35,34};
uint8_t pins[]={34,35,32,33,25,26,27,14,12};
unsigned long lastTurnTriggerTimer=0;
bool stopped=false;
bool isTurning=false;
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

int smoothIsLeftTurn=0;

[[noreturn]] void lineFollowerTask(void *pvParameters) {
    const int baseSpeed = 120; // This should be adjusted based on your robot's design
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(3);
    for(;;) {

        uint16_t position = qtr.readLineBlack(sensorValues);
        long lineError=map(position,0,8000,-baseSpeed*0.85,baseSpeed*0.85);
        if(irSensorArray.isLeftTurn()){
            smoothIsLeftTurn++;
        }
        else{
            smoothIsLeftTurn=0;
        }
        if(smoothIsLeftTurn>999 || isTurning) {
            isTurning=true;
            digitalWrite(2, HIGH);
            if(abs(lineError)>10){
//                leftMotor->followLine(0, -80);
//                rightMotor->followLine(0, 80);
                Motor::enforceMotorsSetpoint(-60,60);
//                Motor::resetMotorsPID();
            }
            else{
                Motor::resetMotorsPID();
                digitalWrite(2, LOW);
                isTurning=false;
            }
        }else{
            digitalWrite(2,LOW);
            leftMotor->followLine(lineError, baseSpeed);
            rightMotor->followLine(lineError, -baseSpeed);
        }


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
            Serial.print("\t"+(String)sensorValue+"\t|");
        }
        Serial.println("\t\t"+(String)position);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

[[noreturn]] void readRPMTask(void *pvParameters){
    for(;;){
        leftMotor->calculateRPM();
        rightMotor->calculateRPM();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
[[noreturn]] void stopTask(void *pvParameters){
    for(;;){
        if(stopped)
        {
            if(irSensorArray.isAllBlack()){
                Motor::applyMotorsOutput(0,0);
                stopped=false;
                Motor::resetMotorsPID();
//                vTaskDelete(partie1);
                digitalWrite(2,HIGH);
                digitalWrite(rightMotorBack,0);
                digitalWrite(rightMotorFront,0);
                digitalWrite(leftMotorBack,0);
                digitalWrite(leftMotorFront,0);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
[[noreturn]] void randomFunctionalityTesting(void *pvParameters){
    for(;;){

//        digitalWrite(2,smoothIsLeftTurn>3?HIGH:LOW);

        vTaskDelay((pdMS_TO_TICKS(10)));
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
    qtr.setSensorPins(pins, SensorCount);
    pinMode(2, OUTPUT);
    pinMode(0, OUTPUT);
    Serial.begin(9600);


    //IR calibration
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


    //Initialization for motors and encoders
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
    //    xTaskCreate(startupTask, "Delay everything for 200ms", 2048, NULL, 9, NULL);
    for(int pin : pins){
        pinMode(pin,INPUT);
    }
    leftMotor->SetPid(5,30,0); //set k to 0.9
    rightMotor->SetPid(5,30,0);
    attachInterrupt(digitalPinToInterrupt(22), rightIncrementEncoderTicks, CHANGE);
    attachInterrupt(digitalPinToInterrupt(17), leftIncrementEncoderTicks, CHANGE);


    //FreeRTOS tasks
    xTaskCreate(lineFollowerTask, "LineFollowerTask", 4096, NULL, 4, &partie1);
    xTaskCreate(readRPMTask, "RPMReading", 2048, nullptr, 2, nullptr);
//    xTaskCreate(stopTask, "Stop", 2048, nullptr, 2, nullptr);
//    xTaskCreate(readIRTask, "ReadEncoders", 4096, NULL, 3, NULL);
//    xTaskCreate(randomFunctionalityTesting, "Functionality Testing", 2048, nullptr, 2, nullptr);

    stopped=true;

}

void loop() {

}