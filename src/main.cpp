#include <Arduino.h>
#include "Motor/Motor.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

Motor rightMotor(22, 110); // Example: encoderPinA = 22, PulsesPerRevolution = 110


void updateMotorTask(void *pvParameters) {
    for (;;) {
        rightMotor.update();
        vTaskDelay(pdMS_TO_TICKS(25)); // Run every 25ms
    }
}

void serialPrintTask(void *pvParameters) {
    for (;;) {
        Serial.println(rightMotor.currentRPM);
        vTaskDelay(pdMS_TO_TICKS(50)); // Print every 1000ms (1 second)
    }
}
void updateLedTask(void *pvParameters) {
    for (;;) {
        digitalWrite(2, rightMotor.currentRPM<20?HIGH:LOW);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void setup() {
    Serial.begin(9600);
    pinMode(22,INPUT);
    pinMode(2,OUTPUT);
    pinMode(33,OUTPUT);
    analogWrite(33,255);
    digitalWrite(2,HIGH);
    attachInterrupt(digitalPinToInterrupt(22),[]() { Motor::incrementEncoderTicksRight(); }, CHANGE);

    // Create a task for updating the motor
    xTaskCreate(updateMotorTask, "Update Motor", 2048, NULL, 1, NULL);
    // Create a task for serial communication
    xTaskCreate(updateLedTask, "Update LED", 2048, NULL, 1, NULL);
}


void loop() {

}
