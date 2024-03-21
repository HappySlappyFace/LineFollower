//
// Created by hsf on 3/20/24.
//

#include "IRSensorArray.h"
#include <Arduino.h>


IRSensorArray::IRSensorArray(const int* pins, const int* weights, int numSensors) :
        sensorPins(pins), sensorWeights(weights), numSensors(numSensors) {}

float IRSensorArray::calculateError() {
    long weightedSum = 0;
    long sum = 0;
    long minReading = 20;
    long maxReading = 600;

    for (int i = 0; i < numSensors; ++i) {
        int sensorValue = analogRead(sensorPins[i]);
        if (sensorValue < minReading) minReading = sensorValue;
        if (sensorValue > maxReading) maxReading = sensorValue;
//        Serial.print((String)analogRead(sensorPins[i])+"\t");
        weightedSum += sensorValue * sensorWeights[i];
        sum += sensorValue;
    }

//    Serial.print((String)weightedSum+"\t");
    if (sum == numSensors * minReading) return 0;

    float position = (float)weightedSum / (float)sum;
//    Serial.print((String)position+"\t");
    float error = position - ((maxReading - minReading) / 2.0);
//    Serial.println((String)error+"\t");
    return position; //this should be error but i edited it
}


