//
// Created by hsf on 3/20/24.
//

#include "IRSensorArray.h"
#include <Arduino.h>


IRSensorArray::IRSensorArray(const uint16_t* values, const int* weights, int numSensors) :
        lineValues(values), sensorWeights(weights), numSensors(numSensors) {}

float IRSensorArray::calculateError() {
    long weightedSum = 0;
    long sum = 0;
    long minReading = 0;
    long maxReading = 1000;

    for (int i = 0; i < numSensors; ++i) {
        int sensorValue = lineValues[i];
//        Serial.print((String)sensorValue+"\t");
//        if (sensorValue < minReading) minReading = sensorValue;
//        if (sensorValue > maxReading) maxReading = sensorValue;
//        Serial.print((String)analogRead(sensorPins[i])+"\t");
        weightedSum += sensorValue * sensorWeights[i];
        sum += sensorValue;
    }
//    Serial.println();

//    Serial.print((String)weightedSum+"\t");
    if (sum == numSensors * minReading) return 0;

    float position = (float)weightedSum / (float)sum;
//    Serial.print((String)position+"\t");
//    float error = position - ((maxReading - minReading) / 2.0)
    float error = (float)weightedSum / (float)sum;
//    Serial.println((String)error+"\t");
    return error; //this should be error but i edited it
}

bool IRSensorArray::isAllWhite(){
    for(int i=0;i<numSensors;i++){
        if(lineValues[i]>250){
            return false;
        }
    }
    return true;
}
bool IRSensorArray::isAllBlack(){
    for(int i=0;i<numSensors;i++){
        if(lineValues[i]<250){
            return false;
        }
    }
    return true;
}
