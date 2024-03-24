//
// Created by hsf on 3/20/24.
//
#include <Arduino.h>
#ifndef UNTITLED_IRSENSORARRAY_H
#define UNTITLED_IRSENSORARRAY_H

class IRSensorArray {
public:
    IRSensorArray(const uint16_t *values, const int* weights, int numSensors);
    float calculateError();

private:
    const int* sensorWeights;
    int numSensors;
    const uint16_t *lineValues;
//    int values{};
};

#endif //UNTITLED_IRSENSORARRAY_H
