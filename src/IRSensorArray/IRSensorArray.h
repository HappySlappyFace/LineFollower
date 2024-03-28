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
    bool isAllWhite();
    bool isAllBlack();
    bool isRightTurn();
    bool isLeftTurn();

private:
    const int* sensorWeights;
    int numSensors;
    const uint16_t *lineValues;
    int targetWhiteValue=250;
    int targetBlackValue=250;

};

#endif //UNTITLED_IRSENSORARRAY_H
