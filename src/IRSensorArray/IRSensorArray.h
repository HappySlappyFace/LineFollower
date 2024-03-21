//
// Created by hsf on 3/20/24.
//

#ifndef UNTITLED_IRSENSORARRAY_H
#define UNTITLED_IRSENSORARRAY_H
class IRSensorArray {
public:
    IRSensorArray(const int* pins, const int* weights, int numSensors);
    float calculateError();

private:
    const int* sensorPins;
    const int* sensorWeights;
    int numSensors;
};

#endif //UNTITLED_IRSENSORARRAY_H
