//
// Created by hsf on 3/16/24.
//

#include "Kalman.h"
Kalman::Kalman(float process_noise, float measurement_noise, float estimated_error, float initial_value)
        : q(process_noise), r(measurement_noise), p(estimated_error), value(initial_value) {}

float Kalman::update(float measurement) {
    // Prediction update
    p += q;
    // Measurement update
    float k = p / (p + r); // Kalman gain
    value += k * (measurement - value);
    p *= (1 - k);
    return value;
}