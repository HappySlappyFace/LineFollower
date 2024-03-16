//
// Created by hsf on 3/16/24.
//

#ifndef UNTITLED_KALMAN_H
#define UNTITLED_KALMAN_H


class Kalman {
public:
    Kalman(float process_noise, float measurement_noise, float estimated_error, float initial_value);
    float update(float measurement);
private:
    float q; // Process noise variance
    float r; // Measurement noise variance
    float p; // Estimated error in the estimate
    float value; // The value being tracked
};


#endif //UNTITLED_KALMAN_H
