#include <Arduino.h>

volatile long totalEncoderTicks = 0;
long lastEncoderTicks = 0;
unsigned long lastCalculationTime = 0;
const int encoderPinA = 22; // Phase A pin
const int pulsesPerRevolution = 110; // Adjust this based on your encoder



class KalmanFilter {
public:
    KalmanFilter(float process_noise, float measurement_noise, float estimated_error, float initial_value) :
            q(process_noise),
            r(measurement_noise),
            p(estimated_error),
            value(initial_value) {}

    float update(float measurement) {
        // Prediction update
        p = p + q;

        // Measurement update
        float k = p / (p + r); // Kalman gain
        value = value + k * (measurement - value);
        p = (1 - k) * p;

        return value;
    }

private:
    float q; // Process noise variance
    float r; // Measurement noise variance
    float p; // Estimated error in the estimate
    float value; // The value being tracked
};

KalmanFilter rpmFilter(20, 10, 3, 0); // Adjust these parameters as needed for your application

void IRAM_ATTR onEncoderTick() {
    totalEncoderTicks++;
}

void setup() {
    Serial.begin(9600);
    pinMode(encoderPinA, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encoderPinA), onEncoderTick, RISING);
    analogWrite(33,255); // Assuming this is related to motor control, adjust as necessary
    lastCalculationTime = millis();
}

const int movingAverageWindowSize = 5;
float rpmReadings[movingAverageWindowSize]; // Array to store recent RPM readings
int rpmReadingIndex = 0; // Index for inserting the next RPM reading
float totalRpm = 0; // Total of the last N readings

void loop() {
    unsigned long currentTime = millis();
    unsigned long timeDelta = currentTime - lastCalculationTime; // Time in milliseconds

    if (timeDelta >= 10) { // Calculate RPM every 10 ms (adjust as needed)
        noInterrupts();
        long currentTicks = totalEncoderTicks;
        interrupts();

        long tickDifference = currentTicks - lastEncoderTicks;
        float rpm = (tickDifference * 60000.0) / (pulsesPerRevolution * timeDelta);

        // Update the total by subtracting the oldest reading and adding the new one
        totalRpm -= rpmReadings[rpmReadingIndex]; // Subtract the value being overwritten
        rpmReadings[rpmReadingIndex] = rpm; // Insert the new reading
        totalRpm += rpm; // Add the new reading to the total

        rpmReadingIndex = (rpmReadingIndex + 1) % movingAverageWindowSize; // Move to the next index, wrap around if necessary

        // Calculate the moving average
        float averageRPM = totalRpm / movingAverageWindowSize;

        // Update Kalman filter with the average RPM measurement
        float filteredRPM = rpmFilter.update(averageRPM);

        Serial.print("Filtered RPM: ");
        Serial.println(filteredRPM);

        // Prepare for the next calculation
        lastCalculationTime = currentTime;
        lastEncoderTicks = currentTicks;
    }
}
