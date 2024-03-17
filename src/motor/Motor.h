//
// Created by REBAI Omar on 19/10/2023.
//

#ifndef VITESSE_MOTOR_H
#define VITESSE_MOTOR_H

#include "Arduino.h"
#include "simplePid/SimplePID.h"
class Motor {
private:
    uint8_t enca;
    uint8_t encb;
    uint8_t pwm;
    uint8_t in1;
    uint8_t in2;


public:
    Motor();
    SimplePID simplePid;
    void setMotorParams(uint8_t _pwm, uint8_t _in1, uint8_t _in2);
    void setEncoderParams(uint8_t _enca, uint8_t _encb, SimplePID _simplePid);
    void setup();

};



#endif //VITESSE_MOTOR_H
