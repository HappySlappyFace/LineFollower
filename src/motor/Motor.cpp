//
// Created by REBAI Omar on 19/10/2023.
//

#include "Motor.h"
Motor::Motor() {}

void Motor::setMotorParams(uint8_t _pwm, uint8_t _in1, uint8_t _in2){
    pwm=_pwm;
    in1=_in1;
    in2=_in2;
}

void Motor::setEncoderParams(uint8_t _enca, uint8_t _encb, SimplePID _simplePid){
    enca=_enca;
    encb=_encb;
    simplePid=_simplePid;
}

void Motor::setup(){
    pinMode(enca, INPUT);
    pinMode(encb, INPUT);
    pinMode(pwm, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
}
