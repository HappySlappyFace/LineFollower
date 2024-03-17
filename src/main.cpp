#include <Arduino.h>
#include "simplePid/SimplePID.h"
#include "motor/Motor.h"
#include <QTRSensors.h>
#include <PIDController.h>

/*
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"
#include <NewPing.h>

#define LTRIG 12
#define LECHO 12
NewPing Lultrason(LTRIG, LECHO, 200);

#define RTRIG 13
#define RECHO 13
NewPing Rultrason(RTRIG, RECHO, 200);

uint8_t I2C_ADDRESS = 0x3C;
const int RST_PIN = -1;
SSD1306AsciiAvrI2c oled;
*/
QTRSensors qtr;
PIDController PIDController;
const uint8_t SensorCount = 3;
uint16_t sensorValues[SensorCount];

uint8_t partie = 0;
int seuil = 150;
long trst = 0;
float v = 180;
bool test = false;

//line follower
//TCRT5000
//const int Xright = A6;
//const int right = A3;
//const int mid = A2;
//const int left = A1;
//const int Xleft = ;


/*
void affiche(String s) {
    oled.clear();
    oled.set2X();
    oled.println(s);
}*/

// How many motors
#define NMOTORS 2
// Pins
// left[0] right[1]
int enca[] = {22, 23};
int encb[] = {22, 23};
int pwm[] = {13, 14};
int in1[] = {33, 32};
int in2[] = {26, 25};

unsigned long prevT = 0;
// PID class instances
SimplePID pid[NMOTORS];
Motor motor[NMOTORS];

void setMotor(int dir, int pwmVal, int speed, int sens1, int sens2) {
//    analogWrite(33, speed);
//    digitalWrite(32, LOW);
    if (dir == 1) {
        analogWrite(sens1, speed);
        digitalWrite(sens2, LOW);
    } else if (dir == -1) {
        digitalWrite(sens1, LOW);
        analogWrite(sens2, speed);
    } else {
        digitalWrite(sens1, LOW);
        digitalWrite(sens2, LOW);
    }
    delay(5);
}

void setMotorSpeed(float targetG, float targetD) {
    // time difference
    unsigned long currT = micros();
    float deltaT = ((float) (currT - prevT)) / (1.0e6f);
    prevT = currT;

    // set target position
    float target[NMOTORS];
    target[0] = targetG;
    target[1] = targetD;
    /*   target[0] = targetG/2.1122F;
       target[1] = targetD/2.1122F;*/
    // loop through the motors
    for (int k = 0; k < NMOTORS; k++) {
        int pwr, dir;
        // evaluate the control signal
        pid[k].evaluSpeed(target[k], deltaT, pwr, dir);
        // signal the motor
        setMotor(dir, pwr, pwm[k], in1[k], in2[k]);
        //Serial.print(String(pwr)+" ");
    }
}

template<int j>
void readEncoder() {
    //RISING ==> enca=1
    int a = digitalRead(enca[j]);
    int b = digitalRead(encb[j]);
    int increment;
    if (b > 0) {
        // If B is high, increment forward
        if (a > 0) {
            increment = 1;
        } else {
            increment = -1;
        }
    } else {
        // Otherwise, increment backward
        if (a > 0) {
            increment = -1;
        } else {
            increment = 1;
        }
    }

    // Get the current position of the motor
    int posi = pid[j].getPosI();

    // Set the new position of the motor
    pid[j].setPosI(posi + increment);
}

char color(int cap) {
    return (analogRead(cap) > seuil) ? 'b' : 'w';
}


void toggle(int led) {
    digitalWrite(led, !digitalRead(led));
}

void avancer(float vg, float vd) {
    setMotorSpeed(vg, vd);
}

void tourner(char d, float vg, float vd) {
    if (d == 'r') {
        setMotorSpeed(vg, -vd);
    } else if (d == 'l') {
        setMotorSpeed(-vg, vd);
    }
}

void stop() {
    while (true) {
        avancer(0, 0);
    }
}

//bool all_in(char s) {
//    return (color(Xleft) == s && color(left) == s && color(mid) == s && color(right) == s && color(Xright) == s);
//}

void erte7() {
    long erte7 = millis();
    while (millis() - erte7 < 5) {
        avancer(0, 0);
    }
}

void tnaffes(long time) {
    long ttx = millis();
    while (millis() - ttx < time) {
        avancer(0, 0);
    }
}

void PidBlack(float v) {
    uint16_t position = qtr.readLineBlack(sensorValues);
    PIDController.setpoint(1000);
    float motorSpeed = PIDController.compute(position);
    motorSpeed = map(motorSpeed, -80, 80, -v, v);
    avancer(v - motorSpeed, v + motorSpeed);
}

void PidWhite(float v) {
    uint16_t position = qtr.readLineWhite(sensorValues);
    PIDController.setpoint(1000);
    float motorSpeed = PIDController.compute(position);
    motorSpeed = map(motorSpeed, -80, 80, -v, v);
    avancer(v - motorSpeed, v + motorSpeed);
}

void Pidmoteurs(float kp, float kd, float ki) {
    for (int k = 0; k < NMOTORS; k++) {
        motor[k].setEncoderParams(enca[k], encb[k], pid[k]);
        motor[k].setMotorParams(pwm[k], in1[k], in2[k]);
        motor[k].setup();
        pid[k].setParams(kp, kd, ki, 255);
    }
}

void Pidcapteurs(float kp, float kd, float ki) {
    PIDController.tune(kp, kd, ki);
}
//
//void partie0(float v) {
//    while ((color(Xleft) == 'b') || (color(Xright) == 'b')) {
//        avancer(v, v);
//    }
//    partie = 1;
//}

// mateha stuff
/*
bool done = false;
int initPosI0 = 0;
int initPosI1 = 0;
float mou7it = 21.5;
int ticks = 204;

void tr90() {
    initPosI0 = pid[0].getPosI();
    initPosI1 = pid[1].getPosI();
    while (!done) {
        tourner('r', 80, 80);
        if (pid[0].getPosI() - initPosI0 >= (130) && pid[1].getPosI() - initPosI1 <= (-130)) {
            done = !done;
        }
    }
    done = false;
    erte7();
}

void tl90() {
    initPosI0 = pid[0].getPosI();
    initPosI1 = pid[1].getPosI();
    while (!done) {
        tourner('l', 80, 80);
        if (pid[1].getPosI() - initPosI1 >= (140) && pid[0].getPosI() - initPosI0 <= (-140)) {
            done = !done;
        }
    }
    done = false;
    long erte7 = millis();
    while (millis() - erte7 < 5) {
        avancer(0, 0);
    }
}

void HugLeft(float x, float v) {
    double distance = Lultrason.ping_cm();
    if (distance > 0 && distance < 199) {
        uint16_t position = 1000 * Lultrason.ping_cm();
        PIDController.setpoint(1000 * x);
        float motorSpeed = PIDController.compute(position);
        motorSpeed = map(motorSpeed, -80, 80, -v, v);
        avancer(v + motorSpeed, v - motorSpeed);
    } else {
        avancer(v, v);
    }
}

void HugRight(float x, float v) {
    double distance = Rultrason.ping_cm();
    if (distance > 0 && distance < 100) {
        uint16_t position = 1000 * Lultrason.ping_cm();
        PIDController.setpoint(1000 * x);
        float motorSpeed = PIDController.compute(position);
        motorSpeed = map(motorSpeed, -80, 80, -v, v);
        avancer(v - motorSpeed, v + motorSpeed);
    } else {
        avancer(v, v);
    }
}

void stepAvancer0(float cm, float v) {
    int t;
    t = cm * ticks / mou7it;
    //Serial.println(t);
    initPosI0 = pid[0].getPosI();
    initPosI1 = pid[1].getPosI();
    while (!done) {
        if (pid[0].getPosI() - initPosI0 >= (t) && pid[1].getPosI() - initPosI1 >= (t)) {
            done = !done;
        } else {
            avancer(v, v);
        }
    }
    done = false;
}

void stepAvancerv(float cm, float v) {
    int t;
    t = cm * ticks / mou7it;
    //Serial.println(t);
    initPosI0 = pid[0].getPosI();
    initPosI1 = pid[1].getPosI();
    while (!done) {
        if (pid[0].getPosI() - initPosI0 >= (t) && pid[1].getPosI() - initPosI1 >= (t)) {
            done = !done;
        } else {
            avancer(v, v);
        }
    }
    done = false;
}
*/

//void prioRight_Black(int v){
//    if (color(Xright) == 'b') {
//        tourner('r', v, v);
//    } else if (color(left) == 'b' || color(mid) == 'b' || color(right) == 'b') {
//        PidBlack(v);
//    } else if (color(Xleft) == 'b') {
//        tourner('l', v, v);
//    } else {
//        tourner('l',v,v);
//    }
//}
//
//void prioLeft_Black(int v){
//    if (color(Xleft) == 'b'){
//        tourner('l', v, v);
//    } else if (color(left) == 'b' || color(mid) == 'b' || color(right) == 'b') {
//        PidBlack(v);
//    } else if (color(Xright) == 'b') {
//        tourner('r', v, v);
//    } else {
//        tourner('r',v,v);
//    }
//}
//
//void prioRight_White(int v){
//    if (color(Xright) == 'w') {
//        tourner('r', v, v);
//    } else if (color(left) == 'w' || color(mid) == 'w' || color(right) == 'w') {
//        PidWhite(v);
//    } else if (color(Xleft) == 'w') {
//        tourner('l', v, v);
//    } else {
//        tourner('l',v,v);
//    }
//}
//
//void prioLeft_White(int v){
//    if (color(Xleft) == 'w'){
//        tourner('l', v, v);
//    } else if (color(left) == 'w' || color(mid) == 'w' || color(right) == 'w') {
//        PidWhite(v);
//    } else if (color(Xright) == 'w') {
//        tourner('r', v, v);
//    } else {
//        tourner('r',v,v);
//    }
//}

void setup() {
    /*oled.begin(&Adafruit128x64, I2C_ADDRESS);
    oled.setFont(Arial_bold_14);*/
    Serial.begin(9600);
    //pinMode(led,OUTPUT);
    //digitalWrite(led,HIGH);
    for (int k = 0; k < NMOTORS; k++) {
        motor[k].setEncoderParams(enca[k], encb[k], pid[k]);
        motor[k].setMotorParams(pwm[k], in1[k], in2[k]);
        motor[k].setup();
        pid[k].setParams(7, 0, 17, 255);
    }
    attachInterrupt(digitalPinToInterrupt(enca[0]), readEncoder<0>, CHANGE);
    attachInterrupt(digitalPinToInterrupt(enca[1]), readEncoder<1>, CHANGE);
    //Serial.println("target pos");
    while (test) {
        for (int i = 0; i < 600; ++i) {
            unsigned long currentTime = millis();
            while (millis() - currentTime < 2) {
                avancer(i, i);
            }
        }
        unsigned long currentTime = millis();
        while (millis() - currentTime < 5) {
            avancer(0, 0);
        }
    }
    // configure the sensors test
//    qtr.setTypeAnalog();
//    qtr.setSensorPins((const uint8_t[]) {left, mid, right}, SensorCount);
//    for (uint16_t i = 0; i < 150; i++) {
//        tourner('l', 70, 70);
//        qtr.calibrate();
//    }
    unsigned long currentTime = millis();

    PIDController.begin();
    PIDController.tune(1.2, 0.5, 0.5);
    PIDController.minimize(25);
    PIDController.limit(-255, 255);
    PIDController.setpoint(0);
    trst = millis();
    while(true){
        avancer(600, 600);
    }
}

void loop() {

    //Serial.println(Lultrason.ping_cm());
//    Serial.println((String)analogRead(Xleft)+'\t'+(String)analogRead(left)+'\t'+(String)analogRead(mid)+'\t'+(String)analogRead(right)+'\t'+(String)analogRead(Xright));
    //Serial.println((String)color(Xleft)+'\t'+(String)color(left)+'\t'+(String)color(mid)+'\t'+(String)color(right)+'\t'+(String)color(Xright));
}

