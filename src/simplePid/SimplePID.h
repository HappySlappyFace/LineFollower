#ifndef SIMPLEPID_H
#define SIMPLEPID_H

class SimplePID {
private:
    float kp, kd, ki, umax; // Parameters
    float eprev, eintegral; // Storage
    unsigned long prevT;
    int posPrev;
    volatile int pos_i;
    volatile float velocity_i;
    volatile unsigned long prevT_i;
    float v1Filt;
    float v1Prev;

public:
    SimplePID();
    void setParams(float kpIn, float kdIn, float kiIn, float umaxIn);
    void evalu(int value, int target, float deltaT, int &pwr, int &dir);
    void evaluSpeed(float target, float deltaT, int &pwr, int &dir);

    unsigned long getPrevT() const;

    void setPrevT(unsigned long prevousTime);

    int getPosPrev() const;

    void setPosPrev(int posPrev);

    volatile int getPosI() const;

    void setPosI(volatile int posI);

    volatile float getVelocityI() const;

    void setVelocityI(volatile float velocityI);

    volatile unsigned long getPrevTI() const;

    void setPrevTI(volatile unsigned long prevTI);
};

#endif
