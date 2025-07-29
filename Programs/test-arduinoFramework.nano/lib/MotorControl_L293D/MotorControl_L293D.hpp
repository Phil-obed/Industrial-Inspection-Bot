#pragma once    
#include <Arduino.h>

class Bot
{
private:
    uint8_t _motors = 2;
    uint8_t _speed;
    uint8_t _Rf;
    uint8_t _Rb;
    uint8_t _Lf;
    uint8_t _Lb;
    uint8_t _enR;
    uint8_t _enL;

    uint8_t motorPins[4];

    uint8_t _current_Speed;
    uint8_t _target_Speed;
    uint8_t _ramp_Step;
    uint8_t _ramp_Interval;
    unsigned long _last_UpdatedT;
    uint8_t _current_Pattern;

public:
    Bot(uint8_t Rf, uint8_t Rb, uint8_t Lf, uint8_t Lb, uint8_t enR, uint8_t enL);
    void init();
    void move(uint8_t pattern);
    void stop(uint8_t rampStep);
    void forward(uint8_t accSpeed, uint8_t speed);
    void reverse(uint8_t accSpeed, uint8_t speed);
    void right(uint8_t accSpeed, uint8_t speed);
    void left(uint8_t accSpeed, uint8_t speed);
    void updateSpeed();
};
