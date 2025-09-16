#pragma once
#include <Arduino.h>

#define STOP    0b0000
#define FORWARD 0b1010
#define REVERSE 0b0101
#define LEFT    0b1001
#define RIGHT   0b0110

class Bot {
private:
    uint8_t motorPins[4];  // Rf, Rb, Lf, Lb
    uint8_t _enR, _enL;
    uint8_t _chR, _chL;    // LEDC channels
    uint8_t _current_Speed;
    uint8_t _target_Speed;
    uint8_t _ramp_Step;
    uint16_t _ramp_Interval;
    unsigned long _last_UpdatedT;
    uint8_t _current_Pattern;

public:
    Bot(uint8_t Rf, uint8_t Rb, uint8_t Lf, uint8_t Lb,
        uint8_t enR, uint8_t enL, uint8_t chR = 0, uint8_t chL = 1);

    void init();
    void move(uint8_t pattern);
    void stop(uint8_t rampStep = 0);
    void forward(uint8_t accSpeed, uint8_t speed);
    void reverse(uint8_t accSpeed, uint8_t speed);
    void left(uint8_t accSpeed, uint8_t speed);
    void right(uint8_t accSpeed, uint8_t speed);
    void updateSpeed();
};
