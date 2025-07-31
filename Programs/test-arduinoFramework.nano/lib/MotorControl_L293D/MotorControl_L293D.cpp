#include <Arduino.h>
#include "MotorControl_L293D.hpp"


#define STOP    0b0000
#define FORWARD 0b1010
#define REVERSE 0b0101
#define LEFT    0b1001
#define RIGHT   0b0110

Bot::Bot(uint8_t Rf, uint8_t Rb, uint8_t Lf, uint8_t Lb, uint8_t enR, uint8_t enL)
{
    motorPins[0] = Rf;
    motorPins[1] = Rb;
    motorPins[2] = Lf;
    motorPins[3] = Lb;

    _enR = enR;
    _enL = enL;

    _current_Speed = 0;
    _target_Speed = 0;
    _ramp_Step = 0;
    _ramp_Interval = 10;
    _last_UpdatedT = 0;
    _current_Pattern = STOP;

    

    stop(0);
}

void Bot::init(){
    pinMode(_enR, OUTPUT);
    pinMode(_enL, OUTPUT);

    for (uint8_t i = 0; i < 4; i++)
    {
        pinMode(motorPins[i], OUTPUT);
    }
}

void Bot::move(uint8_t pattern)
{
    _current_Pattern = pattern;

    for (uint8_t i = 0; i < 4; i++)
    {
        bool state = (_current_Pattern >> (3 - i)) & 0x01;
        digitalWrite(motorPins[i], state ? HIGH : LOW);
    }
}

void Bot::updateSpeed()
{
    _target_Speed = (_target_Speed > 255 ? 255 : _target_Speed);
    unsigned long now = millis();
    if (now - _last_UpdatedT >= _ramp_Interval)
    {
        _last_UpdatedT = now;

        if (_current_Speed < _target_Speed)
        {
            _current_Speed += _ramp_Step;
            if (_current_Speed > _target_Speed)
                _current_Speed = _target_Speed;
        }
        else if (_current_Speed > _target_Speed)
        {
            _current_Speed -= _ramp_Step;
            if (_current_Speed < _target_Speed)
                _current_Speed = _target_Speed;
        }

        analogWrite(_enR, _current_Speed);
        analogWrite(_enL, _current_Speed);
    }
}

void Bot::stop(uint8_t rampStep) {
    if (rampStep == 0) {
        // Hard stop, cuts speed and PWM immediately
        _target_Speed = 0;
        _current_Speed = 0;
        analogWrite(_enR, 0);
        analogWrite(_enL, 0);
        move(STOP);
    } else {
        // Soft stop, ramp down
        _ramp_Step = rampStep;
        _target_Speed = 0;
        move(STOP);
    }
}

void Bot::forward(uint8_t accSpeed, uint8_t speed)
{
    _ramp_Step = accSpeed;
    _target_Speed = speed;
    move(FORWARD);
    updateSpeed();
}

void Bot::reverse(uint8_t accSpeed, uint8_t speed)
{
    _ramp_Step = accSpeed;
    _target_Speed = speed;
    move(REVERSE);
    updateSpeed();
}

void Bot::left(uint8_t accSpeed, uint8_t speed)
{
    _ramp_Step = accSpeed;
    _target_Speed = speed;
    move(LEFT);
    updateSpeed();
}

void Bot::right(uint8_t accSpeed, uint8_t speed)
{
    _ramp_Step = accSpeed;
    _target_Speed = speed;
    move(RIGHT);
    updateSpeed();
}