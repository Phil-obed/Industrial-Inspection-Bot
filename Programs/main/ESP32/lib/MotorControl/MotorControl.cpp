#include "MotorControl_L293D.hpp"

Bot::Bot(uint8_t Rf, uint8_t Rb, uint8_t Lf, uint8_t Lb,
         uint8_t enR, uint8_t enL, uint8_t chR, uint8_t chL) {
    motorPins[0] = Rf; motorPins[1] = Rb; motorPins[2] = Lf; motorPins[3] = Lb;
    _enR = enR; _enL = enL; _chR = chR; _chL = chL;
    _current_Speed = 0; _target_Speed = 0; _ramp_Step = 0;
    _ramp_Interval = 10; _last_UpdatedT = 0; _current_Pattern = STOP;
    // do not call stop here; call init() first
}

void Bot::init() {
    for (uint8_t i = 0; i < 4; i++) pinMode(motorPins[i], OUTPUT);

    // Setup LEDC channels for ESP32 PWM (5kHz, 8-bit)
    ledcSetup(_chR, 5000, 8);
    ledcSetup(_chL, 5000, 8);
    ledcAttachPin(_enR, _chR);
    ledcAttachPin(_enL, _chL);

    // ensure motors stopped
    stop(0);
}

void Bot::move(uint8_t pattern) {
    _current_Pattern = pattern;
    for (uint8_t i = 0; i < 4; i++) {
        bool state = (_current_Pattern >> (3 - i)) & 0x01;
        digitalWrite(motorPins[i], state ? HIGH : LOW);
    }
}

void Bot::updateSpeed() {
    if (_target_Speed > 255) _target_Speed = 255;
    unsigned long now = millis();
    if (now - _last_UpdatedT >= _ramp_Interval) {
        _last_UpdatedT = now;
        if (_current_Speed < _target_Speed) {
            _current_Speed = min<uint8_t>(_current_Speed + _ramp_Step, _target_Speed);
        } else if (_current_Speed > _target_Speed) {
            uint8_t dec = _ramp_Step;
            if (dec == 0) _current_Speed = _target_Speed;
            else _current_Speed = max<int>(_current_Speed - dec, _target_Speed);
        }
        ledcWrite(_chR, _current_Speed);
        ledcWrite(_chL, _current_Speed);
    }
}

void Bot::stop(uint8_t rampStep) {
    if (rampStep == 0) {
        _target_Speed = 0; _current_Speed = 0;
        ledcWrite(_chR, 0); ledcWrite(_chL, 0);
        move(STOP);
    } else {
        _ramp_Step = rampStep;
        _target_Speed = 0;
        move(STOP);
    }
}

void Bot::forward(uint8_t accSpeed, uint8_t speed) {
    _ramp_Step = accSpeed; _target_Speed = speed; move(FORWARD);
}
void Bot::reverse(uint8_t accSpeed, uint8_t speed) {
    _ramp_Step = accSpeed; _target_Speed = speed; move(REVERSE);
}
void Bot::left(uint8_t accSpeed, uint8_t speed) {
    _ramp_Step = accSpeed; _target_Speed = speed; move(LEFT);
}
void Bot::right(uint8_t accSpeed, uint8_t speed) {
    _ramp_Step = accSpeed; _target_Speed = speed; move(RIGHT);
}
