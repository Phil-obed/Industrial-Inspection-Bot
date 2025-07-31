#include "ObstacleAvoidance.hpp"

ObstacleAvoidance::ObstacleAvoidance(Bot* bot,
                                     uint8_t trigFL, uint8_t echoFL,
                                     uint8_t trigFC, uint8_t echoFC,
                                     uint8_t trigFR, uint8_t echoFR) {
    _bot = bot;
    sensors[FL] = { trigFL, echoFL, 400.0, 0, false };
    sensors[FC] = { trigFC, echoFC, 400.0, 0, false };
    sensors[FR] = { trigFR, echoFR, 400.0, 0, false };
}

void ObstacleAvoidance::begin() {
    for (int i = 0; i < 3; i++) {
        pinMode(sensors[i].trig, OUTPUT);
        pinMode(sensors[i].echo, INPUT);
        digitalWrite(sensors[i].trig, LOW);
    }
}

void ObstacleAvoidance::triggerSensor(Sensor& sensor) {
    digitalWrite(sensor.trig, LOW);
    delayMicroseconds(2);
    digitalWrite(sensor.trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(sensor.trig, LOW);
    sensor.startTime = micros();
    sensor.triggered = true;
}

void ObstacleAvoidance::readEcho(Sensor& sensor) {
    if (!sensor.triggered) return;

    unsigned long duration = pulseIn(sensor.echo, HIGH, _sensorTimeout);
    if (duration == 0) {
        sensor.distance = 400.0;  // No echo = assume clear
    } else {
        sensor.distance = duration * 0.0343 / 2.0;
    }

    sensor.triggered = false;
}

void ObstacleAvoidance::update() {
    unsigned long now = millis();

    // 1. Fire one sensor per cycle (non-blocking pulse)
    if (!_lastStateUpdate || now - _lastStateUpdate >= _sensorCycleDelay) {
        readEcho(sensors[_currentSensor]);           // read previous sensor
        _currentSensor = (_currentSensor + 1) % 3;    // next sensor
        triggerSensor(sensors[_currentSensor]);       // fire next sensor
        _lastStateUpdate = now;
    }

    // 2. Decision logic
    float dFL = sensors[FL].distance;
    float dFC = sensors[FC].distance;
    float dFR = sensors[FR].distance;

    if (dFC < STOP_DIST) {
        _bot->stop(5);
        if (dFL > dFR && dFL > AVOID_DIST) {
            _bot->left(_accStep, _speed);
        } else if (dFR > AVOID_DIST) {
            _bot->right(_accStep, _speed);
        } else {
            _bot->reverse(_accStep, _speed / 2);
        }
    }
    else if (dFL < AVOID_DIST || dFR < AVOID_DIST) {
        if (dFL < dFR) {
            _bot->right(_accStep, _speed);
        } else {
            _bot->left(_accStep, _speed);
        }
    }
    else {
        _bot->forward(_accStep, _speed);
    }
}
