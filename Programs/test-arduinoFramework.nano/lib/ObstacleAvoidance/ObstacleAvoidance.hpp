#pragma once
#include <Arduino.h>
#include "MotorControl_L293D.hpp"

class ObstacleAvoidance {
    private:
        struct Sensor {
            uint8_t trig;
            uint8_t echo;
            float distance;
            unsigned long startTime;
            bool triggered;
        };

        Bot* _bot;

        Sensor sensors[3];
        enum SensorIndex { FL = 0, FC = 1, FR = 2 };
        uint8_t _currentSensor = 0;

        unsigned long _lastStateUpdate = 0;
        unsigned long _sensorCycleDelay = 5;  // 5ms between sensors
        unsigned long _sensorTimeout = 30000; // 30ms timeout for echo

        const float STOP_DIST = 20.0;
        const float AVOID_DIST = 35.0;

        const uint8_t _speed = 180;
        const uint8_t _accStep = 10;

        void triggerSensor(Sensor& sensor);
        void readEcho(Sensor& sensor);

    public:
        ObstacleAvoidance(Bot* bot,
                        uint8_t trigFL, uint8_t echoFL,
                        uint8_t trigFC, uint8_t echoFC,
                        uint8_t trigFR, uint8_t echoFR);

        void begin();
        void update();
};
