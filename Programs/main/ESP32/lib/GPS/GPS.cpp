#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "sensor_msg.hpp"

// Pins and baud
static const int RXPin = 16;   // GPS TX -> ESP32 RX
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;
HardwareSerial SerialGPS(1);

// extern queue handle defined in main.cpp
extern QueueHandle_t queueSensorMsgs;

void TaskGPS(void *pvParameters) {
  SerialGPS.begin(GPSBaud, SERIAL_8N1, RXPin, -1);
  vTaskDelay(pdMS_TO_TICKS(200));

  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(500);
  for (;;) {
    while (SerialGPS.available()) {
      gps.encode(SerialGPS.read());
    }

    SensorMsg_t msg;
    msg.type = MSG_GPS;
    msg.payload.gps.valid = gps.location.isValid();
    if (msg.payload.gps.valid) {
      msg.payload.gps.lat = gps.location.lat();
      msg.payload.gps.lng = gps.location.lng();
    } else {
      msg.payload.gps.lat = 0; msg.payload.gps.lng = 0;
    }
    msg.payload.gps.sats = gps.satellites.isValid() ? (uint8_t)gps.satellites.value() : 0;
    msg.payload.gps.ts = millis();

    xQueueSend(queueSensorMsgs, &msg, 0);
    vTaskDelayUntil(&lastWake, period);
  }
}
