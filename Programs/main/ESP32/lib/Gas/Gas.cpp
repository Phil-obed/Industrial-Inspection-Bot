#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "sensor_msg.hpp"

#define MQ135_PIN 34
#define MQ9_PIN   35

extern QueueHandle_t queueSensorMsgs;

const float ADC_REF = 3.3f;
const int ADC_MAX = 4095;
const float VSENSOR_MAX = 5.0f;
const float DIVIDER_FACTOR = 1.667f; // match your resistor values
static float ema135 = 0.0f;
static float ema9   = 0.0f;
const float EMA_ALPHA = 0.2f;

static float readGasPercent(int pin) {
  int raw = analogRead(pin);
  float v_adc = (raw / (float)ADC_MAX) * ADC_REF;
  float v_sensor = v_adc * DIVIDER_FACTOR;
  float p = (v_sensor / VSENSOR_MAX) * 100.0f;
  return constrain(p, 0.0f, 100.0f);
}

void TaskGasSensors(void *pvParameters) {
  (void)pvParameters;
  analogSetPinAttenuation(MQ135_PIN, ADC_11db);
  analogSetPinAttenuation(MQ9_PIN, ADC_11db);
  analogSetWidth(12);

  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(1000);

  for (;;) {
    float v135 = readGasPercent(MQ135_PIN);
    float v9   = readGasPercent(MQ9_PIN);

    ema135 = (EMA_ALPHA * v135) + ((1.0f - EMA_ALPHA) * ema135);
    ema9   = (EMA_ALPHA * v9) + ((1.0f - EMA_ALPHA) * ema9);

    SensorMsg_t msg;
    msg.type = MSG_GAS;
    msg.payload.gas.gas_Air = ema135;
    msg.payload.gas.gas_CH4 = ema9;
    msg.payload.gas.gas_CO = 0.0f;
    msg.payload.gas.gas_LPG = 0.0f;
    msg.payload.gas.ts = millis();
    xQueueSend(queueSensorMsgs, &msg, 0);

    // alert example
    if (ema9 > 85.0f) {
      // duplicate push as urgent alert, dashboard will show
      xQueueSend(queueSensorMsgs, &msg, 0);
    }
    vTaskDelayUntil(&lastWake, period);
  }
}
