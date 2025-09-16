#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include "esp_task_wdt.h"   // Watchdog


// ================= PIN ASSIGNMENTS =================

// --- ULTRASONIC PINS ---
#define TRIG1 5
#define ECHO1 36
#define TRIG2 18
#define ECHO2 39
#define TRIG3 19
#define ECHO3 34
#define TRIG4 23
#define ECHO4 35
#define TRIG5 4
#define ECHO5 32

// --- GAS SENSORS ---
#define MQ135_PIN 33
#define MQ9_PIN   25

// --- GPS (UART2) ---
#define GPS_RX 16   // GPS TX -> ESP RX2

// --- THERMAL CAMERA (I²C) ---
#define I2C_SDA 21
#define I2C_SCL 22

// --- MOTOR DRIVER ---
#define MOTOR_EN1 26
#define MOTOR_EN2 27
#define MOTOR_IN1 12
#define MOTOR_IN2 13
#define MOTOR_IN3 14
#define MOTOR_IN4 15   

// --- SERVO (STEERING) ---
#define SERVO_PIN 2   

// --- CONFIG ---
#define WDT_TIMEOUT 10  // seconds

// --- GLOBALS & HANDLES ---
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// RTOS task handles
TaskHandle_t ultrasonicTaskHandle;
TaskHandle_t gasTaskHandle;
TaskHandle_t gpsTaskHandle;
TaskHandle_t thermalTaskHandle;
TaskHandle_t motorTaskHandle;
TaskHandle_t avoidanceTaskHandle;
TaskHandle_t commandTaskHandle;

// Queues
QueueHandle_t ultrasonicQueue;
QueueHandle_t gasQueue;
QueueHandle_t gpsQueue;
QueueHandle_t thermalQueue;
QueueHandle_t commandQueue;


// ---------------- UTILS ----------------
void sendJson(JsonDocument &doc) {
  String payload;
  serializeJson(doc, payload);
  ws.textAll(payload);
}


// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);

  // Wi-Fi + WebSocket
  setupWiFi();
  setupWebSocket();

  // Init queues
  ultrasonicQueue = xQueueCreate(5, sizeof(int[5]));
  gasQueue        = xQueueCreate(5, sizeof(float[2]));
  gpsQueue        = xQueueCreate(5, sizeof(double[2]));
  thermalQueue    = xQueueCreate(2, sizeof(uint16_t[32*24]));
  commandQueue    = xQueueCreate(10, sizeof(char[64]));

  // Init watchdog
  esp_task_wdt_init(WDT_TIMEOUT, true);  // panic=true = auto reset
  esp_task_wdt_add(NULL);                // Add main loop

  // Spawn tasks
  xTaskCreatePinnedToCore(ultrasonicTask, "Ultrasonic", 4096, NULL, 1, &ultrasonicTaskHandle, 0);
  xTaskCreatePinnedToCore(gasTask,        "Gas",        4096, NULL, 1, &gasTaskHandle, 0);
  xTaskCreatePinnedToCore(gpsTask,        "GPS",        4096, NULL, 1, &gpsTaskHandle, 1);
  xTaskCreatePinnedToCore(thermalTask,    "Thermal",    8192, NULL, 1, &thermalTaskHandle, 1);
  xTaskCreatePinnedToCore(motorTask,      "Motor",      4096, NULL, 1, &motorTaskHandle, 0);
  xTaskCreatePinnedToCore(avoidanceTask,  "Avoidance",  6144, NULL, 1, &avoidanceTaskHandle, 1);
  xTaskCreatePinnedToCore(commandTask,    "Command",    4096, NULL, 1, &commandTaskHandle, 0);
}


// ---------------- LOOP ----------------
void loop() {
  esp_task_wdt_reset();   // feed watchdog
  delay(1000);
}


// ---------------- TASKS ----------------
void ultrasonicTask(void *pvParameters) {
  esp_task_wdt_add(NULL);
  for (;;) {
    // TODO: Read ultrasonic sensors
    StaticJsonDocument<256> doc;
    doc["type"] = "ultrasonic";
    doc["d1"] = 0.0;  // placeholder
    doc["d2"] = 0.0;
    doc["d3"] = 0.0;
    doc["d4"] = 0.0;
    doc["d5"] = 0.0;
    sendJson(doc);

    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void gasTask(void *pvParameters) {
  esp_task_wdt_add(NULL);
  for (;;) {
    // TODO: MQ sensors
    StaticJsonDocument<128> doc;
    doc["type"] = "gas";
    doc["mq9"] = 0.0;     // placeholder
    doc["mq135"] = 0.0;
    sendJson(doc);

    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void gpsTask(void *pvParameters) {
  esp_task_wdt_add(NULL);
  for (;;) {
    // TODO: GPS coords
    StaticJsonDocument<128> doc;
    doc["type"] = "gps";
    doc["lat"] = 0.0;
    doc["lng"] = 0.0;
    sendJson(doc);

    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void thermalTask(void *pvParameters) {
  esp_task_wdt_add(NULL);
  for (;;) {
    // TODO: Thermal camera
    StaticJsonDocument<128> doc;
    doc["type"] = "thermal";
    doc["maxTemp"] = 0.0;
    sendJson(doc);

    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void motorTask(void *pvParameters) {
  esp_task_wdt_add(NULL);
  for (;;) {
    // TODO: Motor control
    StaticJsonDocument<64> doc;
    doc["type"] = "motor";
    doc["status"] = "idle";
    sendJson(doc);

    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void avoidanceTask(void *pvParameters) {
  esp_task_wdt_add(NULL);
  for (;;) {
    // TODO: VFH algorithm
    StaticJsonDocument<64> doc;
    doc["type"] = "avoidance";
    doc["decision"] = "none";
    sendJson(doc);

    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void commandTask(void *pvParameters) {
  esp_task_wdt_add(NULL);
  for (;;) {
    // TODO: handle commands pulled from queue
    char cmd[64];
    if (xQueueReceive(commandQueue, &cmd, 0) == pdTRUE) {
      Serial.printf("Executing command: %s\n", cmd);
    }
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}


// ---------------- COMMUNICATION ----------------
void setupWiFi() {
  WiFi.softAP("Bot_AP", "12345678");
  Serial.println("WiFi ready: " + WiFi.softAPIP().toString());
}

void setupWebSocket() {
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.begin();
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.println("Client connected");
  } else if (type == WS_EVT_DATA) {
    String msg = String((char*)data).substring(0, len);
    Serial.printf("From dash: %s\n", msg.c_str());
    xQueueSend(commandQueue, msg.c_str(), 0);
  }
}
