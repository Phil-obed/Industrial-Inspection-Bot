/* ==================================================
   Industrial Inspection Bot - ESP32 Core (RTOS)
   ================================================== */

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

// ==================== WIFI CONFIG ====================
const char* ssid = "Argus-Bot";
const char* password = "12345678";

AsyncWebServer server(80);
AsyncWebSocket ws("/");

// ==================== SENSOR PINS ====================
// MQ sensors
#define MQ135_PIN 34
#define MQ9_PIN   35

// Ultrasonic sensors
#define TRIG1 12
#define ECHO1 14

// Battery pin
#define BATTERY_PIN 36

// GPS UART pins
#define GPS_RX 16
#define GPS_TX 17
// HardwareSerial gpsSerial(1);

// ==================== TASK HANDLES ====================
TaskHandle_t gasTaskHandle;
TaskHandle_t gpsTaskHandle;
TaskHandle_t ultrasonicTaskHandle;
TaskHandle_t batteryTaskHandle;
TaskHandle_t commsTaskHandle;

// ==================== SHARED DATA ====================
struct SensorData {
  float co;
  float ch4;
  float lpg;
  float airq;

  float lat;
  float lng;

  float ultrasonic[5];
  int battery;

} sensorData;   // global shared struct

SemaphoreHandle_t dataMutex;  // to protect shared data

// ==================== HELPERS ====================
void sendData() {
  StaticJsonDocument<512> doc;

  JsonObject gas = doc.createNestedObject("gas");
  gas["co"]   = sensorData.co;
  gas["ch4"]  = sensorData.ch4;
  gas["lpg"]  = sensorData.lpg;
  gas["airq"] = sensorData.airq;

  JsonObject gps = doc.createNestedObject("gps");
  gps["lat"] = sensorData.lat;
  gps["lng"] = sensorData.lng;

  JsonArray us = doc.createNestedArray("ultrasonic");
  for (int i = 0; i < 5; i++) us.add(sensorData.ultrasonic[i]);

  doc["battery"] = sensorData.battery;

  String json;
  serializeJson(doc, json);
  ws.textAll(json);
}

// ==================== TASKS ====================
void gasTask(void* pv) {
  for (;;) {
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    sensorData.co   = 0;   // placeholder
    sensorData.ch4  = 0;
    sensorData.lpg  = 0;
    sensorData.airq = 0;
    xSemaphoreGive(dataMutex);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

void gpsTask(void* pv) {
  for (;;) {
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    sensorData.lat = 0.0;  // placeholder
    sensorData.lng = 0.0;
    xSemaphoreGive(dataMutex);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void ultrasonicTask(void* pv) {
  for (;;) {
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    for (int i = 0; i < 5; i++) sensorData.ultrasonic[i] = 0; // placeholder
    xSemaphoreGive(dataMutex);
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void batteryTask(void* pv) {
  for (;;) {
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    sensorData.battery = 100; // placeholder
    xSemaphoreGive(dataMutex);
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

void commsTask(void* pv) {
  for (;;) {
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    sendData();
    xSemaphoreGive(dataMutex);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// ==================== CALLBACKS ====================
void onWebSocketEvent(AsyncWebSocket * server, AsyncWebSocketClient * client,
                      AwsEventType type, void * arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("Client %u connected\n", client->id());
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("Client %u disconnected\n", client->id());
  } else if (type == WS_EVT_DATA) {
    // handle incoming commands here
  }
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);

  // Mutex
  dataMutex = xSemaphoreCreateMutex();

  // WiFi
  WiFi.softAP(ssid, password);
  Serial.println(WiFi.softAPIP());

  // WebSocket
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);
  server.begin();

  // Tasks
  xTaskCreatePinnedToCore(gasTask, "Gas Task", 4096, NULL, 1, &gasTaskHandle, 1);
  xTaskCreatePinnedToCore(gpsTask, "GPS Task", 4096, NULL, 1, &gpsTaskHandle, 1);
  xTaskCreatePinnedToCore(ultrasonicTask, "Ultrasonic Task", 4096, NULL, 1, &ultrasonicTaskHandle, 1);
  xTaskCreatePinnedToCore(batteryTask, "Battery Task", 2048, NULL, 1, &batteryTaskHandle, 1);
  xTaskCreatePinnedToCore(commsTask, "Comms Task", 4096, NULL, 1, &commsTaskHandle, 0);
}

void loop() {
  ws.cleanupClients();
}
