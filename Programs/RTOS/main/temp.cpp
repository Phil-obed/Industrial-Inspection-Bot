/*
  ESP32 FreeRTOS Inspection Robot - Full RTOS Skeleton
  - Tasks: Safety, MotorControl, DashboardComms (WebSocket), Ultrasonic, GasSensors, ThermalCamera, GPS
  - Queues: queueCmd, queueComms, queueThermalFrames
  - ISR: E-STOP (hardware emergency stop)
  - Binary thermal frames (uint16_t[768]) are sent over WebSocket to dashboard
  NOTES:
    - Fill hardware-specific functions (MLX90640 reads, ultrasonic timing, MQ ADC conversion)
    - Libraries required: AsyncTCP, ESPAsyncWebServer, ArduinoJson
*/

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// ------------------- CONFIG -------------------
#define WIFI_SSID     "YOUR_SSID"
#define WIFI_PASSWORD "YOUR_PASS"

// pins (adjust for your wiring)
const int PIN_ESTOP = 32;        // E-STOP (active LOW)
const int PIN_MOTOR_EN = 26;     // example motor enable if used

// LEDC channels for 4 motors (example)
const int MOTOR_CH_LEFT_A  = 0;  // LEDC channel 0
const int MOTOR_CH_LEFT_B  = 1;  // LEDC channel 1
const int MOTOR_CH_RIGHT_A = 2;
const int MOTOR_CH_RIGHT_B = 3;
const int LEDC_FREQ = 20000;     // 20 kHz PWM
const int LEDC_RES = 8;          // 8-bit resolution

// MLX90640 resolution
const int TH_WIDTH = 32;
const int TH_HEIGHT = 24;
const int TH_PIXELS = TH_WIDTH * TH_HEIGHT; // 768

// ------------------- TYPES -------------------
typedef enum { CMD_STOP = 0, CMD_MOVE, CMD_SET_SPEED, CMD_GOTO } CmdType_t;

typedef struct {
  CmdType_t type;
  int16_t leftSpeed;   // -1000 .. 1000
  int16_t rightSpeed;
  double lat, lng;     // for GOTO
  uint32_t ts;
} Cmd_t;

typedef struct {
  float ultrasonic[6];      // up to 6 sensors
  float gas_CO;
  float gas_CH4;
  float gas_LPG;
  float gas_Air;
  double lat, lng;
  uint8_t sats;
  float thermalMax;
  bool thermalAlert;
  uint32_t ts;
} SensorFrame_t;

// ------------------- RTOS objects -------------------
QueueHandle_t queueCmd;
QueueHandle_t queueComms;
QueueHandle_t queueThermalFrames; // each item: uint16_t[768]
SemaphoreHandle_t i2cMutex;

TaskHandle_t handleSafety = NULL;
TaskHandle_t handleMotor = NULL;
TaskHandle_t handleComms = NULL;

// ------------------- Globals -------------------
volatile bool estop_triggered = false;
volatile bool wifiConnected = false;

// Web server & websocket
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ------------------- UTILITY / HW PLACEHOLDERS -------------------

// Map scaled speed (-1000..1000) to LEDC duty (0..255)
static inline int speedToDuty(int16_t s) {
  int v = constrain(s, -1000, 1000);
  // convert abs speed to 0..255
  int duty = map(abs(v), 0, 1000, 0, 255);
  return duty;
}

// Very fast immediate motor stop (should be hardware-safe & fast)
void motorStopImmediate() {
  // set PWM to 0 on channels
  ledcWrite(MOTOR_CH_LEFT_A, 0);
  ledcWrite(MOTOR_CH_LEFT_B, 0);
  ledcWrite(MOTOR_CH_RIGHT_A, 0);
  ledcWrite(MOTOR_CH_RIGHT_B, 0);
  // optional: assert motor EN pin low
  if (PIN_MOTOR_EN >= 0) digitalWrite(PIN_MOTOR_EN, LOW);
}

// Apply motor speeds (leftSpeed, rightSpeed in -1000..1000)
void applyMotorCommand(int16_t leftSpeed, int16_t rightSpeed) {
  // Example H-bridge control: two channels per motor (A/B)
  // Replace with your motor driver logic (direction pins + PWM)
  int leftDuty  = speedToDuty(leftSpeed);
  int rightDuty = speedToDuty(rightSpeed);

  // Example: sign = DIR, duty = PWM
  if (leftSpeed >= 0) {
    ledcWrite(MOTOR_CH_LEFT_A, leftDuty);
    ledcWrite(MOTOR_CH_LEFT_B, 0);
  } else {
    ledcWrite(MOTOR_CH_LEFT_A, 0);
    ledcWrite(MOTOR_CH_LEFT_B, leftDuty);
  }

  if (rightSpeed >= 0) {
    ledcWrite(MOTOR_CH_RIGHT_A, rightDuty);
    ledcWrite(MOTOR_CH_RIGHT_B, 0);
  } else {
    ledcWrite(MOTOR_CH_RIGHT_A, 0);
    ledcWrite(MOTOR_CH_RIGHT_B, rightDuty);
  }

  // ensure EN high
  if (PIN_MOTOR_EN >= 0) digitalWrite(PIN_MOTOR_EN, HIGH);
}

// Placeholder: read ultrasonic sensor (implement with RMT or timer)
float readUltrasonicSingle(int idx) {
  // TODO: implement real reading
  // For now return simulated 120 cm
  return 120.0f;
}

// Placeholder: read gas from ADC and return normalized 0..100
float readGasCO()   { return random(0, 100); }
float readGasCH4()  { return random(0, 100); }
float readGasLPG()  { return random(0, 100); }
float readGasAir()  { return random(0, 100); }

// Placeholder: read MLX90640 frame into uint16_t buffer (temp*100)
bool readMlx90640Frame(uint16_t *outFrame) {
  // Replace this with actual MLX90640 driver read
  for (int i = 0; i < TH_PIXELS; ++i) {
    float fakeT = 25.0f + sin((millis() / 1000.0f) + i * 0.01f) * 5.0f;
    // store as uint16_t scaled by 100
    int v = (int)round(fakeT * 100.0f);
    outFrame[i] = (uint16_t)v;
  }
  return true;
}

// GPS NMEA parse placeholder: produce lat,lng,sats
void parseGpsAndUpdate(double &lat, double &lng, uint8_t &sats) {
  // Replace with actual NMEA parsing from Serial1
  lat = 7.351136 + random(-10, 10) * 1e-6;
  lng = -2.341782 + random(-10, 10) * 1e-6;
  sats = 6;
}

// ------------------- ISR -------------------
void IRAM_ATTR isr_estop(void* arg) {
  estop_triggered = true;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (handleSafety) {
    vTaskNotifyGiveFromISR(handleSafety, &xHigherPriorityTaskWoken);
  }
  if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
}

// ------------------- TASKS -------------------

// Safety task: highest priority, reacts to E-STOP & battery (fast)
void TaskSafety(void *pvParameters) {
  const TickType_t period = pdMS_TO_TICKS(50);
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    // Block until notified by ISR OR wait period
    ulTaskNotifyTake(pdTRUE, 0); // clear notification if any (non-blocking)
    if (estop_triggered) {
      // Clear and act
      estop_triggered = false;
      motorStopImmediate();

      // publish an emergency frame to comms queue
      SensorFrame_t f = {};
      f.ts = millis();
      f.gas_CH4 = -1.0f; // sentinel: E-STOP
      xQueueSend(queueComms, &f, 0);
    }

    // TODO: check battery (ADC) and feed watchdog if used
    // e.g., if battery low => motorStopImmediate() and send alert

    vTaskDelayUntil(&lastWake, period);
  }
}

// Motor control: reads queueCmd and applies motors, periodic loop
void TaskMotorControl(void *pvParameters) {
  Cmd_t cmd;
  // keep last command to repeat
  Cmd_t lastCmd; lastCmd.type = CMD_STOP; lastCmd.leftSpeed = 0; lastCmd.rightSpeed = 0;

  TickType_t period = pdMS_TO_TICKS(10);
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    // fetch latest command (drain queue to take most recent)
    while (xQueueReceive(queueCmd, &cmd, 0) == pdTRUE) {
      lastCmd = cmd;
    }

    // Execute lastCmd
    if (lastCmd.type == CMD_STOP) {
      motorStopImmediate();
    } else {
      applyMotorCommand(lastCmd.leftSpeed, lastCmd.rightSpeed);
    }

    vTaskDelayUntil(&lastWake, period);
  }
}

// DashboardComms: WebSocket server handler: sends sensor frames & binary thermal frames
void sendSensorFrameJson(const SensorFrame_t &f) {
  // Build JSON (small) and broadcast
  StaticJsonDocument<256> doc;
  doc["ts"] = f.ts;
  doc["gas"] = JsonObject();
  doc["gas"]["CO"] = f.gas_CO;
  doc["gas"]["CH4"] = f.gas_CH4;
  doc["gas"]["LPG"] = f.gas_LPG;
  doc["gas"]["Air"] = f.gas_Air;
  doc["thermalMax"] = f.thermalMax;
  doc["thermalAlert"] = f.thermalAlert;
  doc["ultrasonic0"] = f.ultrasonic[0];
  doc["lat"] = f.lat;
  doc["lng"] = f.lng;
  doc["sats"] = f.sats;
  char buf[256];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  ws.textAll(buf, n);
}

void TaskDashboardComms(void *pvParameters) {
  SensorFrame_t frame;
  uint16_t frameBuf[TH_PIXELS];

  for (;;) {
    // Priority: check for thermal frames and send them as binary
    if (xQueueReceive(queueThermalFrames, &frameBuf, pdMS_TO_TICKS(200)) == pdTRUE) {
      // send binary raw frame (uint16_t array)
      // AsyncWebSocket expects (uint8_t*) + len
      ws.binaryAll((uint8_t*)frameBuf, sizeof(frameBuf));
    }
    // send aggregated sensor frames (non-blocking)
    while (xQueueReceive(queueComms, &frame, 0) == pdTRUE) {
      sendSensorFrameJson(frame);
    }

    // small delay to yield CPU if idle
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// Ultrasonic: read sensors and push to queueComms
void TaskUltrasonic(void *pvParameters) {
  SensorFrame_t f;
  memset(&f, 0, sizeof(f));
  for (;;) {
    // read up to 6 sensors (implement your own)
    for (int i = 0; i < 6; ++i) {
      f.ultrasonic[i] = readUltrasonicSingle(i);
    }
    f.ts = millis();
    xQueueSend(queueComms, &f, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// Gas sensor task: read ADCs, process, push small updates
void TaskGasSensors(void *pvParameters) {
  SensorFrame_t f;
  for (;;) {
    f.gas_CO = readGasCO();
    f.gas_CH4 = readGasCH4();
    f.gas_LPG = readGasLPG();
    f.gas_Air = readGasAir();
    f.ts = millis();
    xQueueSend(queueComms, &f, 0);

    // if threshold exceeded -> also notify safety via queueComms (safety monitors this)
    if (f.gas_CH4 > 85.0f) { // example threshold
      // send flag to safety via queueComms
      SensorFrame_t alert = f;
      alert.gas_CH4 = f.gas_CH4;
      xQueueSend(queueComms, &alert, 0);
    }

    vTaskDelay(pdMS_TO_TICKS(1000)); // 1Hz
  }
}

// Thermal camera task: read frame, push to thermal queue and send small summary to comms
void TaskThermalCamera(void *pvParameters) {
  uint16_t frame[TH_PIXELS];
  SensorFrame_t summary;
  for (;;) {
    // get exclusive I2C access
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
      bool ok = readMlx90640Frame(frame); // placeholder
      xSemaphoreGive(i2cMutex);

      if (ok) {
        // push binary frame to queue (non-blocking)
        if (xQueueSend(queueThermalFrames, &frame, 0) != pdPASS) {
          // queue full; drop oldest by receiving one then sending new
          uint16_t tmp[TH_PIXELS];
          xQueueReceive(queueThermalFrames, &tmp, 0);
          xQueueSend(queueThermalFrames, &frame, 0);
        }

        // compute small summary (max/avg)
        float maxT = -1000.0f, sum = 0.0f;
        for (int i = 0; i < TH_PIXELS; ++i) {
          float t = frame[i] / 100.0f;
          sum += t;
          if (t > maxT) maxT = t;
        }
        float avg = sum / TH_PIXELS;
        summary.thermalMax = maxT;
        summary.thermalAlert = (maxT > 60.0f); // example threshold
        summary.ts = millis();
        xQueueSend(queueComms, &summary, 0);
      }
    } // if sem taken

    vTaskDelay(pdMS_TO_TICKS(200)); // aim for 4-5 FPS
  }
}

// GPS task: read Serial1 (UART) and parse NMEA, push latest to comms
void TaskGPS(void *pvParameters) {
  SensorFrame_t f;
  for (;;) {
    double lat, lng;
    uint8_t sats;
    parseGpsAndUpdate(lat, lng, sats); // placeholder
    f.lat = lat; f.lng = lng; f.sats = sats; f.ts = millis();
    xQueueSend(queueComms, &f, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// ------------------- WebSocket handlers -------------------
void handleWsMessage(void *arg, uint8_t *data, size_t len) {
  // data is text JSON or command string
  // parse JSON
  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, data, len);
  if (!err) {
    if (doc.containsKey("cmd")) {
      const char* cmd = doc["cmd"];
      Cmd_t c = {};
      c.ts = millis();
      if (strcmp(cmd, "stop") == 0) {
        c.type = CMD_STOP;
        xQueueSend(queueCmd, &c, 0);
      } else if (strcmp(cmd, "move") == 0 && doc.containsKey("left") && doc.containsKey("right")) {
        c.type = CMD_MOVE;
        c.leftSpeed = doc["left"];
        c.rightSpeed = doc["right"];
        xQueueSend(queueCmd, &c, 0);
      } else if (strcmp(cmd, "goto") == 0 && doc.containsKey("lat") && doc.containsKey("lng")) {
        c.type = CMD_GOTO;
        c.lat = doc["lat"];
        c.lng = doc["lng"];
        xQueueSend(queueCmd, &c, 0);
      } else if (strcmp(cmd, "mode") == 0 && doc.containsKey("value")) {
        // set mode logic (not implemented)
      }
    }
  } else {
    // fallback: plain text commands
    String s((char*)data, len);
    s.trim();
    if (s.equalsIgnoreCase("stop")) {
      Cmd_t c = {}; c.type = CMD_STOP; c.ts = millis();
      xQueueSend(queueCmd, &c, 0);
    } else if (s.equalsIgnoreCase("move forward")) {
      Cmd_t c = {}; c.type = CMD_MOVE; c.leftSpeed = 600; c.rightSpeed = 600; c.ts = millis();
      xQueueSend(queueCmd, &c, 0);
    } else if (s.equalsIgnoreCase("move back")) {
      Cmd_t c = {}; c.type = CMD_MOVE; c.leftSpeed = -400; c.rightSpeed = -400; c.ts = millis();
      xQueueSend(queueCmd, &c, 0);
    }
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("WebSocket client #%u connected\n", client->id());
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->opcode == WS_TEXT) {
      // text message
      handleWsMessage(arg, data, len);
    }
    // optionally handle binary from client (not used)
  }
}

// ------------------- Setup & loop -------------------
void initLEDC() {
  // configure LEDC channels (4 channels)
  ledcSetup(MOTOR_CH_LEFT_A, LEDC_FREQ, LEDC_RES);
  ledcSetup(MOTOR_CH_LEFT_B, LEDC_FREQ, LEDC_RES);
  ledcSetup(MOTOR_CH_RIGHT_A, LEDC_FREQ, LEDC_RES);
  ledcSetup(MOTOR_CH_RIGHT_B, LEDC_FREQ, LEDC_RES);
  // attach pins (set real motor pins)
  // ledcAttachPin(PIN_MOTOR_LEFT_A, MOTOR_CH_LEFT_A); // define your motor pins
  // ...
}

void setupWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("Connecting to WiFi %s ", WIFI_SSID);
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries++ < 20) {
    Serial.print('.');
    delay(500);
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected");
    Serial.print("IP: "); Serial.println(WiFi.localIP());
    wifiConnected = true;
  } else {
    Serial.println("\nWiFi failed");
    wifiConnected = false;
  }
}

void setupWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
  server.begin();
  Serial.println("WebSocket server started at /ws");
}

void setup() {
  Serial.begin(115200);
  delay(100);

  // init pins
  pinMode(PIN_ESTOP, INPUT_PULLUP); // active LOW
  attachInterruptArg(PIN_ESTOP, isr_estop, NULL, FALLING);

  pinMode(PIN_MOTOR_EN, OUTPUT);
  digitalWrite(PIN_MOTOR_EN, LOW);

  initLEDC();

  // create queues & mutex
  queueCmd = xQueueCreate(8, sizeof(Cmd_t));
  queueComms = xQueueCreate(12, sizeof(SensorFrame_t));
  // queueThermalFrames: each entry is TH_PIXELS * sizeof(uint16_t)
  queueThermalFrames = xQueueCreate(2, TH_PIXELS * sizeof(uint16_t));
  i2cMutex = xSemaphoreCreateMutex();

  // Connect WiFi & start WebSocket server
  setupWiFi();
  setupWebSocket();

  // create tasks pinned to cores (adjust stack sizes as needed)
  xTaskCreatePinnedToCore(TaskSafety, "Safety", 2048, NULL, 7, &handleSafety, 1);
  xTaskCreatePinnedToCore(TaskMotorControl, "Motor", 4096, NULL, 6, &handleMotor, 1);
  xTaskCreatePinnedToCore(TaskDashboardComms, "Comms", 12288, NULL, 5, &handleComms, 0);
  xTaskCreatePinnedToCore(TaskUltrasonic, "Ultra", 4096, NULL, 4, NULL, 1);
  xTaskCreatePinnedToCore(TaskGasSensors, "Gas", 2048, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(TaskThermalCamera, "Therm", 8192, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(TaskGPS, "GPS", 4096, NULL, 3, NULL, 1);

  Serial.println("All tasks created");
}

void loop() {
  // Nothing here — RTOS tasks run independently
  delay(1000);
}
