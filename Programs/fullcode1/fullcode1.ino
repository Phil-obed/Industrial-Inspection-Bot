/* Argus Bot - ESP32 RTOS sketch
   Publishes JSON updates via AsyncWebSocket on /ws to the dashboard.
   - Ultrasonics (5)
   - Gas MQ9 / MQ135 (ADC)
   - GPS (TinyGPS++)
   - Thermal MLX90640 (summary)
   - Motor control (2 motors + servo)
   - Simple avoidance

   Note: tune pin defines and PWM channels to your hardware.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include "esp_task_wdt.h"
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <Adafruit_MLX90640.h>

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

const int TRIG_PINS[5] = {TRIG1, TRIG2, TRIG3, TRIG4, TRIG5};
const int ECHO_PINS[5] = {ECHO1, ECHO2, ECHO3, ECHO4, ECHO5};

// --- GAS SENSORS ---
#define MQ135_PIN 33
#define MQ9_PIN   25

// --- GPS (UART2) ---
#define GPS_RX 16
#define GPS_TX 17

// --- THERMAL CAMERA (I2C) ---
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

// LEDC channels / pwm config
#define PWM_FREQ_MOTOR 20000
#define PWM_RES_MOTOR 8     // 0-255
#define PWM_CH_EN1 0
#define PWM_CH_EN2 1

// Servo (50Hz) LEDC config
#define PWM_FREQ_SERVO 50
#define PWM_RES_SERVO 16
#define PWM_CH_SERVO 2

// --- CONFIG ---
#define WDT_TIMEOUT 10  // seconds

// waypoints / navigation
bool navActive = false;
double navTargetLat = 0.0, navTargetLng = 0.0;
unsigned long lastNavStep = 0;

// Websocket & server
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
QueueHandle_t commandQueue;

// GPS + MLX + globals
HardwareSerial GPSserial(2);
TinyGPSPlus gps;
Adafruit_MLX90640 mlx;

float latestUltrasonic[5] = {400,400,400,400,400};
float latestGasMQ9 = 0.0;
float latestGasMQ135 = 0.0;
double latestLat = 0.0, latestLng = 0.0;
bool gpsHasFix = false;
float latestThermalMin = 0.0, latestThermalMax = 0.0, latestThermalAvg = 0.0;

// VFH / avoidance thresholds
const float D_BLOCK = 22.0;  // cm
const float D_FREE  = 28.0;  // cm

// motor state
uint8_t currentSpeed = 0;
String currentMotion = "stopped";
int currentSteering = 90; // degrees

// helper prototypes
void motorStop();
void motorForward(uint8_t speed);
void motorBackward(uint8_t speed);
void motorLeft(uint8_t speed);
void motorRight(uint8_t speed);
void setServoAngle(int angle);
void sendJson(JsonDocument &doc);
void rampMotorForward(uint8_t targetSpeed, uint8_t step = 10, int stepDelay = 30);
float readUltrasonicCm(int trigPin, int echoPin, int samples = 3);

// Task prototypes
void ultrasonicTask(void *pvParameters);
void gasTask(void *pvParameters);
void gpsTask(void *pvParameters);
void thermalTask(void *pvParameters);
void motorTask(void *pvParameters);
void avoidanceTask(void *pvParameters);
void commandTask(void *pvParameters);

// ---------------- UTILS ----------------
void sendJson(JsonDocument &doc) {
  String payload;
  serializeJson(doc, payload);
  ws.textAll(payload);
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  delay(100);

  // pin modes
  for (int i=0;i<5;i++) {
    pinMode(TRIG_PINS[i], OUTPUT);
    digitalWrite(TRIG_PINS[i], LOW);
    pinMode(ECHO_PINS[i], INPUT);
  }

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_IN3, OUTPUT);
  pinMode(MOTOR_IN4, OUTPUT);

  pinMode(MQ135_PIN, INPUT);
  pinMode(MQ9_PIN, INPUT);

  // LEDC (motor EN pins)
  ledcSetup(PWM_CH_EN1, PWM_FREQ_MOTOR, PWM_RES_MOTOR);
  ledcAttachPin(MOTOR_EN1, PWM_CH_EN1);
  ledcSetup(PWM_CH_EN2, PWM_FREQ_MOTOR, PWM_RES_MOTOR);
  ledcAttachPin(MOTOR_EN2, PWM_CH_EN2);

  // Servo PWM
  ledcSetup(PWM_CH_SERVO, PWM_FREQ_SERVO, PWM_RES_SERVO);
  ledcAttachPin(SERVO_PIN, PWM_CH_SERVO);

  // Motor default stop
  motorStop();

  // I2C for MLX
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  if (!mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire)) {
    Serial.println("MLX90640 not found - thermal disabled.");
  } else {
    Serial.println("MLX90640 OK");
    mlx.setRefreshRate(MLX90640_8_HZ);
  }

  // GPS UART2
  GPSserial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("GPS UART2 started");

  // Wi-Fi + WebSocket
  WiFi.softAP("Bot_AP", "12345678");
  Serial.printf("AP IP: %s\n", WiFi.softAPIP().toString().c_str());

  // WebSocket handlers
  ws.onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client,
                AwsEventType type, void *arg, uint8_t *data, size_t len){
    if (type == WS_EVT_CONNECT) {
      Serial.printf("WS client %u connected\n", client->id());
      DynamicJsonDocument doc(256);
      doc["type"] = "status";
      doc["msg"] = "welcome";
      String out;
      serializeJson(doc, out);
      server->textAll(out);
    } else if (type == WS_EVT_DISCONNECT) {
      Serial.printf("WS client %u disconnected\n", client->id());
    } else if (type == WS_EVT_DATA && len > 0) {
        String msg = String((char*)data).substring(0, len);
        if (msg == "ping") return;  // ignore keep-alives
        Serial.printf("From dash: %s\n", msg.c_str());
        if (commandQueue) {
            char buf[128];
            strncpy(buf, msg.c_str(), sizeof(buf)-1);
            buf[sizeof(buf)-1] = 0;
            xQueueSend(commandQueue, buf, 0);
        }
    }
  });

  server.addHandler(&ws);
  server.begin();

  // Create command queue (strings up to 128 chars)
  commandQueue = xQueueCreate(10, 128);

  // Setup watchdog
  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL); // main loop registered

  // Create tasks
  xTaskCreatePinnedToCore(ultrasonicTask, "Ultrasonic", 4096, NULL, 2, &ultrasonicTaskHandle, 0);
  xTaskCreatePinnedToCore(gasTask,        "Gas",        4096, NULL, 1, &gasTaskHandle, 0);
  xTaskCreatePinnedToCore(gpsTask,        "GPS",        4096, NULL, 1, &gpsTaskHandle, 1);
  xTaskCreatePinnedToCore(thermalTask,    "Thermal",    8192, NULL, 1, &thermalTaskHandle, 1);
  xTaskCreatePinnedToCore(motorTask,      "Motor",      4096, NULL, 3, &motorTaskHandle, 0);
  xTaskCreatePinnedToCore(avoidanceTask,  "Avoidance",  6144, NULL, 3, &avoidanceTaskHandle, 1);
  xTaskCreatePinnedToCore(commandTask,    "Command",    4096, NULL, 2, &commandTaskHandle, 0);

  Serial.println("Setup complete");
}

// ---------------- LOOP ----------------
void loop() {
  esp_task_wdt_reset();
  delay(1000);
}

// ---------------- SENSOR / ACT TASKS ----------------
void ultrasonicTask(void *pvParameters) {
  esp_task_wdt_add(NULL);
  for (;;) {
    for (int i = 0; i < 5; i++) {
      latestUltrasonic[i] = readUltrasonicCm(TRIG_PINS[i], ECHO_PINS[i]);
      delay(50);
    }

    DynamicJsonDocument doc(256);
    doc["type"] = "ultrasonic";
    JsonArray arr = doc.createNestedArray("dist");
    for (int i = 0; i < 5; i++) arr.add(latestUltrasonic[i]);

    serializeJson(doc, Serial);
    Serial.println();

    sendJson(doc);

    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void gasTask(void *pvParameters) {
  esp_task_wdt_add(NULL);
  const float ADC_REF = 3.3;
  const int ADC_MAX = 4095;
  for (;;) {
    int raw135 = analogRead(MQ135_PIN);
    int raw9 = analogRead(MQ9_PIN);
    latestGasMQ135 = (raw135 / (float)ADC_MAX) * 100.0;
    latestGasMQ9 = (raw9 / (float)ADC_MAX) * 100.0;

    StaticJsonDocument<128> doc;
    doc["type"] = "gas";
    doc["mq135_pct"] = latestGasMQ135;
    doc["mq9_pct"] = latestGasMQ9;

    sendJson(doc);

    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void gpsTask(void *pvParameters) {
  esp_task_wdt_add(NULL);
  for (;;) {
    while (GPSserial.available()) gps.encode(GPSserial.read());

    if (gps.location.isValid()) {
      latestLat = gps.location.lat();
      latestLng = gps.location.lng();
      gpsHasFix = true;

      StaticJsonDocument<128> doc;
      doc["type"] = "gps";
      doc["lat"] = latestLat;
      doc["lng"] = latestLng;
      doc["sats"] = gps.satellites.isValid() ? gps.satellites.value() : 0;
      doc["fix"] = true;
      sendJson(doc);

      Serial.printf("[GPS] Fix: YES Lat: %.6f Lng: %.6f Sats: %d\n",
                    latestLat, latestLng,
                    gps.satellites.isValid() ? gps.satellites.value() : 0);
    } else {
      StaticJsonDocument<128> doc;
      doc["type"] = "gps";
      doc["fix"] = false;
      sendJson(doc);
      Serial.println("[GPS] Fix: NO");
    }

    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void thermalTask(void *pvParameters) {
  esp_task_wdt_add(NULL);
  float frame[32*24];
  for (;;) {
    if (mlx.getFrame(frame) == 0) {
      float sum = 0;
      float tmin = frame[0];
      float tmax = frame[0];
      for (int i=0;i<32*24;i++) {
        float v = frame[i]; sum += v;
        if (v < tmin) tmin = v;
        if (v > tmax) tmax = v;
      }
      float tavg = sum / (32.0*24.0);
      latestThermalMin = tmin; latestThermalMax = tmax; latestThermalAvg = tavg;

      StaticJsonDocument<128> doc;
      doc["type"] = "thermal";
      doc["min"] = tmin;
      doc["avg"] = tavg;
      doc["max"] = tmax;
      sendJson(doc);
    }
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void motorTask(void *pvParameters) {
  esp_task_wdt_add(NULL);
  for (;;) {
    StaticJsonDocument<128> doc;
    doc["type"] = "motor";
    doc["status"] = currentMotion;
    doc["speed"] = currentSpeed;
    doc["steering_deg"] = currentSteering;
    sendJson(doc);

    Serial.printf("[MOTOR] Motion=%s Speed=%d Steering=%d\n",
                  currentMotion.c_str(), currentSpeed, currentSteering);

    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void avoidanceTask(void *pvParameters) {
  esp_task_wdt_add(NULL);
  const float T_block = 1.0f / (D_BLOCK * D_BLOCK);
  const float T_free  = 1.0f / (D_FREE * D_FREE);
  enum BinState { BIN_FREE=0, BIN_BLOCKED=1 };
  BinState binStateArr[5];
  for (int i=0;i<5;i++) binStateArr[i] = BIN_FREE;
  const int HYST_COUNT = 2;
  int pendingCount[5] = {0};
  BinState pendingState[5];
  for (int i=0;i<5;i++) pendingState[i] = BIN_FREE;

  for (;;) {
    if (navActive) { vTaskDelay(pdMS_TO_TICKS(200)); continue; }

    // ensure moving forward if nothing
    if (currentSpeed < 255) rampMotorForward(255);

    for (int i=0;i<5;i++) {
      float d = latestUltrasonic[i];
      if (d <= 0) continue;
      if (d > 300) d = 300;
      float H = 1.0f / (d * d);
      BinState target;
      if (H > T_block) target = BIN_BLOCKED;
      else if (H < T_free) target = BIN_FREE;
      else target = binStateArr[i];

      if (target == binStateArr[i]) pendingCount[i] = 0;
      else {
        if (pendingState[i] == target) pendingCount[i]++;
        else { pendingState[i] = target; pendingCount[i] = 1; }
        if (pendingCount[i] >= HYST_COUNT) { binStateArr[i] = target; pendingCount[i] = 0; }
      }
    }

    bool blockedFront = (binStateArr[0]==BIN_BLOCKED) || (binStateArr[1]==BIN_BLOCKED) || (binStateArr[2]==BIN_BLOCKED);

    DynamicJsonDocument doc(128);
    doc["type"] = "avoidance";
    if (blockedFront) {
      float left = max(latestUltrasonic[1], latestUltrasonic[3]);
      float right = max(latestUltrasonic[2], latestUltrasonic[4]);
      if (left > right) {
        doc["decision"] = "turn_left";
        motorLeft(150);
        currentMotion = "left";
        currentSpeed = 150;
      } else {
        doc["decision"] = "turn_right";
        motorRight(150);
        currentMotion = "right";
        currentSpeed = 150;
      }
    } else {
      doc["decision"] = "go_forward";
      rampMotorForward(255);
      currentMotion = "forward";
    }

    serializeJson(doc, Serial);
    Serial.println();
    sendJson(doc);

    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void commandTask(void *pvParameters) {
  esp_task_wdt_add(NULL);
  char buf[128];
  for (;;) {
    if (xQueueReceive(commandQueue, buf, pdMS_TO_TICKS(100)) == pdTRUE) {
      String msg = String(buf);
      Serial.printf("CommandTask got: %s\n", msg.c_str());
      StaticJsonDocument<256> j;
      DeserializationError err = deserializeJson(j, msg);
      if (!err) {
        if (j["cmd"].is<const char*>()) {
          String cmd = j["cmd"].as<const char*>();
          if (cmd == "stop") {
            motorStop(); currentMotion = "stopped"; currentSpeed = 0;
          } else if (cmd == "forward") {
            int sp = j["speed"] | 200; sp = constrain(sp, 0, 255);
            rampMotorForward((uint8_t)sp); currentMotion = "forward"; currentSpeed = sp;
          } else if (cmd == "back") {
            int sp = j["speed"] | 200; sp = constrain(sp, 0, 255);
            motorBackward((uint8_t)sp); currentMotion = "back"; currentSpeed = sp;
          } else if (cmd == "left") {
            int sp = j["speed"] | 200; sp = constrain(sp, 0, 255);
            motorLeft((uint8_t)sp); currentMotion = "left"; currentSpeed = sp;
          } else if (cmd == "right") {
            int sp = j["speed"] | 200; sp = constrain(sp, 0, 255);
            motorRight((uint8_t)sp); currentMotion = "right"; currentSpeed = sp;
          } else if (cmd == "goto") {
            navTargetLat = j["lat"] | 0.0; navTargetLng = j["lng"] | 0.0; navActive = true;
            Serial.printf("[NAV] New target: %.6f, %.6f\n", navTargetLat, navTargetLng);
            StaticJsonDocument<128> ack; ack["type"] = "nav"; ack["action"] = "goto"; ack["lat"] = navTargetLat; ack["lng"] = navTargetLng; sendJson(ack);
          } else if (cmd == "steer") {
            int angle = j["angle"] | 90; angle = constrain(angle, 0, 180); setServoAngle(angle); currentSteering = angle;
          } else {
            Serial.printf("Unknown cmd: %s\n", cmd.c_str());
          }
        } else {
          String s = msg; s.trim();
          if (s == "help") {
            DynamicJsonDocument doc(256); doc["type"] = "cmd_ack"; doc["msg"] = "help: stop/forward/back/left/right/goto/steer"; sendJson(doc);
          } else {
            Serial.printf("Unrecognized command text: %s\n", s.c_str());
          }
        }
      } else {
        String s = msg; s.trim();
        if (s == "stop") { motorStop(); currentMotion = "stopped"; currentSpeed = 0; }
        else Serial.printf("Non-json command received: %s\n", s.c_str());
      }
    }

    // NAV state machine
    if (navActive && millis() - lastNavStep > 500) {
      lastNavStep = millis();
      if (!gpsHasFix) {
        Serial.println("[NAV] Waiting for GPS fix...");
      } else {
        double d = TinyGPSPlus::distanceBetween(latestLat, latestLng, navTargetLat, navTargetLng);
        double courseTo = TinyGPSPlus::courseTo(latestLat, latestLng, navTargetLat, navTargetLng);
        double heading = gps.course.isValid() ? gps.course.deg() : 0;
        Serial.printf("[NAV] Dist=%.1fm CourseTo=%.1f Head=%.1f\n", d, courseTo, heading);
        if (d < 2.0) {
          Serial.println("[NAV] Arrived at target."); motorStop(); currentMotion = "stopped"; currentSpeed = 0; navActive = false;
        } else {
          double diff = courseTo - heading; if (diff > 180) diff -= 360; if (diff < -180) diff += 360;
          if (fabs(diff) < 20) { rampMotorForward(255); }
          else if (diff > 0) { setServoAngle(120); rampMotorForward(255); }
          else { setServoAngle(60); rampMotorForward(255); }
        }
      }
    }

    esp_task_wdt_reset();
  }
}

// ---------------- ACTUATORS ----------------
void rampMotorForward(uint8_t targetSpeed, uint8_t step, int stepDelay) {
  if (targetSpeed <= currentSpeed) { motorForward(targetSpeed); currentSpeed = targetSpeed; return; }
  for (int sp = currentSpeed; sp < targetSpeed; sp += step) {
    motorForward(sp);
    vTaskDelay(pdMS_TO_TICKS(stepDelay));
  }
  motorForward(targetSpeed);
  currentSpeed = targetSpeed;
}

void motorStop() {
  digitalWrite(MOTOR_IN1, LOW); digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, LOW); digitalWrite(MOTOR_IN4, LOW);
  ledcWrite(PWM_CH_EN1, 0);
  ledcWrite(PWM_CH_EN2, 0);
}

void motorForward(uint8_t speed) {
  digitalWrite(MOTOR_IN1, HIGH); digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, HIGH); digitalWrite(MOTOR_IN4, LOW);
  ledcWrite(PWM_CH_EN1, speed);
  ledcWrite(PWM_CH_EN2, speed);
}

void motorBackward(uint8_t speed) {
  digitalWrite(MOTOR_IN1, LOW); digitalWrite(MOTOR_IN2, HIGH);
  digitalWrite(MOTOR_IN3, LOW); digitalWrite(MOTOR_IN4, HIGH);
  ledcWrite(PWM_CH_EN1, speed);
  ledcWrite(PWM_CH_EN2, speed);
}

void motorLeft(uint8_t speed) {
  digitalWrite(MOTOR_IN1, LOW); digitalWrite(MOTOR_IN2, HIGH);
  digitalWrite(MOTOR_IN3, HIGH); digitalWrite(MOTOR_IN4, LOW);
  ledcWrite(PWM_CH_EN1, speed);
  ledcWrite(PWM_CH_EN2, speed);
}

void motorRight(uint8_t speed) {
  digitalWrite(MOTOR_IN1, HIGH); digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, LOW); digitalWrite(MOTOR_IN4, HIGH);
  ledcWrite(PWM_CH_EN1, speed);
  ledcWrite(PWM_CH_EN2, speed);
}

void setServoAngle(int angle) {
  uint32_t duty_min = (uint32_t)((65535.0 * 1.0) / 20.0);
  uint32_t duty_max = (uint32_t)((65535.0 * 2.0) / 20.0);
  uint32_t duty = duty_min + (uint32_t)((angle / 180.0) * (duty_max - duty_min));
  ledcWrite(PWM_CH_SERVO, duty);
}

// ---------------- ULTRASONIC HELPER ----------------
float readUltrasonicCm(int trigPin, int echoPin, int samples) {
  float sum = 0; int validSamples = 0;
  for (int i = 0; i < samples; i++) {
    digitalWrite(trigPin, LOW); delayMicroseconds(2);
    digitalWrite(trigPin, HIGH); delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    unsigned long duration = pulseIn(echoPin, HIGH, 30000);
    if (duration > 0) {
      float cm = (duration * 0.0343) / 2.0;
      sum += cm; validSamples++;
    }
    delay(10);
  }
  if (validSamples == 0) return 0.0;
  return sum / validSamples;
}

// ---------------- END ----------------
