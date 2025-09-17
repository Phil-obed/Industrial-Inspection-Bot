/* Full integrated RTOS sketch for Argus Bot (patched)
   - Ultrasonics (5)
   - Gas MQ9 / MQ135
   - GPS (TinyGPS++)
   - Thermal MLX90640 (summary)
   - Motor control (2 motors + servo steering)
   - Simple VFH-like avoidance
   - WebSocket JSON publish / command receive
*/

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include "esp_task_wdt.h"   // Watchdog
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

// --- GAS SENSORS ---
#define MQ135_PIN 33
#define MQ9_PIN   25

// --- GPS (UART2) ---
#define GPS_RX 16   // GPS TX -> ESP RX2
#define GPS_TX 17   // GPS RX <- ESP TX2 (optional)

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

// helper prototypes
float readUltrasonicCm(int trigPin, int echoPin);
void motorStop();
void motorForward(uint8_t speed);
void motorBackward(uint8_t speed);
void motorLeft(uint8_t speed);
void motorRight(uint8_t speed);
void setServoAngle(int angle);
void sendJson(JsonDocument &doc);
void handleCommandJson(const String &msg);

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
  pinMode(TRIG1, OUTPUT); digitalWrite(TRIG1, LOW);
  pinMode(TRIG2, OUTPUT); digitalWrite(TRIG2, LOW);
  pinMode(TRIG3, OUTPUT); digitalWrite(TRIG3, LOW);
  pinMode(TRIG4, OUTPUT); digitalWrite(TRIG4, LOW);
  pinMode(TRIG5, OUTPUT); digitalWrite(TRIG5, LOW);

  pinMode(ECHO1, INPUT);
  pinMode(ECHO2, INPUT);
  pinMode(ECHO3, INPUT);
  pinMode(ECHO4, INPUT);
  pinMode(ECHO5, INPUT);

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
  setupWiFi();
  setupWebSocket();

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

// Ultrasonic task: measure all 5 sensors and broadcast
void ultrasonicTask(void *pvParameters) {
  esp_task_wdt_add(NULL);
  for (;;) {
    latestUltrasonic[0] = readUltrasonicCm(TRIG1, ECHO1);
    latestUltrasonic[1] = readUltrasonicCm(TRIG2, ECHO2);
    latestUltrasonic[2] = readUltrasonicCm(TRIG3, ECHO3);
    latestUltrasonic[3] = readUltrasonicCm(TRIG4, ECHO4);
    latestUltrasonic[4] = readUltrasonicCm(TRIG5, ECHO5);

    // Build JSON
    StaticJsonDocument<256> doc;
    doc["type"] = "ultrasonic";
    JsonArray arr = doc.createNestedArray("dist");
    for (int i=0;i<5;i++) arr.add(latestUltrasonic[i]);

    sendJson(doc);

    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(200)); // 5 Hz
  }
}

// Gas sensors: read ADCs and broadcast
void gasTask(void *pvParameters) {
  esp_task_wdt_add(NULL);
  const float ADC_REF = 3.3;
  const int ADC_MAX = 4095;
  // if you have voltage divider specifics, apply here
  for (;;) {
    int raw135 = analogRead(MQ135_PIN);
    int raw9 = analogRead(MQ9_PIN);

    // simple scale 0..ADC_MAX -> 0..100%
    latestGasMQ135 = (raw135 / (float)ADC_MAX) * 100.0;
    latestGasMQ9 = (raw9 / (float)ADC_MAX) * 100.0;

    StaticJsonDocument<128> doc;
    doc["type"] = "gas";
    doc["mq135_pct"] = latestGasMQ135;
    doc["mq9_pct"] = latestGasMQ9;

    sendJson(doc);

    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(1000)); // 1 Hz
  }
}

// GPS task: parse NMEA and broadcast when fix/updated
void gpsTask(void *pvParameters) {
  esp_task_wdt_add(NULL);
  for (;;) {
    while (GPSserial.available()) {
      gps.encode(GPSserial.read());
    }

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
    } else {
      // optional: send no-fix message occasionally
      StaticJsonDocument<64> doc;
      doc["type"] = "gps";
      doc["fix"] = false;
      sendJson(doc);
    }

    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// Thermal task: read MLX frame and broadcast min/avg/max
void thermalTask(void *pvParameters) {
  esp_task_wdt_add(NULL);
  float frame[32*24];
  for (;;) {
    // Removed repeated mlx.begin() here — initialization already done in setup()
    if (mlx.getFrame(frame) == 0) {
      float sum = 0;
      float tmin = frame[0];
      float tmax = frame[0];
      for (int i=0;i<32*24;i++) {
        float v = frame[i];
        sum += v;
        if (v < tmin) tmin = v;
        if (v > tmax) tmax = v;
      }
      float tavg = sum / (32.0*24.0);
      latestThermalMin = tmin;
      latestThermalMax = tmax;
      latestThermalAvg = tavg;

      StaticJsonDocument<192> doc;
      doc["type"] = "thermal";
      doc["min"] = tmin;
      doc["avg"] = tavg;
      doc["max"] = tmax;
      sendJson(doc);
    }
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(500)); // 2 Hz
  }
}

// Motor task: accept direct commands through commandTask (commandTask will modify these globals),
// and publish motor status periodically
volatile uint8_t currentSpeed = 0;
volatile String currentMotion = "stopped";
volatile int currentSteering = 90; // degrees: 0..180

void motorTask(void *pvParameters) {
  esp_task_wdt_add(NULL);

  // ensure PWM channels already setup in setup()
  for (;;) {
    // publish motor state periodically
    StaticJsonDocument<128> doc;
    doc["type"] = "motor";
    doc["status"] = currentMotion;
    doc["speed"] = currentSpeed;
    doc["steering_deg"] = currentSteering;
    sendJson(doc);

    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// Avoidance Task: simple VFH-like binning + decision
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
    // Create density H = 1/(d^2) per sensor
    for (int i=0;i<5;i++) {
      float d = latestUltrasonic[i];
      if (d <= 0) continue;
      if (d > 300) d = 300;
      float H = 1.0f / (d * d);
      BinState target;
      if (H > T_block) target = BIN_BLOCKED;
      else if (H < T_free) target = BIN_FREE;
      else target = binStateArr[i];

      if (target == binStateArr[i]) {
        pendingCount[i] = 0;
      } else {
        if (pendingState[i] == target) pendingCount[i]++;
        else { pendingState[i] = target; pendingCount[i] = 1; }
        if (pendingCount[i] >= HYST_COUNT) {
          binStateArr[i] = target;
          pendingCount[i] = 0;
        }
      }
    }

    // Simple decision: if any of front 3 bins blocked -> stop & turn away
    bool blockedFront = (binStateArr[0]==BIN_BLOCKED) || (binStateArr[1]==BIN_BLOCKED) || (binStateArr[2]==BIN_BLOCKED);
    StaticJsonDocument<128> doc;
    doc["type"] = "avoidance";
    if (blockedFront) {
      // choose side with more free space (use max of respective sensors to bias toward freer side)
      float left = max(latestUltrasonic[1], latestUltrasonic[3]);
      float right = max(latestUltrasonic[2], latestUltrasonic[4]);
      if (left > right) {
        // turn left
        doc["decision"] = "turn_left";
        motorLeft(150); // steer/move small turn
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
      motorForward(190);
      currentMotion = "forward";
      currentSpeed = 190;
    }

    sendJson(doc);

    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

// Command Task: pull commands from queue and execute
void commandTask(void *pvParameters) {
  esp_task_wdt_add(NULL);
  char buf[128];
  for (;;) {
    // NOTE: pass 'buf' (pointer) not '&buf'
    if (xQueueReceive(commandQueue, buf, pdMS_TO_TICKS(100)) == pdTRUE) {
      String msg = String(buf);
      Serial.printf("CommandTask got: %s\n", msg.c_str());
      // attempt parse as JSON
      StaticJsonDocument<256> j;
      DeserializationError err = deserializeJson(j, msg);
      if (!err) {
        if (j.containsKey("cmd")) {
          String cmd = j["cmd"].as<String>();
          if (cmd == "stop") {
            motorStop();
            currentMotion = "stopped";
            currentSpeed = 0;
          } else if (cmd == "forward") {
            int sp = j["speed"] | 200;
            sp = constrain(sp, 0, 255);
            motorForward((uint8_t)sp);
            currentMotion = "forward";
            currentSpeed = sp;
          } else if (cmd == "back") {
            int sp = j["speed"] | 200;
            sp = constrain(sp, 0, 255);
            motorBackward((uint8_t)sp);
            currentMotion = "back";
            currentSpeed = sp;
          } else if (cmd == "left") {
            int sp = j["speed"] | 200;
            sp = constrain(sp, 0, 255);
            motorLeft((uint8_t)sp);
            currentMotion = "left";
            currentSpeed = sp;
          } else if (cmd == "right") {
            int sp = j["speed"] | 200;
            sp = constrain(sp, 0, 255);
            motorRight((uint8_t)sp);
            currentMotion = "right";
            currentSpeed = sp;
          } else if (cmd == "goto") {
            // { "cmd":"goto", "lat":..., "lng":... }
            double lat = j["lat"] | 0.0;
            double lng = j["lng"] | 0.0;
            // For now: just publish a route ack to dashboard
            StaticJsonDocument<128> doc;
            doc["type"] = "nav";
            doc["action"] = "goto";
            doc["lat"] = lat;
            doc["lng"] = lng;
            sendJson(doc);
            // navigation control should be added (path planning + follow)
          } else if (cmd == "steer") {
            int angle = j["angle"] | 90;
            angle = constrain(angle, 0, 180);
            setServoAngle(angle);
            currentSteering = angle;
          } else {
            Serial.printf("Unknown cmd: %s\n", cmd.c_str());
          }
        } else {
          // not JSON? maybe plain text commands
          String s = msg;
          s.trim();
          if (s == "help") {
            // respond with help
            StaticJsonDocument<256> doc;
            doc["type"] = "cmd_ack";
            doc["msg"] = "help: available commands stop/forward/back/left/right/goto/steer";
            sendJson(doc);
          } else {
            Serial.printf("Unrecognized command text: %s\n", s.c_str());
          }
        }
      } else {
        // not JSON, treat as simple text
        String s = msg;
        s.trim();
        if (s == "stop") {
          motorStop(); currentMotion = "stopped"; currentSpeed = 0;
        } else {
          Serial.printf("Non-json command received: %s\n", s.c_str());
        }
      }
    } // if received
    esp_task_wdt_reset();
  }
}

// ---------------- ACTUATORS ----------------
void motorStop() {
  digitalWrite(MOTOR_IN1, LOW); digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, LOW); digitalWrite(MOTOR_IN4, LOW);
  ledcWrite(PWM_CH_EN1, 0);
  ledcWrite(PWM_CH_EN2, 0);
}

void motorForward(uint8_t speed) {
  // Motor A forward
  digitalWrite(MOTOR_IN1, HIGH); digitalWrite(MOTOR_IN2, LOW);
  // Motor B forward
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
  // left turn: left motor backward, right motor forward (skid-ish small)
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

// Servo: map 0..180 to 1ms..2ms at 50Hz
void setServoAngle(int angle) {
  // resolution 16-bit, range 0..65535
  // duty for 50Hz: 1ms = 0.05 fraction of 20ms
  uint32_t duty_min = (uint32_t)((65535.0 * 1.0) / 20.0); // ~3276
  uint32_t duty_max = (uint32_t)((65535.0 * 2.0) / 20.0); // ~6553
  uint32_t duty = duty_min + (uint32_t)((angle / 180.0) * (duty_max - duty_min));
  ledcWrite(PWM_CH_SERVO, duty);
}

// ---------------- ULTRASONIC HELPER ----------------
float readUltrasonicCm(int trigPin, int echoPin) {
  // trigger 10 us pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, 30000); // 30 ms timeout
  if (duration == 0) return 400.0; // no echo
  float cm = (duration * 0.0343) / 2.0;
  return cm;
}

// ---------------- COMMUNICATION ----------------
void setupWiFi() {
  WiFi.softAP("Bot_AP", "12345678");
  Serial.printf("AP IP: %s\n", WiFi.softAPIP().toString().c_str());
}

void setupWebSocket() {
  ws.onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client,
                AwsEventType type, void *arg, uint8_t *data, size_t len){
    if (type == WS_EVT_CONNECT) {
      Serial.printf("WS client %u connected\n", client->id());
      // send initial status snapshot
      StaticJsonDocument<256> doc;
      doc["type"] = "status";
      doc["msg"] = "welcome";
      String out;
      serializeJson(doc, out);
      server->textAll(out);
    } else if (type == WS_EVT_DISCONNECT) {
      Serial.printf("WS client %u disconnected\n", client->id());
    } else if (type == WS_EVT_DATA) {
      // data may be in 'data' buffer with length 'len'
      String msg = String((char*)data).substring(0, len);
      Serial.printf("From dash: %s\n", msg.c_str());
      // push to command queue (copy into buffer)
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
}
