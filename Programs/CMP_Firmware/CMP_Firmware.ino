#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_MLX90640.h>
#include <TinyGPSPlus.h>

// ================== PIN MAPPING ==================
// Gas sensors (analog) - keep these on ADC1 pins
#define MQ135_PIN 33
#define MQ9_PIN   32

/*MLX90640 (I2C)
#define SDA_PIN 21
#define SCL_PIN 22
*/
// GPS (Hardware Serial 1) - GPS TX -> ESP32 RX (GPS_RX_PIN)
#define GPS_TX_PIN 16
static const uint32_t GPSBaud = 9600;

// Ultrasonic sensors (5)
const int TRIG_PINS[5] = {5, 19, 22, 4, 2};
const int ECHO_PINS[5] = {18, 21, 23, 35, 34}; // note: 36 & 39 are input-only (OK for echo)

// Motor pins (example) — adapt to your motor driver wiring
const int M1_IN1 = 27;
const int M2_IN2 = 14;
const int M3_IN3 = 12;
const int M4_IN4 = 13;

// PWM channels
const int EN1 = 25;
const int EN2 = 26;
const int PWM_FREQ = 20000;
const int PWM_RES = 8; // 0-255

// ================ CONSTANTS ================
const float R1 = 10000.0;   // 10k divider top
const float R2 = 15000.0;   // 15k divider bottom
const float DIVIDER_FACTOR = (R1 + R2) / R2; // 1.6667
const float ADC_REF = 3.3;
const int ADC_MAX = 4095;
const float VSENSOR_MAX = 5.0; // sensor supply

//This is where i left off
// Ultrasonic timing & filtering
const unsigned long PULSE_TIMEOUT_US = 25000UL;
const unsigned long SENSOR_DELAY_MS = 25;   // between ultrasonic pulses to avoid cross-talk
const unsigned long SAMPLE_INTERVAL_MS = 100; // ~10Hz cycle

const int MEDIAN_WINDOW = 3;
float medBuf[5][MEDIAN_WINDOW];
int medIdx[5] = {0};
float emaVal[5];
const float ALPHA = 0.3; // EMA factor

// MLX90640
Adafruit_MLX90640 mlx;
float mlxFrame[32*24];
const int MLX_READ_INTERVAL_MS = 500; // read thermal summary twice per second

// GPS
TinyGPSPlus gps;
HardwareSerial ss(1);

// WiFi AP credentials
const char *apSSID = "ARGUS_BOT";
const char *apPASS = "12345678";

// Web server + websocket
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

unsigned long lastSend = 0;
const unsigned long SEND_INTERVAL_MS = 1000; // broadcast every 1s

// helper prototypes
int readAveragedRawADC(int pin, int samples=20, int delayMs=5);
float rawAdcToNodeVoltage(int raw);
float recoverSensorVoltage(float v_node);
float percentageFromVoltage(float v);
void broadcastSensorData(JsonVariantConst data);
void handleWSMessage(void *arg, uint8_t *data, size_t len);
void onWsEvent(AsyncWebSocket *serverW, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len);

// -------------------------- utility functions --------------------------
int readAveragedRawADC(int pin, int samples, int delayMs) {
  long sum = 0;
  int valid = 0;
  for (int i=0;i<samples;i++) {
    int v = analogRead(pin);
    if (v > 0 && v < ADC_MAX) { sum += v; valid++; }
    delay(delayMs);
  }
  if (valid==0) return -1;
  return (int)(sum / valid);
}

float rawAdcToNodeVoltage(int raw) {
  return (raw / (float)ADC_MAX) * ADC_REF;
}
float recoverSensorVoltage(float v_node) {
  return v_node * DIVIDER_FACTOR;
}
float percentageFromVoltage(float v) {
  return constrain((v / VSENSOR_MAX) * 100.0, 0.0, 100.0);
}

// ---------- ultrasonic helpers ----------
float median_local(float arr[], int size) {
  float tmp[MEDIAN_WINDOW];
  for (int i=0;i<size;i++) tmp[i]=arr[i];
  // simple bubble sort
  for (int i=0;i<size-1;i++)
    for (int j=0;j<size-i-1;j++)
      if (tmp[j] > tmp[j+1]) { float t=tmp[j]; tmp[j]=tmp[j+1]; tmp[j+1]=t; }
  return tmp[size/2];
}

float readUltrasonicCm(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, PULSE_TIMEOUT_US);
  if (duration == 0) return 400.0; // sentinel for out-of-range (~4m)
  return (duration * 0.0343) / 2.0; // cm
}

// ---------- motor helpers ----------
void motorStop() {
  digitalWrite(M1_IN1, LOW); digitalWrite(M1_IN2, LOW);
  digitalWrite(M2_IN1, LOW); digitalWrite(M2_IN2, LOW);
  ledcWrite(PWM_CH_M1, 0);
  ledcWrite(PWM_CH_M2, 0);
}
void motorForward(int speed) {
  digitalWrite(M1_IN1, HIGH); digitalWrite(M1_IN2, LOW);
  digitalWrite(M2_IN1, HIGH); digitalWrite(M2_IN2, LOW);
  ledcWrite(PWM_CH_M1, speed);
  ledcWrite(PWM_CH_M2, speed);
}
void motorBackward(int speed) {
  digitalWrite(M1_IN1, LOW); digitalWrite(M1_IN2, HIGH);
  digitalWrite(M2_IN1, LOW); digitalWrite(M2_IN2, HIGH);
  ledcWrite(PWM_CH_M1, speed);
  ledcWrite(PWM_CH_M2, speed);
}
void motorLeft(int speed) {
  digitalWrite(M1_IN1, LOW); digitalWrite(M1_IN2, HIGH);
  digitalWrite(M2_IN1, HIGH); digitalWrite(M2_IN2, LOW);
  ledcWrite(PWM_CH_M1, speed);
  ledcWrite(PWM_CH_M2, speed);
}
void motorRight(int speed) {
  digitalWrite(M1_IN1, HIGH); digitalWrite(M1_IN2, LOW);
  digitalWrite(M2_IN1, LOW); digitalWrite(M2_IN2, HIGH);
  ledcWrite(PWM_CH_M1, speed);
  ledcWrite(PWM_CH_M2, speed);
}

// broadcast convenience
void broadcastSensorData(JsonVariantConst data) {
  StaticJsonDocument<512> doc;
  doc.set(data);
  char buf[512];
  size_t n = serializeJson(doc, buf);
  ws.textAll(buf, n);
}

// handle incoming websocket messages (JSON)
void handleWSMessage(void *arg, uint8_t *data, size_t len) {
  StaticJsonDocument<200> doc;
  DeserializationError err = deserializeJson(doc, data, len);
  if (err) {
    Serial.print("JSON parse error: ");
    Serial.println(err.f_str());
    return;
  }
  const char *cmd = doc["cmd"];
  if (!cmd) return;
  String sCmd = String(cmd);
  int speed = doc["speed"] | 200;
  if (sCmd == "stop") { motorStop(); Serial.println("CMD stop"); }
  else if (sCmd == "forward") { motorForward(constrain(speed,0,255)); Serial.println("CMD forward"); }
  else if (sCmd == "back") { motorBackward(constrain(speed,0,255)); Serial.println("CMD back"); }
  else if (sCmd == "left") { motorLeft(constrain(speed,0,255)); Serial.println("CMD left"); }
  else if (sCmd == "right") { motorRight(constrain(speed,0,255)); Serial.println("CMD right"); }
  else Serial.print("Unknown cmd: "), Serial.println(sCmd);
}

// WS event handler
void onWsEvent(AsyncWebSocket *serverW, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("WS client %u connected\n", client->id());
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("WS client %u disconnected\n", client->id());
  } else if (type == WS_EVT_DATA) {
    handleWSMessage(arg, data, len);
  }
}

// ----------------- Frontend HTML (served by ESP32) -----------------
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>ARGUS Bot Monitor</title>
<style>body{font-family:Arial;text-align:center;padding:10px} .bar{width:300px;height:18px;background:#ddd;margin:6px auto;border-radius:4px;overflow:hidden}.fill{height:100%;background:linear-gradient(90deg,#ff5722,#b71c1c)}button{margin:6px;padding:8px 12px}.row{display:flex;justify-content:center;gap:8px}</style>
</head><body>
<h2>ARGUS Bot Monitor (AP Mode)</h2>
<div>MQ135: <span id="val135">--</span></div><div class="bar"><div id="bar135" class="fill" style="width:0%"></div></div>
<div>MQ9: <span id="val9">--</span></div><div class="bar"><div id="bar9" class="fill" style="width:0%"></div></div>
<div>FrontMin: <span id="frontmin">--</span> cm | Left: <span id="left">--</span> cm | Right: <span id="right">--</span> cm</div>
<div>GPS: <span id="gps">no fix</span></div>
<div>Thermal (avg/min/max): <span id="tavg">--</span> / <span id="tmin">--</span> / <span id="tmax">--</span> °C</div>
<div class="row">
  <button onclick="sendCmd('forward')">Forward</button>
  <button onclick="sendCmd('back')">Back</button>
  <button onclick="sendCmd('left')">Left</button>
  <button onclick="sendCmd('right')">Right</button>
  <button onclick="sendCmd('stop')">Stop</button>
</div>
<div>Speed: <input id="speed" type="range" min="0" max="255" value="200" oninput="speedVal.innerText=this.value"><span id="speedVal">200</span></div>
<div id="status" style="margin-top:8px;color:green">Connecting...</div>
<script>
let ws;
function init() {
  ws = new WebSocket('ws://' + location.hostname + '/ws');
  ws.onopen = () => { document.getElementById('status').innerText = 'WS: connected'; };
  ws.onclose = () => { document.getElementById('status').innerText = 'WS: disconnected'; setTimeout(init,2000); };
  ws.onmessage = (evt) => {
    try {
      const data = JSON.parse(evt.data);
      if ('mq135' in data) { document.getElementById('val135').innerText = data.mq135.toFixed(1)+'%'; document.getElementById('bar135').style.width = data.mq135 + '%'; }
      if ('mq9' in data)  { document.getElementById('val9').innerText  = data.mq9.toFixed(1)+'%';  document.getElementById('bar9').style.width  = data.mq9 + '%'; }
      if ('ultrasonic' in data) {
        document.getElementById('frontmin').innerText = data.ultrasonic.frontMin.toFixed(1);
        document.getElementById('left').innerText = data.ultrasonic.left.toFixed(1);
        document.getElementById('right').innerText = data.ultrasonic.right.toFixed(1);
      }
      if ('gps' in data) {
        const g = data.gps;
        if (g.fix) document.getElementById('gps').innerText = g.lat.toFixed(6)+', '+g.lng.toFixed(6)+' (sats:'+g.sats+')';
        else document.getElementById('gps').innerText = 'NO FIX';
      }
      if ('thermal' in data) {
        document.getElementById('tavg').innerText = data.thermal.avg.toFixed(1);
        document.getElementById('tmin').innerText = data.thermal.min.toFixed(1);
        document.getElementById('tmax').innerText = data.thermal.max.toFixed(1);
      }
    } catch(e) { console.log('parse err', e); }
  };
}
function sendCmd(cmd) {
  const speed = parseInt(document.getElementById('speed').value);
  const msg = JSON.stringify({ cmd: cmd, speed: speed });
  if (ws && ws.readyState === WebSocket.OPEN) ws.send(msg);
}
window.onload = init;
</script></body></html>
)rawliteral";

// ------------------ setup & loop ------------------
void setup() {
  Serial.begin(115200);
  delay(100);

  // ADC config
  analogSetWidth(12);
  analogSetPinAttenuation(MQ135_PIN, ADC_11db);
  analogSetPinAttenuation(MQ9_PIN, ADC_11db);

  // Motor pins
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);

  // PWM
  ledcSetup(PWM_CH_M1, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_M2, PWM_FREQ, PWM_RES);
  // NOTE: attach PWM to the pins that control enable on your motor driver if available.
  ledcAttachPin(M1_IN1, PWM_CH_M1);
  ledcAttachPin(M2_IN1, PWM_CH_M2);

  motorStop();

  // Ultrasonic init
  for (int i=0;i<5;i++) {
    pinMode(TRIG_PINS[i], OUTPUT);
    pinMode(ECHO_PINS[i], INPUT);
    digitalWrite(TRIG_PINS[i], LOW);
    for (int j=0;j<MEDIAN_WINDOW;j++) medBuf[i][j] = 400.0;
    emaVal[i] = 400.0;
  }

  // MLX90640 init
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  if (!mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire)) {
    Serial.println("MLX90640 not found! Thermal disabled.");
  } else {
    Serial.println("MLX90640 OK");
    mlx.setRefreshRate(MLX90640_8_HZ);
  }

  // GPS init (Serial1) - RX pin defined above
  ss.begin(GPSBaud, SERIAL_8N1, GPS_RX_PIN, -1);
  Serial.println("GPS Serial initialized.");

  // Start AP and server
  WiFi.softAP(apSSID, apPASS);
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());

  // serve web UI
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.begin();
  Serial.println("Server started.");
}

unsigned long lastSample = 0;
unsigned long lastThermal = 0;

void loop() {
  // handle websocket housekeeping
  ws.cleanupClients();

  // read GPS NMEA bytes non-blocking
  while (ss.available() > 0) {
    gps.encode(ss.read());
  }

  unsigned long now = millis();

  // Ultrasonic + gas + broadcast every SAMPLE_INTERVAL_MS or SEND_INTERVAL_MS
  if (now - lastSample >= SAMPLE_INTERVAL_MS) {
    lastSample = now;

    // read and filter ultrasonic sensors sequentially
    float raw[5];
    float filtered[5];
    for (int i=0;i<5;i++) {
      raw[i] = readUltrasonicCm(TRIG_PINS[i], ECHO_PINS[i]);
      // update median buffer
      medBuf[i][medIdx[i]] = raw[i];
      medIdx[i] = (medIdx[i] + 1) % MEDIAN_WINDOW;
      float window[MEDIAN_WINDOW];
      for (int j=0;j<MEDIAN_WINDOW;j++) window[j] = medBuf[i][j];
      float med = median_local(window, MEDIAN_WINDOW);
      emaVal[i] = ALPHA * med + (1.0 - ALPHA) * emaVal[i];
      filtered[i] = emaVal[i];
      delay(SENSOR_DELAY_MS);
    }

    float frontMin = min(filtered[0], min(filtered[1], filtered[2]));
    float left = filtered[3];
    float right = filtered[4];

    // Gas sensors
    int raw135 = readAveragedRawADC(MQ135_PIN);
    int raw9 = readAveragedRawADC(MQ9_PIN);
    float p135 = 0.0, p9 = 0.0;
    if (raw135 > 0) { float v_node = rawAdcToNodeVoltage(raw135); float v_sensor = recoverSensorVoltage(v_node); p135 = percentageFromVoltage(v_sensor); }
    if (raw9 > 0)   { float v_node = rawAdcToNodeVoltage(raw9);   float v_sensor = recoverSensorVoltage(v_node);   p9   = percentageFromVoltage(v_sensor); }

    // GPS summary
    JsonObject gpsObj;
    StaticJsonDocument<512> doc; // temp doc to construct message
    JsonObject root = doc.to<JsonObject>();
    JsonObject gpsJ = root.createNestedObject("gps");
    if (gps.location.isValid()) {
      gpsJ["fix"] = true;
      gpsJ["lat"] = gps.location.lat();
      gpsJ["lng"] = gps.location.lng();
    } else {
      gpsJ["fix"] = false;
      gpsJ["lat"] = 0.0;
      gpsJ["lng"] = 0.0;
    }
    gpsJ["alt"] = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
    gpsJ["speed"] = gps.speed.isValid() ? gps.speed.kmph() : 0.0;
    gpsJ["sats"] = gps.satellites.isValid() ? gps.satellites.value() : 0;

    // Thermal summary (read less frequently)
    float tmin = 0, tmax = 0, tavg = 0;
    bool thermal_ok = false;
    if ((millis() - lastThermal) >= MLX_READ_INTERVAL_MS) {
      lastThermal = millis();
      if (mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire)) {
        // should already be initialized; but safe check
      }
      if (mlx.getFrame(mlxFrame) == 0) {
        // compute min/max/avg
        float sum = 0.0;
        tmin = mlxFrame[0]; tmax = mlxFrame[0];
        for (int i=0;i<32*24;i++) {
          float v = mlxFrame[i];
          sum += v;
          if (v < tmin) tmin = v;
          if (v > tmax) tmax = v;
        }
        tavg = sum / (32.0*24.0);
        thermal_ok = true;
      } else {
        thermal_ok = false;
      }
    }

    // Build JSON to broadcast
    root["mq135"] = p135;
    root["mq9"]   = p9;
    JsonObject ult = root.createNestedObject("ultrasonic");
    ult["frontMin"] = frontMin;
    ult["left"] = left;
    ult["right"] = right;

    JsonObject th = root.createNestedObject("thermal");
    if (thermal_ok) {
      th["avg"] = tavg;
      th["min"] = tmin;
      th["max"] = tmax;
    } else {
      th["avg"] = 0.0;
      th["min"] = 0.0;
      th["max"] = 0.0;
    }

    root["gps"] = gpsJ;

    // Broadcast
    char outBuf[1024];
    size_t len = serializeJson(root, outBuf);
    ws.textAll(outBuf, len);

    // Debug print to Serial
    Serial.printf("MQ135: %.1f%% MQ9: %.1f%% | FrontMin: %.1f cm | L: %.1f R: %.1f | Tavg: %.1f\n",
                  p135, p9, frontMin, left, right, (thermal_ok? tavg: 0.0));

    // Clear doc for next use
    doc.clear();
  }
}
