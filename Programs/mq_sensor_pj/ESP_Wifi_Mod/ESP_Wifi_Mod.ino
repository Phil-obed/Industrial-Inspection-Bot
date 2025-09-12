/* ESP32 WebSocket Gas Monitor + Motor Control (AP mode)
   Uses: ESPAsyncWebServer, AsyncTCP, ArduinoJson
*/

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

// --- SENSOR PINS & DIVIDER ---
#define MQ135_PIN 34
#define MQ9_PIN   35

const float R1 = 10000.0;   // 10k
const float R2 = 15000.0;   // 15k
const float DIVIDER_FACTOR = (R1 + R2) / R2;  // 1.6667

const float ADC_REF = 3.3;
const int ADC_MAX = 4095;
const float VSENSOR_MAX = 5.0;

// --- MOTOR PINS (example) ---
const int M1_IN1 = 16;
const int M1_IN2 = 17;
const int M2_IN1 = 18;
const int M2_IN2 = 19;
// PWM channel numbers
const int PWM_CH_M1 = 0;
const int PWM_CH_M2 = 1;
const int PWM_FREQ = 20000; // 20 kHz
const int PWM_RES = 8;      // 8-bit resolution (0-255)

// --- WIFI AP CREDENTIALS ---
const char *apSSID = "ARGUS_BOT";
const char *apPASS = "12345678"; // set secure password

// --- SERVER & WS ---
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Timing
unsigned long lastSend = 0;
const unsigned long SEND_INTERVAL_MS = 1000;

// Utility: read averaged ADC
int readAveragedRaw(int pin, int samples = 20, int delayMs = 5) {
  long sum = 0;
  int valid = 0;
  for (int i=0; i<samples; ++i) {
    int v = analogRead(pin);
    if (v > 0 && v < ADC_MAX) { sum += v; valid++; }
    delay(delayMs);
  }
  if (valid == 0) return -1;
  return sum / valid;
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

// Motor helper: simple differential drive commands
void motorStop() {
  // stop motors
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
  // left turn by reversing left wheel
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

// broadcast JSON to all connected websockets
void broadcastSensorData(float p135, float p9) {
  StaticJsonDocument<200> doc;
  doc["mq135"] = p135;
  doc["mq9"] = p9;
  char buf[200];
  size_t n = serializeJson(doc, buf);
  ws.textAll(buf, n);
}

// handle incoming websocket messages (JSON)
void handleWSMessage(void *arg, uint8_t *data, size_t len) {
  // parse JSON command
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
  int speed = doc["speed"] | 200; // default speed (0-255)
  if (sCmd == "stop") {
    motorStop();
    Serial.println("CMD: stop");
  } else if (sCmd == "forward") {
    motorForward(constrain(speed, 0, 255));
    Serial.println("CMD: forward");
  } else if (sCmd == "back") {
    motorBackward(constrain(speed, 0, 255));
    Serial.println("CMD: back");
  } else if (sCmd == "left") {
    motorLeft(constrain(speed, 0, 255));
    Serial.println("CMD: left");
  } else if (sCmd == "right") {
    motorRight(constrain(speed, 0, 255));
    Serial.println("CMD: right");
  } else {
    Serial.print("Unknown cmd: "); Serial.println(sCmd);
  }
}

// WS event handler (wrapper)
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


// ---------- minified front-end page (see below) ----------
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Robot Gas Monitor</title>
<style>
body{font-family:Arial;text-align:center;padding:10px} .bar{width:300px;height:20px;background:#ddd;margin:6px auto;border-radius:4px;overflow:hidden}
.fill{height:100%;background:linear-gradient(90deg,#ff5722,#b71c1c)} button{margin:6px;padding:10px 16px} .row{display:flex;justify-content:center;gap:8px}
</style></head><body>
<h2>Robot Gas Monitor (AP Mode)</h2>
<div>MQ135: <span id="val135">--</span></div><div class="bar"><div id="bar135" class="fill" style="width:0%"></div></div>
<div>MQ9: <span id="val9">--</span></div><div class="bar"><div id="bar9" class="fill" style="width:0%"></div></div>
<div class="row">
  <button onclick="sendCmd('forward')">Forward</button>
  <button onclick="sendCmd('back')">Back</button>
  <button onclick="sendCmd('left')">Left</button>
  <button onclick="sendCmd('right')">Right</button>
  <button onclick="sendCmd('stop')">Stop</button>
</div>
<div>
  Speed: <input id="speed" type="range" min="0" max="255" value="200" oninput="speedVal.innerText=this.value">
  <span id="speedVal">200</span>
</div>
<div id="status" style="margin-top:10px;color:green">Connecting...</div>
<script>
let ws;
function init() {
  const host = location.hostname;
  ws = new WebSocket('ws://' + host + '/ws');
  ws.onopen = () => { document.getElementById('status').innerText = 'WS: connected'; };
  ws.onclose = () => { document.getElementById('status').innerText = 'WS: disconnected'; setTimeout(init, 2000); };
  ws.onmessage = (evt) => {
    try {
      const data = JSON.parse(evt.data);
      if ('mq135' in data) {
        document.getElementById('val135').innerText = data.mq135.toFixed(1) + '%';
        document.getElementById('bar135').style.width = data.mq135 + '%';
      }
      if ('mq9' in data) {
        document.getElementById('val9').innerText = data.mq9.toFixed(1) + '%';
        document.getElementById('bar9').style.width = data.mq9 + '%';
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

  // Setup PWM channels
  ledcSetup(PWM_CH_M1, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_M2, PWM_FREQ, PWM_RES);
  ledcAttachPin(M1_IN1, PWM_CH_M1); // if using H-bridge with PWM pin on enable, adjust accordingly
  // NOTE: If your motor driver expects separate PWM pin, wire accordingly. For simplicity we write to channels for speed.

  // Start AP
  WiFi.softAP(apSSID, apPASS);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  // Webserver: serve frontend page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });

  // Attach websocket event handler
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.begin();
}

// Loop: send sensor JSON periodically
void loop() {
  ws.cleanupClients();

  unsigned long now = millis();
  if (now - lastSend >= SEND_INTERVAL_MS) {
    lastSend = now;

    int raw135 = readAveragedRaw(MQ135_PIN);
    int raw9 = readAveragedRaw(MQ9_PIN);
    float p135 = 0.0, p9 = 0.0;

    if (raw135 > 0) {
      float v_node = rawAdcToNodeVoltage(raw135);
      float v_sensor = recoverSensorVoltage(v_node);
      p135 = percentageFromVoltage(v_sensor);
    }
    if (raw9 > 0) {
      float v_node = rawAdcToNodeVoltage(raw9);
      float v_sensor = recoverSensorVoltage(v_node);
      p9 = percentageFromVoltage(v_sensor);
    }

    broadcastSensorData(p135, p9);
  }
}
