#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

// ---------------- WiFi (Access Point mode) ----------------
const char* ssid     = "ArgusBot_AP";   // WiFi name
const char* password = "argusbot123";   // WiFi password (>=8 chars)

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ---------------- GPS ----------------
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

// ---------------- Gas Sensors ----------------
const int MQ135_PIN = 34;
const int MQ9_PIN   = 35;

// ---------------- Helpers ----------------
void notifyClients(String msg) {
  ws.textAll(msg);
}

void handleGPS() {
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isUpdated()) {
        String json = "{\"type\":\"gps\",\"lat\":" + String(gps.location.lat(), 6) +
                      ",\"lng\":" + String(gps.location.lng(), 6) + "}";
        notifyClients(json);
      }
    }
  }
}

void handleGas() {
  int mq135 = analogRead(MQ135_PIN);
  int mq9   = analogRead(MQ9_PIN);

  String json = "{\"type\":\"gas\",\"mq135\":" + String(mq135) +
                ",\"mq9\":" + String(mq9) + "}";
  notifyClients(json);
}

// ---------------- WebSocket ----------------
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
             AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("Client %u connected\n", client->id());
    notifyClients("{\"type\":\"log\",\"msg\":\"Client connected\"}");
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("Client %u disconnected\n", client->id());
  }
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17); // RX=16, TX=17

  // Start AP
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  Serial.println("Access Point started!");
  Serial.print("SSID: ");
  Serial.println(ssid);
  Serial.print("Password: ");
  Serial.println(password);
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());

  ws.onEvent(onEvent);
  server.addHandler(&ws);
  server.begin();

  notifyClients("{\"type\":\"log\",\"msg\":\"ESP32 started in AP mode\"}");
}

// ---------------- Loop ----------------
void loop() {
  handleGPS();

  static unsigned long lastGas = 0;
  if (millis() - lastGas > 2000) { // every 2s
    handleGas();
    lastGas = millis();
  }
}
