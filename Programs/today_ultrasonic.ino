// Hybrid median(3) + EMA filter for 5 ultrasonic sensors (ESP32 + Arduino-style)
#define NUM_SENSORS 2
// index mapping: 0..2 = front-left, front-mid, front-right, 3 = left, 4 = right
const int TRIG_PINS[NUM_SENSORS] = {19};
const int ECHO_PINS[NUM_SENSORS] = {21};

const unsigned long PULSE_TIMEOUT_US = 30000UL; // 30 ms timeout -> ~5 m
const unsigned long SAMPLE_INTERVAL_MS = 60; // ~16.7 Hz

// Median buffers (size 3)
float medBuf[NUM_SENSORS][3] = {0};
int medIdx[NUM_SENSORS] = {0};

// EMA state
float emaVal[NUM_SENSORS] = {0};
const float alphaDefault = 0.20; // tune per-sensor if needed
float alpha[NUM_SENSORS];

// Helpers
float median3(float a, float b, float c) {
  // manual median for 3 values (faster, no sort)
  if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
  if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
  return c;
}

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < NUM_SENSORS; ++i) {
    pinMode(TRIG_PINS[i], OUTPUT);
    pinMode(ECHO_PINS[i], INPUT);
    digitalWrite(TRIG_PINS[i], LOW);
    // initialize filter buffers with a reasonable large value (e.g., 300 cm) so EMA doesn't start at 0
    for (int j = 0; j < 3; ++j) medBuf[i][j] = 300.0;
    emaVal[i] = 300.0;
    alpha[i] = alphaDefault;
  }
  delay(10);
}

float readUltrasonicRaw(int trigPin, int echoPin) {
  // Trigger 10us pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure pulse width in microseconds
  unsigned long dur = pulseIn(echoPin, HIGH, PULSE_TIMEOUT_US);
  if (dur == 0) { // timeout: treat as max range (or NaN if you prefer)
    return 500.0; // e.g., 500 cm sentinel for "no echo"
  }
  // convert to cm: speed of sound ~0.0343 cm/us, divide by 2 for round trip
  float dist_cm = (dur * 0.0343f) / 2.0f;
  return dist_cm;
}

void loop() {
  static unsigned long lastSample = 0;
  if (millis() - lastSample < SAMPLE_INTERVAL_MS) return;
  lastSample = millis();

  float raw[NUM_SENSORS];
  float med[NUM_SENSORS];
  float out[NUM_SENSORS];

  // Read sensors one at a time (reduces cross-talk)
  for (int i = 0; i < NUM_SENSORS; ++i) {
    raw[i] = readUltrasonicRaw(TRIG_PINS[i], ECHO_PINS[i]);

    // median(3) buffer update
    medBuf[i][medIdx[i]] = raw[i];
    medIdx[i] = (medIdx[i] + 1) % 3;
    med[i] = median3(medBuf[i][0], medBuf[i][1], medBuf[i][2]);

    // EMA
    emaVal[i] = alpha[i] * med[i] + (1 - alpha[i]) * emaVal[i];
    out[i] = emaVal[i];
  }

  // Simple fusion: for going forward, take the minimum of the 3 front sensors
  float frontMin = min(out[0], min(out[1], out[2]));
  float left   = out[3];
  float right  = out[4];

  // Print results
  // Serial.print("Raw: ");
  // for (int i = 0; i < NUM_SENSORS; ++i) { Serial.print(raw[i]); Serial.print(i==NUM_SENSORS-1? "":" , "); }
  // Serial.print(" | Filtered: ");
  // for (int i = 0; i < NUM_SENSORS; ++i) { Serial.print(out[i]); Serial.print(i==NUM_SENSORS-1? "":" , "); }
  // Serial.print(" | FrontMin: "); Serial.print(frontMin);
  // Serial.print(" | L: "); Serial.print(left); Serial.print(" R: "); Serial.println(right);

  // Print only filtered sensor values (comma-separated)
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(out[i], 2); // 2 decimals
    if (i < NUM_SENSORS - 1) Serial.print(",");
  }
  Serial.println();

  // Example avoidance logic (very simple)
  const float STOP_DIST_CM = 25.0; // stop threshold
  if (frontMin < STOP_DIST_CM) {
    Serial.println("OBSTACLE: STOP or TURN");
    // insert robot control commands here
  }

  // small delay to let sensors settle before next loop if desired
}

