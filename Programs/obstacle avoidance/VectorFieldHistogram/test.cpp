// ===== VFH-Lite Bin Processing for ESP32 + 5 Ultrasonic Sensors =====
// Using float version with hysteresis (two thresholds)

#include<Arduino.h>
#define NUM_SENSORS 5

// Pin definitions (change to your wiring)
int trigPins[NUM_SENSORS] = {5, 19, 22, 2, 4};  // Example pins
int echoPins[NUM_SENSORS] = {18, 21, 23, 34, 35};  // Example pins

// Safe distance settings (cm)
float d_block = 20.0;   // blocked if closer than this
float d_free  = 30.0;   // free if farther than this

// Thresholds
float T_block;  
float T_free;   

// Bin states
enum BinState { BIN_FREE = 0, BIN_BLOCKED = 1 };
BinState binState[NUM_SENSORS];

// For extra smoothing (N consecutive detections before switching)
const int HYST_COUNT = 2;  
int pendingCount[NUM_SENSORS] = {0};
BinState pendingState[NUM_SENSORS];

// Function to read distance from one ultrasonic sensor
float readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout
  if (duration == 0) return 999.0; // no echo = very far

  float distance = duration * 0.0343 / 2.0; // cm
  return distance;
}

// Update bin states using hysteresis
void updateBins(float dist[]) {
  for (int i=0; i<NUM_SENSORS; i++) {
    float d = dist[i];
    if (d <= 0) continue;  // invalid
    if (d > 300) d = 300;  // cap max range at 3m

    float H = 1.0f / (d * d); // obstacle density

    BinState target;
    if (H > T_block) target = BIN_BLOCKED;
    else if (H < T_free) target = BIN_FREE;
    else target = binState[i]; // stay the same if between thresholds

    if (target == binState[i]) {
      pendingCount[i] = 0; // no change
    } else {
      if (pendingState[i] == target) {
        pendingCount[i]++;
      } else {
        pendingState[i] = target;
        pendingCount[i] = 1;
      }

      if (pendingCount[i] >= HYST_COUNT) {
        binState[i] = target;
        pendingCount[i] = 0;
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  for (int i=0; i<NUM_SENSORS; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
    binState[i] = BIN_FREE;
    pendingState[i] = BIN_FREE;
  }

  // Precompute thresholds
  T_block = 1.0f / (d_block * d_block);
  T_free  = 1.0f / (d_free  * d_free);

  Serial.println("VFH Bin Processing Started...");
}

void loop() {
  float dist[NUM_SENSORS];

  // 1) Read all sensors
  for (int i=0; i<NUM_SENSORS; i++) {
    dist[i] = readUltrasonic(trigPins[i], echoPins[i]);
  }

  // 2) Update bins
  updateBins(dist);

  // 3) Print results
  Serial.print("Distances(cm): ");
  for (int i=0; i<NUM_SENSORS; i++) {
    Serial.print(dist[i]); Serial.print(" ");
  }

  Serial.print(" | States: ");
  for (int i=0; i<NUM_SENSORS; i++) {
    Serial.print(binState[i] == BIN_BLOCKED ? "X " : "O ");
  }
  Serial.println();

  delay(100); // 10 Hz update
}
