#define NUM_SENSORS 5
// Pin mapping: 0=front-left, 1=front-mid, 2=front-right, 3=left, 4=right
const int TRIG_PINS[NUM_SENSORS] = {5, 21, 22, 4, 2};
const int ECHO_PINS[NUM_SENSORS] = {18, 19, 23, 35, 34};

const unsigned long PULSE_TIMEOUT_US = 25000UL; // ~4.3m max range
const unsigned long SENSOR_DELAY_MS = 20; // Delay between sensor reads to reduce cross-talk
const unsigned long SAMPLE_INTERVAL_MS = 100; // ~10 Hz full cycle

// Filter parameters
const int MEDIAN_WINDOW = 3;
float medBuf[NUM_SENSORS][MEDIAN_WINDOW];
int medIdx[NUM_SENSORS] = {0};
float emaVal[NUM_SENSORS];
const float ALPHA = 0.3; // EMA smoothing factor (0.0 to 1.0)

// Helper: Simple bubble sort for median (small arrays)
float median(float arr[], int size) {
  float temp[size];
  for (int i = 0; i < size; i++) temp[i] = arr[i];
  for (int i = 0; i < size - 1; i++)
    for (int j = 0; j < size - i - 1; j++)
      if (temp[j] > temp[j + 1]) {
        float t = temp[j];
        temp[j] = temp[j + 1];
        temp[j + 1] = t;
      }
  return temp[size / 2];
}

float readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, PULSE_TIMEOUT_US);
  if (duration == 0) return 400.0; // Max range sentinel (~4m)
  return (duration * 0.0343) / 2.0; // Convert to cm
}

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(TRIG_PINS[i], OUTPUT);
    pinMode(ECHO_PINS[i], INPUT);
    digitalWrite(TRIG_PINS[i], LOW);
    for (int j = 0; j < MEDIAN_WINDOW; j++) medBuf[i][j] = 400.0; // Initialize buffers
    emaVal[i] = 400.0;
  }
  delay(10); // Initial settle
}

void loop() {
  static unsigned long lastSample = 0;
  if (millis() - lastSample < SAMPLE_INTERVAL_MS) return;
  lastSample = millis();

  float raw[NUM_SENSORS];
  float filtered[NUM_SENSORS];

  // Read sensors sequentially with delays to avoid cross-talk
  for (int i = 0; i < NUM_SENSORS; i++) {
    raw[i] = readUltrasonic(TRIG_PINS[i], ECHO_PINS[i]);

    // Update median buffer
    medBuf[i][medIdx[i]] = raw[i];
    medIdx[i] = (medIdx[i] + 1) % MEDIAN_WINDOW;

    // Calculate median
    float window[MEDIAN_WINDOW];
    for (int j = 0; j < MEDIAN_WINDOW; j++) window[j] = medBuf[i][j];
    float med = median(window, MEDIAN_WINDOW);

    // Apply EMA
    emaVal[i] = ALPHA * med + (1.0 - ALPHA) * emaVal[i];
    filtered[i] = emaVal[i];

    delay(SENSOR_DELAY_MS); // Reduce cross-talk
  }

  // Fusion: Minimum of front sensors
  float frontMin = min(filtered[0], min(filtered[1], filtered[2]));
  float left = filtered[3];
  float right = filtered[4];

  // Output results
  Serial.print("Raw: ");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(raw[i], 1);
    Serial.print(i == NUM_SENSORS - 1 ? "" : ", ");
  }
  Serial.print(" | Filtered: ");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(filtered[i], 1);
    Serial.print(i == NUM_SENSORS - 1 ? "" : ", ");
  }
  Serial.print(" | FrontMin: "); Serial.print(frontMin, 1);
  Serial.print(" | Left: "); Serial.print(left, 1);
  Serial.print(" | Right: "); Serial.println(right, 1);

  // Basic obstacle avoidance
  const float STOP_DIST_CM = 30.0;
  if (frontMin < STOP_DIST_CM) {
    Serial.println("STOP: Obstacle detected!");
    // Add robot control here
  }
}
