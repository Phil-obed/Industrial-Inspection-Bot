#include <ESP32Servo.h>

// --- Motor driver pins ---
#define EN1 26
#define EN2 27
#define IN1 12
#define IN2 13
#define IN3 14
#define IN4 15
#define SERVO_PIN 2   // ✅ servo steering pin

// --- Ultrasonic sensor pins ---
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

Servo myservo;

// PWM settings for ESP32
const int PWM_CHANNEL_EN1 = 0;
const int PWM_CHANNEL_EN2 = 1;
const int PWM_FREQ = 5000;       // Hz
const int PWM_RESOLUTION = 8;    // bits (0–255)

// --- VFH+ parameters ---
const int HIST_SECTORS = 37;           // ~5° resolution
const float SECTOR_ANGLE = 180.0 / (HIST_SECTORS - 1);
const float SENSOR_ANGLES[5] = {75.0, 90.0, 105.0, 45.0, 135.0};
// sensors order: FL, FC, FR, SL, SR
const int ULTRAS_TRIG[5] = {TRIG1, TRIG2, TRIG3, TRIG4, TRIG5};
const int ULTRAS_ECHO[5] = {ECHO1, ECHO2, ECHO3, ECHO4, ECHO5};

const float MAX_SENSOR_RANGE_CM = 250.0;
const float OBSTACLE_THRESHOLD_CM = 40.0;
const float SECTOR_INFLUENCE_WIDTH_DEG = 20.0;
const int CLEARANCE_REQUIRED_SECTORS = 3;

const int TARGET_DIR = 90;  // straight ahead
int prevChosenAngle = 90;

// --- Speed control ---
const int MAX_SPEED = 220;
const int CRUISE_SPEED = 200;
const int SLOW_SPEED = 120;
const int REVERSE_SPEED = 170;

// --- Timing ---
unsigned long lastUltrasonicMillis = 0;
const unsigned long ULTRASONIC_PERIOD_MS = 120;

// Function prototypes
void stopMotors();
void driveForward(int speed, int steeringAngle);
void driveBackward(int speed, int steeringAngle);
float readUltrasonicCM(int trigPin, int echoPin);
void computeVFHAndSteer(const float distances[5]);
void markHistogramFromSensors(const float distances[5], float histogram[]);
void smoothHistogram(float hist[], int sectors);
int pickBestCandidate(const float histogram[], int sectors);

void setup() {
  Serial.begin(115200);

  // --- Motor setup ---
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  ledcSetup(PWM_CHANNEL_EN1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_EN2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(EN1, PWM_CHANNEL_EN1);
  ledcAttachPin(EN2, PWM_CHANNEL_EN2);

  // --- Servo setup ---
  myservo.attach(SERVO_PIN);

  // --- Ultrasonic setup ---
  for (int i = 0; i < 5; ++i) {
    pinMode(ULTRAS_TRIG[i], OUTPUT);
    pinMode(ULTRAS_ECHO[i], INPUT);
    digitalWrite(ULTRAS_TRIG[i], LOW);
  }

  stopMotors();
  myservo.write(90); // center steering
  prevChosenAngle = 90;

  delay(200);
  Serial.println("VFH+ obstacle avoidance started (all pins set).");
}

void loop() {
  static float distances[5] = {
    MAX_SENSOR_RANGE_CM,
    MAX_SENSOR_RANGE_CM,
    MAX_SENSOR_RANGE_CM,
    MAX_SENSOR_RANGE_CM,
    MAX_SENSOR_RANGE_CM
  };

  if (millis() - lastUltrasonicMillis >= ULTRASONIC_PERIOD_MS) {
    lastUltrasonicMillis = millis();
    for (int i = 0; i < 5; ++i) {
      distances[i] = readUltrasonicCM(ULTRAS_TRIG[i], ULTRAS_ECHO[i]);
      if (distances[i] <= 0) distances[i] = MAX_SENSOR_RANGE_CM;
    }
    Serial.print("Distances (cm): ");
    for (int i = 0; i < 5; ++i) {
      Serial.print(distances[i], 1);
      if (i < 4) Serial.print(", ");
    }
    Serial.println();
  }

  computeVFHAndSteer(distances);
  delay(10);
}

// --- Motor control functions ---
void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  ledcWrite(PWM_CHANNEL_EN1, 0);
  ledcWrite(PWM_CHANNEL_EN2, 0);
}

void driveForward(int speed, int steeringAngle) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  ledcWrite(PWM_CHANNEL_EN1, constrain(speed, 0, 255));
  ledcWrite(PWM_CHANNEL_EN2, constrain(speed, 0, 255));
  myservo.write(constrain(steeringAngle, 0, 180));
  prevChosenAngle = constrain(steeringAngle, 0, 180);
}

void driveBackward(int speed, int steeringAngle) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  ledcWrite(PWM_CHANNEL_EN1, constrain(speed, 0, 255));
  ledcWrite(PWM_CHANNEL_EN2, constrain(speed, 0, 255));
  myservo.write(constrain(steeringAngle, 0, 180));
  prevChosenAngle = constrain(steeringAngle, 0, 180);
}

// --- Ultrasonic function ---
float readUltrasonicCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, 30000UL);
  if (duration == 0) return MAX_SENSOR_RANGE_CM;
  float dist_cm = duration * 0.01715;
  if (dist_cm > MAX_SENSOR_RANGE_CM) dist_cm = MAX_SENSOR_RANGE_CM;
  return dist_cm;
}

// --- VFH+ Implementation ---
void computeVFHAndSteer(const float distances[5]) {
  float histogram[HIST_SECTORS];
  for (int i = 0; i < HIST_SECTORS; i++) histogram[i] = 0.0;

  markHistogramFromSensors(distances, histogram);
  smoothHistogram(histogram, HIST_SECTORS);

  int chosenAngle = pickBestCandidate(histogram, HIST_SECTORS);

  if (chosenAngle < 0) {
    Serial.println("No path, reversing...");
    driveBackward(REVERSE_SPEED, prevChosenAngle);
  } else {
    Serial.print("Chosen angle: ");
    Serial.println(chosenAngle);
    driveForward(CRUISE_SPEED, chosenAngle);
  }
}

void markHistogramFromSensors(const float distances[5], float histogram[]) {
  for (int s = 0; s < 5; s++) {
    float d = distances[s];
    float angle = SENSOR_ANGLES[s];

    if (d < OBSTACLE_THRESHOLD_CM) {
      float weight = (OBSTACLE_THRESHOLD_CM - d) / OBSTACLE_THRESHOLD_CM;
      for (int i = 0; i < HIST_SECTORS; i++) {
        float sectorAngle = i * SECTOR_ANGLE;
        if (fabs(sectorAngle - angle) <= SECTOR_INFLUENCE_WIDTH_DEG) {
          histogram[i] += weight;
        }
      }
    }
  }
}

void smoothHistogram(float hist[], int sectors) {
  float temp[HIST_SECTORS];
  for (int i = 0; i < sectors; i++) temp[i] = hist[i];

  for (int i = 1; i < sectors - 1; i++) {
    hist[i] = (temp[i - 1] + temp[i] + temp[i + 1]) / 3.0;
  }
}

int pickBestCandidate(const float histogram[], int sectors) {
  int bestIndex = -1;
  float bestScore = 1e9;

  for (int i = 0; i < sectors; i++) {
    if (histogram[i] < 0.5) {
      bool clear = true;
      for (int j = -CLEARANCE_REQUIRED_SECTORS; j <= CLEARANCE_REQUIRED_SECTORS; j++) {
        int idx = i + j;
        if (idx < 0 || idx >= sectors) continue;
        if (histogram[idx] >= 0.5) {
          clear = false;
          break;
        }
      }
      if (clear) {
        float sectorAngle = i * SECTOR_ANGLE;
        float score = fabs(sectorAngle - TARGET_DIR) + 0.5 * fabs(sectorAngle - prevChosenAngle);
        if (score < bestScore) {
          bestScore = score;
          bestIndex = i;
        }
      }
    }
  }

  if (bestIndex < 0) return -1;
  return bestIndex * SECTOR_ANGLE;
}
