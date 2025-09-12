#define MQ135_PIN 34   // ADC1 channel
#define MQ9_PIN   35   // ADC1 channel

// Divider values
const float R1 = 1000.0;   // 1k
const float R2 = 2000.0;   // 2k
const float DIVIDER_FACTOR = (R1 + R2) / R2;  // = 1.5

// ADC settings
const float ADC_REF = 3.3;    // ESP32 ADC reference voltage
const int ADC_MAX = 4095;     // 12-bit ADC

float readVoltage(int pin) {
  int raw = analogRead(pin);                      // raw ADC value
  float v_adc = (raw / (float)ADC_MAX) * ADC_REF; // voltage at ESP32 pin
  float v_sensor = v_adc * DIVIDER_FACTOR;        // recovered actual sensor output
  Serial.print("Raw ADC: "); Serial.print(raw);
  Serial.print(" | Vadc: "); Serial.print(v_adc, 3);
  Serial.print(" V | Vsensor: "); Serial.print(v_sensor, 3);
  Serial.println(" V");
  return v_sensor;
}

void setup() {
  Serial.begin(115200);
  delay(500);

  analogSetWidth(12); // 0–4095 range
  analogSetPinAttenuation(MQ135_PIN, ADC_11db);
  analogSetPinAttenuation(MQ9_PIN,   ADC_11db);

  Serial.println("MQ sensors divider test starting...");
}

void loop() {
  Serial.print("MQ135 -> ");
  readVoltage(MQ135_PIN);

  Serial.print("MQ9   -> ");
  readVoltage(MQ9_PIN);

  Serial.println("----------------------------");
  delay(1000);
}