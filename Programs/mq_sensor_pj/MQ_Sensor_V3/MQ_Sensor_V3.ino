#define MQ135_PIN 34   
#define MQ9_PIN   35   

// Divider values
const float R1 = 10000.0;   // 10k
const float R2 = 15000.0;   // 15k
const float DIVIDER_FACTOR = (R1 + R2) / R2;  // = 1.667

// ADC settings
const float ADC_REF = 3.3;    // ESP32 ADC reference voltage
const int ADC_MAX = 4095;     // 12-bit ADC

// Max sensor voltage (sensor powered by 5V)
const float VSENSOR_MAX = 5.0;

float readVoltage(int pin) {
  int raw = analogRead(pin);
  float v_adc = (raw / (float)ADC_MAX) * ADC_REF;
  float v_sensor = v_adc * DIVIDER_FACTOR;
  return v_sensor;
}

void printBar(float percent) {
  int bars = percent / 5; // 20 chars max
  Serial.print("[");
  for (int i = 0; i < 20; i++) {
    if (i < bars) Serial.print("█"); // filled
    else Serial.print(" ");
  }
  Serial.print("] ");
  Serial.print(percent, 1);
  Serial.println("%");
}

void setup() {
  Serial.begin(115200);
  delay(500);

  analogSetWidth(12); 
  analogSetPinAttenuation(MQ135_PIN, ADC_11db);
  analogSetPinAttenuation(MQ9_PIN,   ADC_11db);

  Serial.println("MQ sensors online...");
}

void loop() {
  float v135 = readVoltage(MQ135_PIN);
  float v9   = readVoltage(MQ9_PIN);

  float p135 = constrain((v135 / VSENSOR_MAX) * 100.0, 0, 100);
  float p9   = constrain((v9   / VSENSOR_MAX) * 100.0, 0, 100);

  Serial.print("MQ135: ");
  printBar(p135);

  Serial.print("MQ9:   ");
  printBar(p9);

  Serial.println("---------------------------------------");
  delay(1000);
}
