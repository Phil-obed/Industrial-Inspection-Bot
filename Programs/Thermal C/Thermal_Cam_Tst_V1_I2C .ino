#include <Wire.h>
#include <Adafruit_MLX90640.h>

#define SDA_PIN 21
#define SCL_PIN 22

Adafruit_MLX90640 mlx;
float frame[32 * 24];

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("MLX90640 Thermal Camera Test");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  delay(2000); // give sensor time to power up

  // Retry loop until sensor responds
  while (!mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire)) {
    Serial.println("MLX90640 not found, retrying...");
    delay(1000);
  }

  Serial.println("MLX90640 initialized successfully!");
  mlx.setRefreshRate(MLX90640_8_HZ);
}

void loop() {
  if (mlx.getFrame(frame) != 0) {
    Serial.println("Failed to read frame!");
    return;
  }

  Serial.println("Thermal Image (Temperatures in Celsius):");
  for (int y = 0; y < 24; y++) {
    for (int x = 0; x < 32; x++) {
      float temp = frame[y * 32 + x];
      Serial.print(temp, 1);
      Serial.print(" ");
    }
    Serial.println();
  }
  Serial.println("END");
  delay(1000);
}
