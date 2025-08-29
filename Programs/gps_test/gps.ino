#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

TinyGPSPlus gps;
HardwareSerial ss(1);

static const int RXPin = 16;   // GPS TX -> ESP32 RX
static const uint32_t GPSBaud = 9600;

void setup()
{
  Serial.begin(115200);
  ss.begin(GPSBaud, SERIAL_8N1, RXPin, -1);

  Serial.println(F("Testing GY-GPS6MV2 with ESP32..."));
}

void loop()
{
  while (ss.available() > 0)
  {
    gps.encode(ss.read());
  }

  // Show info every second
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 1000)
  {
    lastPrint = millis();

    Serial.println(F("----- GPS STATUS -----"));

    // Satellites in use
    if (gps.satellites.isValid())
      Serial.printf("Satellites: %d\n", gps.satellites.value());
    else
      Serial.println(F("Satellites: not available"));

    // Fix status
    if (gps.location.isValid())
    {
      Serial.printf("Location: %.6f, %.6f\n",
                    gps.location.lat(), gps.location.lng());
    }
    else
    {
      Serial.println(F("Location: NO FIX"));
    }

    // Altitude
    if (gps.altitude.isValid())
    {
      Serial.printf("Altitude: %.2f m\n", gps.altitude.meters());
    }
    else
    {
      Serial.println(F("Altitude: not available"));
    }

    // Speed
    if (gps.speed.isValid())
    {
      Serial.printf("Speed: %.2f km/h\n", gps.speed.kmph());
    }
    else
    {
      Serial.println(F("Speed: not available"));
    }

    // Date & Time
    if (gps.date.isValid() && gps.time.isValid())
    {
      Serial.printf("Date: %02d/%02d/%04d\n", 
                    gps.date.day(), gps.date.month(), gps.date.year());
      Serial.printf("Time (UTC): %02d:%02d:%02d\n", 
                    gps.time.hour(), gps.time.minute(), gps.time.second());
    }
    else
    {
      Serial.println(F("Date/Time: not available"));
    }

    Serial.println(F("----------------------\n"));
  }
}
