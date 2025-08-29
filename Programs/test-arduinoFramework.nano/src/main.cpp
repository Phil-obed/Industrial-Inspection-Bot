#include <Arduino.h>
#include "Mototr"
#define Rf 23
#define Rb 22
#define enR 21

#define Lf 19
#define Lb 18
#define enL 5

void setup() {
  pinMode(Rf, OUTPUT);
  pinMode(Rb, OUTPUT);
  pinMode(enR, OUTPUT);

  pinMode(Lf, OUTPUT);
  pinMode(Lb, OUTPUT);
  pinMode(enL, OUTPUT);
}

// Ramp up/down PWM
void rampSpeed(int startSpeed, int endSpeed, int stepDelay = 10) {
  if (startSpeed < endSpeed) {
    for (int spd = startSpeed; spd <= endSpeed; spd += 5) {
      analogWrite(enR, spd);
      analogWrite(enL, spd);
      delay(stepDelay);
    }
  } else {
    for (int spd = startSpeed; spd >= endSpeed; spd -= 5) {
      analogWrite(enR, spd);
      analogWrite(enL, spd);
      delay(stepDelay);
    }
  }
}

void move(int rf, int rb, int lf, int lb, int duration) {
  digitalWrite(Rf, rf);
  digitalWrite(Rb, rb);
  digitalWrite(Lf, lf);
  digitalWrite(Lb, lb);

  rampSpeed(0, 255);   // Ramp up
  delay(duration);
  rampSpeed(255, 0);   // Ramp down

  // Stop motors
  digitalWrite(Rf, LOW);
  digitalWrite(Rb, LOW);
  digitalWrite(Lf, LOW);
  digitalWrite(Lb, LOW);
}

void loop() {
  move(HIGH, LOW, HIGH, LOW, 3000);   // Forward for 3s
  delay(500);

  move(LOW, HIGH, LOW, LOW, 3000);    // Reverse left (left motor back only)
  delay(500);

  move(LOW, LOW, LOW, HIGH, 3000);    // Reverse right (right motor back only)
  delay(500);
}
