#include <Servo.h>

Servo myServo;  // Create servo object

int pos = 0;    // Variable to store servo position

void setup() {
  myServo.attach(13);  // Attach servo to pin 9
  Serial.begin(9600);
  Serial.println("Servo Sweep Test Started");
}

void loop() {
  // Sweep from 0 to 180 degrees
  for (pos = 0; pos <= 180; pos += 1) {
    myServo.write(pos);
    Serial.print("Moving to: ");
    Serial.print(pos);
    Serial.println(" degrees");
    delay(15); // Wait for servo to reach position
  }
  
  delay(1000); // Pause at 180 degrees
  
  // Sweep from 180 to 0 degrees
  for (pos = 180; pos >= 0; pos -= 1) {
    myServo.write(pos);
    Serial.print("Moving to: ");
    Serial.print(pos);
    Serial.println(" degrees");
    delay(15);
  }
  
  delay(1000); // Pause at 0 degrees
}
