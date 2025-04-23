#include <SoftwareSerial.h>

SoftwareSerial BTSerial(10, 11);
const int ledPin = 13;
bool isBlinking = false;

void setup() {
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  Serial.begin(9600);
  BTSerial.begin(9600);
  Serial.println("Arduino with HC-05 is ready");
}

void loop() {
  // Check for any incoming data
  if (BTSerial.available()) {
    String command = BTSerial.readStringUntil('\n');
    command.trim();

    // Debug: Print raw received data
    Serial.print("Raw data received: [");
    Serial.print(command);
    Serial.println("]");

    // Case-insensitive comparison
    if (command.equalsIgnoreCase("ON")) {
      isBlinking = true;
      Serial.println("LED blinking started");
    } else if (command.equalsIgnoreCase("OFF")) {
      isBlinking = false;
      digitalWrite(ledPin, LOW);
      Serial.println("LED blinking stopped");
    } else {
      Serial.println("Unknown command");
    }
  }

  if (isBlinking) {
    digitalWrite(ledPin, HIGH);
    delay(500);
    digitalWrite(ledPin, LOW);
    delay(500);
  }
}
