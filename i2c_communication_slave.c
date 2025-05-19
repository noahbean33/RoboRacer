#include <Wire.h>

#define MOTOR_PIN 9 // PWM output pin
#define SLAVE_ADDRESS 8 // I2C slave address
int pwmValue = 0; // Store received PWM value

void setup() {
  // Start I2C as slave
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveEvent); // Register receive event
  // Set up motor pin
  pinMode(MOTOR_PIN, OUTPUT);
}

void loop() {
  // Apply received PWM value
  analogWrite(MOTOR_PIN, pwmValue);
}

void receiveEvent(int bytes) {
  if (Wire.available()) {
    pwmValue = Wire.read(); // Read PWM value (0-255)
  }
}
