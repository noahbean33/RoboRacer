#include <Wire.h>

#define SLAVE_ADDRESS 8 // I2C address for Arduino 2
int pwmValue = 0; // Current PWM value

void setup() {
  // Start I2C as master
  Wire.begin();
}

void loop() {
  // Send current PWM value to Slave
  Wire.beginTransmission(SLAVE_ADDRESS);
  Wire.write((byte)pwmValue); // Send PWM value as byte
  Wire.endTransmission();

  // Increment PWM value by 10
  pwmValue += 10;
  if (pwmValue > 255) {
    pwmValue = 0; // Reset to 0 after 255
  } else if (pwmValue == 250) {
    pwmValue = 255; // Include 255 before resetting
  }

  delay(5000); // Hold for 5 seconds
}
