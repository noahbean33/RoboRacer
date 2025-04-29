// Define pin numbers
const int sensorPin = 3;  // Photo interrupter signal pin
const int pwmPin = 9;     // PWM output pin

void setup() {
  pinMode(sensorPin, INPUT);  // Set sensor pin as input
  pinMode(pwmPin, OUTPUT);    // Set PWM pin as output (optional but good practice)
}

void loop() {
  int val = digitalRead(sensorPin);  // Read the sensor state
  if (val == HIGH) {                 // Sensor is blocked
    analogWrite(pwmPin, 100);        // Output PWM value of 100
  } else {                           // Sensor is unblocked
    analogWrite(pwmPin, 0);          // Stop PWM (set to 0)
  }
}
