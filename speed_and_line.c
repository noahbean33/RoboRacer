#include <Servo.h>

// Define servo object
Servo steeringServo;

// Sensor pins (sensor 1 on pin 2 is far right, sensor 7 on pin 8 is far left)
const int irSensorPins[] = {2, 3, 4, 5, 6, 7, 8};

// PWM pin for steering servo
const int steeringPWMPin = 9;

// Majority-vote filter parameters
#define FILTER_SIZE 5
int sBuffers[7][FILTER_SIZE] = {0}; // Buffers for each sensor
int sIndices[7] = {0};              // Current index in each buffer

// PID constants (tuned for high speed)
float kp = 15.0; // Proportional gain
float ki = 0.05; // Integral gain (lower to avoid overshoot at high speed)
float kd = 2.0;  // Derivative gain (increased for stability)

// PID variables
float integral = 0.0;
float previousError = 0.0;
unsigned long lastTime = 0;

// Sensor weights (index 0 = sensor 1 far right, index 6 = sensor 7 far left)
const float sensorWeights[] = {3, 2, 1, 0, -1, -2, -3};

// Steering angle parameters
const int baseAngle = 90;  // Center position (degrees)
const int minAngle = 60;   // Full left
const int maxAngle = 120;  // Full right

void setup() {
  // Initialize sensor pins as inputs
  for (int i = 0; i < 7; i++) {
    pinMode(irSensorPins[i], INPUT);
  }

  // Attach servo to PWM pin
  steeringServo.attach(steeringPWMPin);

  // Set initial steering to center
  steeringServo.write(baseAngle);

  // Start serial for debugging
  Serial.begin(115200);
  Serial.println("Line-Following Robot Initialized");

  // Initialize timing
  lastTime = millis();
}

void loop() {
  // Read raw sensor values
  int rawSensors[7];
  for (int i = 0; i < 7; i++) {
    rawSensors[i] = digitalRead(irSensorPins[i]);
  }

  // Update filter buffers
  for (int i = 0; i < 7; i++) {
    addToBuffer(sBuffers[i], sIndices[i], rawSensors[i]);
  }

  // Get filtered sensor values
  int filtSensors[7];
  for (int i = 0; i < 7; i++) {
    filtSensors[i] = getFilteredValue(sBuffers[i]);
  }

  // Calculate line position
  float position = 0.0;
  int activeSensors = 0;
  for (int i = 0; i < 7; i++) {
    if (filtSensors[i] == 0) { // Line detected (LOW)
      position += sensorWeights[i];
      activeSensors++;
    }
  }
  if (activeSensors > 0) {
    position /= activeSensors; // Average position
  } else {
    position = 0; // No line detected, assume center
  }

  // PID calculation
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0; // Time in seconds
  lastTime = currentTime;

  float error = position; // Setpoint is 0 (line centered)
  integral += error * deltaTime;

  // Limit integral windup
  const float integralLimit = 50.0;
  integral = constrain(integral, -integralLimit, integralLimit);

  float derivative = (error - previousError) / deltaTime;
  previousError = error;

  float output = kp * error + ki * integral + kd * derivative;

  // Calculate and constrain steering angle
  int steeringAngle = baseAngle - output; // Negative to steer right when line is right
  steeringAngle = constrain(steeringAngle, minAngle, maxAngle);

  // Set steering position
  steeringServo.write(steeringAngle);

  // Debug output
  Serial.print("Pos: "); Serial.print(position, 2);
  Serial.print(" | Error: "); Serial.print(error, 2);
  Serial.print(" | PID Output: "); Serial.print(output, 2);
  Serial.print(" | Angle: "); Serial.print(steeringAngle);
  Serial.print(" | PWM (Angle): "); Serial.print(steeringAngle); // Servo angle is PWM equivalent
  Serial.print(" | Sensors: ");
  for (int i = 0; i < 7; i++) {
    Serial.print(filtSensors[i]); Serial.print(" ");
  }
  Serial.println();

  // Minimize loop delay for high-speed response
  delay(5); // Reduced from 10ms
}

// Add new sensor reading to buffer
void addToBuffer(int buffer[], int &index, int newValue) {
  buffer[index] = newValue;
  index = (index + 1) % FILTER_SIZE;
}

// Get majority-vote filtered value
int getFilteredValue(const int buffer[]) {
  int sum = 0;
  for (int i = 0; i < FILTER_SIZE; i++) {
    sum += buffer[i];
  }
  return (sum > (FILTER_SIZE / 2)) ? 1 : 0; // Majority vote
}
