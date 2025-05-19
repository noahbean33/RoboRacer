#include <Wire.h>
#include <Adafruit_TCS34725.h>

// --- Configuration ---
const int NUM_SENSORS = 7;
const uint8_t TCA_ADDR = 0x70;
const int I2C_SDA_PIN = 21;
const int I2C_SCL_PIN = 8;

// PWM Output Configuration 
const int PWM_OUTPUT_PIN = 2; // PWM output 
const int LEDC_CHANNEL = 0;     // LEDC channel (0-15)
const int LEDC_FREQ = 50;       // PWM frequency in Hz (50Hz for servos)
const int LEDC_RESOLUTION_BITS = 16; // PWM resolution (0-65535 duty cycle)
const int LEDC_MAX_DUTY = (1 << LEDC_RESOLUTION_BITS) - 1; // Max duty for 16-bit = 65535

// Servo-like pulse width parameters (in microseconds)
const int PWM_PERIOD_US = 20000;   // 1/50Hz = 20ms = 20000 us
const int PWM_CENTER_PULSE_US = 1500; // Center pulse width
const int PWM_PULSE_DEVIATION_US = 400; // +/- from center (e.g., 1100us to 1900us range)

// --- Line Sensor Configuration ---
const int sensorWeights[NUM_SENSORS] = {-3, -2, -1, 0, 1, 2, 3};
bool isSensorWhite[NUM_SENSORS] = {false};

// --- SMA Filter for Line Position Error ---
const int SMA_WINDOW_SIZE = 5;
float errorWindow[SMA_WINDOW_SIZE] = {0.0};
int errorWindowIndex = 0;
float filteredError = 0.0;

// --- PI Controller Configuration ---
float Kp = 0.5;
float Ki = 0.02;
float setpoint = 0.0;
float integralSum = 0.0;
const float MAX_INTEGRAL_SUM = 50.0;
const float MIN_CONTROL_OUTPUT = -100.0;
const float MAX_CONTROL_OUTPUT = 100.0;

// --- EMA Filter for Sensor Readings ---
const float EMA_ALPHA = 0.3;
float ema_C[NUM_SENSORS];
float ema_B[NUM_SENSORS];
bool ema_initialized[NUM_SENSORS] = {false};

// --- Calibrated Color Classification Thresholds ---
const float FILTERED_C_WHITE_THRESHOLD = 6000.0;
const float FILTERED_B_WHITE_THRESHOLD = 1600.0;


Adafruit_TCS34725 tcs[NUM_SENSORS] = {
  Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X),
  Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X),
  Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X), 
  Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X),
  Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X),
  Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X),
  Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X)
};

void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

bool checkWhite(float filtered_c, float filtered_b) {
  return (filtered_c > FILTERED_C_WHITE_THRESHOLD && filtered_b > FILTERED_B_WHITE_THRESHOLD);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Serial.println("ESP32-S3 Line Follower - PI to PWM Test");
  Serial.println("--- Monitoring PWM on Oscilloscope ---");

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  // Setup LEDC PWM channel
  ledcSetup(LEDC_CHANNEL, LEDC_FREQ, LEDC_RESOLUTION_BITS);
  ledcAttachPin(PWM_OUTPUT_PIN, LEDC_CHANNEL);
  
  // Write center pulse initially
  long center_duty = map(PWM_CENTER_PULSE_US, 0, PWM_PERIOD_US, 0, LEDC_MAX_DUTY);
  ledcWrite(LEDC_CHANNEL, center_duty);
  Serial.print("Initial PWM Duty (Center): "); Serial.println(center_duty);


  bool all_sensors_found_in_setup = true;
  for (int i = 0; i < NUM_SENSORS; i++) {
    tcaSelect(i);
    delay(5);
    if (tcs[i].begin()) {
      tcs[i].setIntegrationTime(TCS34725_INTEGRATIONTIME_50MS);
      tcs[i].setGain(TCS34725_GAIN_4X);
    } else {
      Serial.print("Sensor on Ch "); Serial.print(i); Serial.println(" NOT FOUND!");
      all_sensors_found_in_setup = false;
    }
  }

  if (!all_sensors_found_in_setup) {
     Serial.println("One or more sensors failed to initialize.");
     while(1);
  }
  Serial.println("\nAll sensors initialized\n");
}

void loop() {
  uint16_t r_raw, g_raw, b_raw, c_raw;
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    tcaSelect(i);
    delay(1); 
    tcs[i].getRawData(&r_raw, &g_raw, &b_raw, &c_raw);

    if (!ema_initialized[i]) {
      ema_C[i] = c_raw;
      ema_B[i] = b_raw;
      ema_initialized[i] = true;
    } else {
      ema_C[i] = (c_raw * EMA_ALPHA) + (ema_C[i] * (1.0 - EMA_ALPHA));
      ema_B[i] = (b_raw * EMA_ALPHA) + (ema_B[i] * (1.0 - EMA_ALPHA));
    }
    isSensorWhite[i] = checkWhite(ema_C[i], ema_B[i]);
  }

  float currentErrorSum = 0;
  int whiteSensorsCount = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (isSensorWhite[i]) {
      currentErrorSum += sensorWeights[i];
      whiteSensorsCount++;
    }
  }

  float rawError;
  if (whiteSensorsCount > 0) {
    rawError = currentErrorSum; 
  } else {
    rawError = (filteredError == 0) ? (MAX_CONTROL_OUTPUT/Kp /2.0) : filteredError * 1.5 ;
    if (rawError > sensorWeights[NUM_SENSORS-1]*2) rawError = sensorWeights[NUM_SENSORS-1]*2;
    if (rawError < sensorWeights[0]*2) rawError = sensorWeights[0]*2;
  }

  errorWindow[errorWindowIndex] = rawError;
  errorWindowIndex = (errorWindowIndex + 1) % SMA_WINDOW_SIZE;
  
  float sumForSma = 0;
  for (int i = 0; i < SMA_WINDOW_SIZE; i++) {
    sumForSma += errorWindow[i];
  }
  filteredError = sumForSma / SMA_WINDOW_SIZE;

  float pTerm = Kp * filteredError;
  integralSum += (Ki * filteredError);
  if (integralSum > MAX_INTEGRAL_SUM) integralSum = MAX_INTEGRAL_SUM;
  if (integralSum < -MAX_INTEGRAL_SUM) integralSum = -MAX_INTEGRAL_SUM;

  float piOutput = pTerm + integralSum;

  if (piOutput > MAX_CONTROL_OUTPUT) piOutput = MAX_CONTROL_OUTPUT;
  if (piOutput < MIN_CONTROL_OUTPUT) piOutput = MIN_CONTROL_OUTPUT;

  // Map PI output to a pulse width in microseconds
  long pulse_width_us = PWM_CENTER_PULSE_US + map(piOutput, MIN_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT, -PWM_PULSE_DEVIATION_US, PWM_PULSE_DEVIATION_US);
  
  if (pulse_width_us < (PWM_CENTER_PULSE_US - PWM_PULSE_DEVIATION_US)) {
    pulse_width_us = PWM_CENTER_PULSE_US - PWM_PULSE_DEVIATION_US;
  }
  if (pulse_width_us > (PWM_CENTER_PULSE_US + PWM_PULSE_DEVIATION_US)) {
    pulse_width_us = PWM_CENTER_PULSE_US + PWM_PULSE_DEVIATION_US;
  }
  
  // Convert pulse width in microseconds to LEDC duty cycle
  uint32_t dutyCycle = (pulse_width_us * LEDC_MAX_DUTY) / PWM_PERIOD_US;
  
  ledcWrite(LEDC_CHANNEL, dutyCycle);

  // Debug Printing
  Serial.print("Sens: ");
  for(int i=0; i<NUM_SENSORS; i++) {
    Serial.print(isSensorWhite[i] ? "W" : "_");
  }
  Serial.print(" | FErr: "); Serial.print(filteredError, 1);
  Serial.print(" | PI_Out: "); Serial.print(piOutput, 1);
  Serial.print(" | Pulse(us): "); Serial.print(pulse_width_us);
  Serial.print(" | Duty: "); Serial.println(dutyCycle);

  delay(10); 
}
