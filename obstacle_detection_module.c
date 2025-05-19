// Define the serial communication pins for TFmini Plus
#define TFMINI_RX_PIN 2  // Connect TFmini TX to Arduino Digital Pin 2 (RX)
#define TFMINI_TX_PIN 3  // Connect TFmini RX to Arduino Digital Pin 3 (TX)

// Include the SoftwareSerial library for serial communication
#include <SoftwareSerial.h>

// Create a SoftwareSerial object for TFmini Plus
SoftwareSerial tfminiSerial(TFMINI_RX_PIN, TFMINI_TX_PIN);

// Buffer for incoming TFmini data (9 bytes per frame)
uint8_t buffer[9];

// Raw distance from TFmini (in cm)
int lidarDistance = 0;

// Number of samples to hold in median buffer
#define MEDIAN_FILTER_SIZE 5

// Circular buffer to store recent distance samples
int distanceBuffer[MEDIAN_FILTER_SIZE] = {0};
int bufferIndex = 0;

// Tracks how many samples collected so far
int samplesCollected = 0;

// ========== Setup ==========

void setup() {
  Serial.begin(115200);         // Serial monitor output
  tfminiSerial.begin(115200);   // TFmini default baud rate

  // Print the expected polling (sampling) rate
  Serial.println("TFmini LiDAR Obstacle Detection Initialized");
  Serial.println("Expected polling rate: ~100 Hz (default for TFmini).");

  // Command to enable continuous data output on TFmini
  uint8_t enableDataOutput[] = {
    0x42, 0x57, 0x02, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x06
  };
  tfminiSerial.write(enableDataOutput, sizeof(enableDataOutput));
}

void loop() {
  // Read TFmini data if available
  readTFmini();

  // Compute or retrieve the median distance from our buffer
  float medianDistance = getMedianDistance();

  // Print both the latest raw reading and the median
  Serial.print("LiDAR Distance (raw): ");
  Serial.print(lidarDistance);
  Serial.print(" cm, (median-filtered): ");
  Serial.print(medianDistance, 1); // 1 decimal place
  Serial.println(" cm");

  // Check if obstacle is within 8 meters (800 cm), using the median value
  checkObstacle(medianDistance);
}


// Reads a 9-byte frame from TFmini and updates lidarDistance & median buffer
void readTFmini() {
  // Process data only if at least 9 bytes in the buffer
  while (tfminiSerial.available() >= 9) {
    // Read the 9-byte frame
    for (int i = 0; i < 9; i++) {
      buffer[i] = tfminiSerial.read();
    }
    // Check for valid frame header (0x59 0x59)
    if (buffer[0] == 0x59 && buffer[1] == 0x59) {
      // Extract raw distance in cm (low byte + high byte)
      int rawDistance = buffer[2] + (buffer[3] << 8);

      // Update global lidarDistance
      lidarDistance = rawDistance;

      // Store into circular buffer for median filtering
      distanceBuffer[bufferIndex] = rawDistance;
      bufferIndex = (bufferIndex + 1) % MEDIAN_FILTER_SIZE;

      // Keep track of how many samples placed in the buffer
      if (samplesCollected < MEDIAN_FILTER_SIZE) {
        samplesCollected++;
      }
    }
  }
}

// Returns the median of the distances in distanceBuffer
float getMedianDistance() {
  // If no samples collected yet, just return the last read distance
  if (samplesCollected == 0) {
    return (float)lidarDistance;
  }

  // Temporary array to copy the valid samples
  int temp[MEDIAN_FILTER_SIZE];

  // Decide how many samples (up to MEDIAN_FILTER_SIZE)
  int count = samplesCollected;

  // Copy those samples into temp
  for (int i = 0; i < count; i++) {
    temp[i] = distanceBuffer[i];
  }

  // Sort the subset [0..count-1]
  for (int i = 0; i < count - 1; i++) {
    for (int j = i + 1; j < count; j++) {
      if (temp[j] < temp[i]) {
        int swap = temp[i];
        temp[i] = temp[j];
        temp[j] = swap;
      }
    }
  }

  // If the entire buffer has not been filled yet, take the middle of 'count' samples
  int middle = count / 2;

  // If count is odd, just pick the middle
  // If count is even, average the two middle values or just pick one
  if (count % 2 == 1) {
    return (float)temp[middle];
  } else {
    // average the two middle values
    float avg = (float)(temp[middle - 1] + temp[middle]) / 2.0;
    return avg;
  }
}

// Checks if distance <= 800 cm, if distance <= 800 cm, print alert
void checkObstacle(float distanceCm) {
  if (distanceCm > 0 && distanceCm <= 800.0) {
    Serial.print("Obstacle detected at ");
    Serial.print(distanceCm, 1); // 1 decimal place
    Serial.println(" cm -> Emergency Brake Engaged!");
  }
}
