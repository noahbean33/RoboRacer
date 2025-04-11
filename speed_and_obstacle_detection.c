/************************************************
 * Simple Obstacle Detection with TFmini Plus LiDAR
 * Outputs PWM at 100 until an obstacle is detected within 800 cm
 * PWM stops permanently until board is restarted
 * LED indicates PWM state: ON when PWM is active, OFF when stopped
 ************************************************/

// --- Pin Definitions ---
const uint8_t TFMINI_RX_PIN = 2;  // LiDAR TX (green wire) to Arduino Pin 2 (RX)
const uint8_t TFMINI_TX_PIN = 3;  // LiDAR RX (white wire) to Arduino Pin 3 (TX) - Use voltage divider!
const uint8_t PWM_PIN = 9;        // PWM output pin (PWM-capable: 3, 5, 6, 9, 10, 11)
const uint8_t STATUS_LED_PIN = 13; // LED to indicate PWM state (built-in LED on Pin 13)

// --- LiDAR Communication Constants ---
const long TFMINI_BAUD_RATE = 115200;
const uint8_t LIDAR_FRAME_LENGTH = 9;
const uint8_t LIDAR_HEADER = 0x59;

// --- Application Constants ---
const int OBSTACLE_THRESHOLD_CM = 800; // Distance threshold in cm
const int PWM_ON_VALUE = 100;          // PWM duty cycle (0-255) when no obstacle
const int FILTER_WINDOW_SIZE = 15;     // Size of the median filter window (larger for robustness)
const int CONSISTENT_READINGS_REQUIRED = 5; // Number of consecutive readings below threshold to confirm obstacle

// --- Global Variables ---
#include <SoftwareSerial.h>
SoftwareSerial tfminiSerial(TFMINI_RX_PIN, TFMINI_TX_PIN);

int distanceBuffer[FILTER_WINDOW_SIZE]; // Circular buffer for median filter
int bufferIndex = 0;
int samplesCollected = 0;
int consecutiveReadingsBelowThreshold = 0;
bool pwmStopped = false; // Flag to indicate if PWM has been stopped

void setup() {
    // Initialize Serial for debugging
    Serial.begin(115200);
    while (!Serial) {
        ; // Wait for Serial Monitor to connect
    }
    Serial.println("Initializing Obstacle Detection System...");

    // Initialize PWM pin
    pinMode(PWM_PIN, OUTPUT);
    analogWrite(PWM_PIN, PWM_ON_VALUE); // Start with PWM on
    Serial.print("PWM started on Pin ");
    Serial.print(PWM_PIN);
    Serial.print(" with value ");
    Serial.println(PWM_ON_VALUE);

    // Initialize status LED
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, HIGH); // LED ON when PWM is active
    Serial.print("Status LED initialized on Pin ");
    Serial.println(STATUS_LED_PIN);

    // Initialize SoftwareSerial for LiDAR
    tfminiSerial.begin(TFMINI_BAUD_RATE);
    if (!tfminiSerial) {
        Serial.println("SoftwareSerial for TFmini failed to start!");
        while (1); // Halt
    }
    Serial.println("TFmini SoftwareSerial Initialized.");

    // Flush the SoftwareSerial buffer
    while (tfminiSerial.available()) {
        tfminiSerial.read();
    }

    // Factory reset the LiDAR
    uint8_t factoryReset[] = {0x5A, 0x04, 0x10, 0x6E};
    tfminiSerial.write(factoryReset, sizeof(factoryReset));
    Serial.println("Sent factory reset command");
    delay(200);

    // Set the frame rate to 10 Hz to reduce load on SoftwareSerial
    uint8_t setFrameRate[] = {0x5A, 0x06, 0x03, 0x0A, 0x00, 0x5D};
    tfminiSerial.write(setFrameRate, sizeof(setFrameRate));
    Serial.println("Sent command to set frame rate to 10 Hz");
    delay(200);

    // Enable data output
    uint8_t enableDataOutput[] = {0x5A, 0x05, 0x07, 0x01, 0x50};
    tfminiSerial.write(enableDataOutput, sizeof(enableDataOutput));
    Serial.println("Sent command to enable data output");
    delay(200);

    // Save settings
    uint8_t saveSettings[] = {0x5A, 0x04, 0x11, 0x6E};
    tfminiSerial.write(saveSettings, sizeof(saveSettings));
    Serial.println("Sent command to save settings");
    delay(200);

    // Flush the buffer again
    while (tfminiSerial.available()) {
        tfminiSerial.read();
    }

    // Warm-up delay to ensure LiDAR is ready
    Serial.println("Waiting for LiDAR to initialize...");
    delay(1000); // 1-second delay to allow LiDAR to start sending data

    // Initialize the filter buffer
    for (int i = 0; i < FILTER_WINDOW_SIZE; i++) {
        distanceBuffer[i] = 0;
    }

    Serial.println("System Initialized. Starting detection loop.");
}

void loop() {
    // If PWM has been stopped, do nothing until the board is restarted
    if (pwmStopped) {
        digitalWrite(STATUS_LED_PIN, LOW); // Ensure LED is OFF when PWM is stopped
        delay(1000); // Small delay to reduce CPU usage
        return;
    }

    // Read raw distance from the LiDAR
    int rawDistance = -1;
    if (readTFmini(&rawDistance)) {
        // Apply robust median filter
        float filteredDistance = applyMedianFilter(rawDistance);

        // Print raw and filtered distances for debugging
        Serial.print("Raw Distance: ");
        Serial.print(rawDistance);
        Serial.print(" cm, Filtered Distance: ");
        Serial.print(filteredDistance, 1);
        Serial.println(" cm");

        // Check for obstacle with consistency requirement
        if (filteredDistance > 0 && filteredDistance <= OBSTACLE_THRESHOLD_CM) {
            consecutiveReadingsBelowThreshold++;
            Serial.print("Obstacle detected (");
            Serial.print(consecutiveReadingsBelowThreshold);
            Serial.print("/");
            Serial.print(CONSISTENT_READINGS_REQUIRED);
            Serial.println(" consecutive readings below threshold)");
        } else {
            consecutiveReadingsBelowThreshold = 0; // Reset if reading is above threshold or invalid
            Serial.println("No obstacle detected.");
        }

        // Stop PWM if obstacle is consistently detected
        if (consecutiveReadingsBelowThreshold >= CONSISTENT_READINGS_REQUIRED) {
            analogWrite(PWM_PIN, 0); // Stop PWM
            digitalWrite(STATUS_LED_PIN, LOW); // Turn LED OFF
            pwmStopped = true;
            Serial.println("!!! OBSTACLE CONFIRMED !!! PWM STOPPED. Restart the board to resume.");
        }
    } else {
        Serial.print("Failed to read from LiDAR. Bytes available: ");
        Serial.println(tfminiSerial.available());
        consecutiveReadingsBelowThreshold = 0; // Reset on invalid reading
    }

    delay(100); // Match the 10 Hz frame rate
}

// Read a 9-byte frame from the TFmini Plus LiDAR with better synchronization
bool readTFmini(int* distance) {
    *distance = -1;

    // Retry up to 3 times to read a valid frame
    for (int retry = 0; retry < 3; retry++) {
        // Search for the frame header (0x59 0x59)
        unsigned long startTime = millis();
        while (true) {
            // Wait for at least 1 byte to be available
            if (tfminiSerial.available() < 1) {
                if (millis() - startTime > 500) { // Increased timeout to 500 ms
                    return false; // Timeout if no data
                }
                delay(1);
                continue;
            }

            // Read the first byte
            uint8_t firstByte = tfminiSerial.read();
            if (firstByte != LIDAR_HEADER) {
                continue; // Not the header, keep looking
            }

            // Wait for the second byte
            startTime = millis();
            while (tfminiSerial.available() < 1) {
                if (millis() - startTime > 500) {
                    return false; // Timeout
                }
                delay(1);
            }

            uint8_t secondByte = tfminiSerial.read();
            if (secondByte != LIDAR_HEADER) {
                continue; // Not the header, keep looking
            }

            // Found the header, now read the remaining 7 bytes
            uint8_t buffer[LIDAR_FRAME_LENGTH];
            buffer[0] = LIDAR_HEADER;
            buffer[1] = LIDAR_HEADER;

            // Wait for the remaining bytes
            startTime = millis();
            while (tfminiSerial.available() < (LIDAR_FRAME_LENGTH - 2)) {
                if (millis() - startTime > 500) {
                    return false; // Timeout
                }
                delay(1);
            }

            for (int i = 2; i < LIDAR_FRAME_LENGTH; i++) {
                buffer[i] = tfminiSerial.read();
            }

            // Print raw bytes for debugging
            Serial.print("Raw Frame: ");
            for (int i = 0; i < LIDAR_FRAME_LENGTH; i++) {
                Serial.print("0x");
                if (buffer[i] < 0x10) Serial.print("0");
                Serial.print(buffer[i], HEX);
                Serial.print(" ");
            }
            Serial.println();

            // Verify the checksum
            uint8_t calculatedChecksum = 0;
            for (int i = 0; i < LIDAR_FRAME_LENGTH - 1; i++) {
                calculatedChecksum += buffer[i];
            }
            if (buffer[LIDAR_FRAME_LENGTH - 1] != calculatedChecksum) {
                Serial.print("Checksum error! Calculated: 0x");
                if (calculatedChecksum < 0x10) Serial.print("0");
                Serial.print(calculatedChecksum, HEX);
                Serial.print(", Expected: 0x");
                if (buffer[LIDAR_FRAME_LENGTH - 1] < 0x10) Serial.print("0");
                Serial.println(buffer[LIDAR_FRAME_LENGTH - 1], HEX);
                continue; // Retry on checksum error
            }

            // Extract the distance
            *distance = buffer[2] + (buffer[3] << 8);
            return true;
        }
    }

    return false; // Failed after retries
}

// Apply a robust median filter to the raw distance
float applyMedianFilter(int rawDistance) {
    distanceBuffer[bufferIndex] = rawDistance;
    bufferIndex = (bufferIndex + 1) % FILTER_WINDOW_SIZE;
    if (samplesCollected < FILTER_WINDOW_SIZE) {
        samplesCollected++;
    }

    if (samplesCollected == 0) {
        return (float)rawDistance;
    }

    // Copy the buffer for sorting
    int tempBuffer[FILTER_WINDOW_SIZE];
    for (int i = 0; i < samplesCollected; i++) {
        tempBuffer[i] = distanceBuffer[i];
    }

    // Bubble sort
    for (int i = 0; i < samplesCollected - 1; i++) {
        for (int j = 0; j < samplesCollected - 1 - i; j++) {
            if (tempBuffer[j] > tempBuffer[j + 1]) {
                int temp = tempBuffer[j];
                tempBuffer[j] = tempBuffer[j + 1];
                tempBuffer[j + 1] = temp;
            }
        }
    }

    int middleIndex = samplesCollected / 2;
    if (samplesCollected % 2 == 1) {
        return (float)tempBuffer[middleIndex];
    } else {
        return (float)(tempBuffer[middleIndex - 1] + tempBuffer[middleIndex]) / 2.0f;
    }
}
