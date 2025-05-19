// --- Pin Definitions ---
const uint8_t TFMINI_RX_PIN = 2;      // LiDAR TX (green wire) to Arduino Pin 2 (RX)
const uint8_t TFMINI_TX_PIN = 3;      // LiDAR RX (white wire) to Arduino Pin 3 (TX)
const uint8_t ESC_CONTROL_PIN = 9;    // ESC signal pin 
const uint8_t STATUS_LED_PIN = 13;    // LED to indicate ESC state
const uint8_t KILL_SWITCH_PIN = 4;    // Kill switch signal pin

// --- LiDAR Communication Constants ---
const long TFMINI_BAUD_RATE = 115200;
const uint8_t LIDAR_FRAME_LENGTH = 9;
const uint8_t LIDAR_HEADER = 0x59;

// --- Application Constants ---
const int OBSTACLE_THRESHOLD_CM = 800;      // Distance threshold in cm (8 meters)

// ESC Control Values
const int ESC_STOP_PULSE = 2000;            // neutral/stop pulse for ESCs
const int ESC_ARM_PULSE = 1500;             // Pulse to send during arming sequence
const int ESC_TARGET_SPEED_PULSE = 1400;    // 1494 Wheel RPM (20mph)
                                            
const unsigned long ESC_ARMING_TIME_MS = 3000; // Time for ESC to arm (send stop pulse)

const int FILTER_WINDOW_SIZE = 15;          // Size of the median filter window
const int CONSISTENT_READINGS_REQUIRED = 2; // Number of consecutive readings below threshold to confirm obstacle

// --- Global Variables ---
#include <SoftwareSerial.h>
#include <Servo.h> // Servo library is used for ESC control

SoftwareSerial tfminiSerial(TFMINI_RX_PIN, TFMINI_TX_PIN);
Servo esc; // Create a Servo object to control the ESC

int distanceBuffer[FILTER_WINDOW_SIZE]; // Circular buffer for median filter
int bufferIndex = 0;
int samplesCollected = 0;
int consecutiveReadingsBelowThreshold = 0;
bool motorStoppedPermanently = false; // Flag to indicate if motor has been stopped

void setup() {
    Serial.begin(115200);
    while (!Serial) { ; }
    Serial.println("Initializing Obstacle Detection & RC ESC Control System...");
    Serial.print("Target wheel RPM for 20mph (4.5\" wheels): ~");
    Serial.println(1494); // Calculated RPM
    Serial.print("Consistent readings required for stop: ");
    Serial.println(CONSISTENT_READINGS_REQUIRED);


    esc.attach(ESC_CONTROL_PIN);
    Serial.println("ESC Pin Attached. Arming sequence initiated...");
    Serial.print("Sending STOP pulse (");
    Serial.print(ESC_ARM_PULSE);
    Serial.println("us) for arming.");
    esc.writeMicroseconds(ESC_ARM_PULSE);
    delay(ESC_ARMING_TIME_MS);
    Serial.println("ESC Arming sequence complete.");


    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);
    Serial.print("Status LED initialized on Pin ");
    Serial.println(STATUS_LED_PIN);

    pinMode(KILL_SWITCH_PIN, INPUT_PULLUP);
    Serial.print("Kill Switch initialized on Pin ");
    Serial.print(KILL_SWITCH_PIN);
    Serial.print(", Initial State: ");
    Serial.println(digitalRead(KILL_SWITCH_PIN) == HIGH ? "Not Activated" : "Activated");


    tfminiSerial.begin(TFMINI_BAUD_RATE);
    if (!tfminiSerial) {
        Serial.println("SoftwareSerial for TFmini failed to start! Halting.");
        while (1);
    }
    Serial.println("TFmini SoftwareSerial Initialized.");

    while (tfminiSerial.available()) { tfminiSerial.read(); }

    Serial.println("Configuring TFmini Plus...");
    uint8_t setStandardOutput[] = {0x5A, 0x05, 0x05, 0x01, 0x65};
    tfminiSerial.write(setStandardOutput, sizeof(setStandardOutput));
    Serial.println("Sent: Set standard output format (cm).");
    delay(100);

    uint8_t setFrameRate10Hz[] = {0x5A, 0x06, 0x03, 0x0A, 0x00, 0x6D}; // 10Hz
    tfminiSerial.write(setFrameRate10Hz, sizeof(setFrameRate10Hz));
    Serial.println("Sent: Set frame rate to 10 Hz.");
    delay(100);

    uint8_t saveSettings[] = {0x5A, 0x04, 0x11, 0x6F};
    tfminiSerial.write(saveSettings, sizeof(saveSettings));
    Serial.println("Sent: Save settings.");
    delay(100);

    while (tfminiSerial.available()) { tfminiSerial.read(); }

    Serial.println("Waiting for LiDAR to stabilize...");
    delay(1000);

    for (int i = 0; i < FILTER_WINDOW_SIZE; i++) { distanceBuffer[i] = 0; }

    Serial.print("System Initialized. Starting motor with pulse: ");
    Serial.print(ESC_TARGET_SPEED_PULSE);
    Serial.println("us");
    esc.writeMicroseconds(ESC_TARGET_SPEED_PULSE);
    digitalWrite(STATUS_LED_PIN, HIGH);

    Serial.println("Detection loop starting.");
}

void loop() {
    if (motorStoppedPermanently) {
        delay(1000);
        return;
    }

    if (digitalRead(KILL_SWITCH_PIN) == LOW) {
        Serial.println("KILL SWITCH ACTIVATED. Restart board to resume.");
        esc.writeMicroseconds(ESC_STOP_PULSE);
        digitalWrite(STATUS_LED_PIN, LOW);
        motorStoppedPermanently = true;
        return;
    }

    int rawDistance = -1;
    if (readTFmini(&rawDistance)) {
        float filteredDistance = applyMedianFilter(rawDistance);
        Serial.print("Raw: "); Serial.print(rawDistance);
        Serial.print("cm, Filtered: "); Serial.print(filteredDistance, 1); Serial.print("cm");

        if (filteredDistance > 0 && filteredDistance <= OBSTACLE_THRESHOLD_CM) {
            consecutiveReadingsBelowThreshold++;
            Serial.print(" | Obstacle detected (");
            Serial.print(consecutiveReadingsBelowThreshold);
            Serial.print("/");
            Serial.print(CONSISTENT_READINGS_REQUIRED);
            Serial.println(")");
        } else {
            if (consecutiveReadingsBelowThreshold > 0) { // Print only if it was counting
                 Serial.println(" | Obstacle condition lost or out of range. Resetting counter.");
            }
            consecutiveReadingsBelowThreshold = 0;
            if (filteredDistance > OBSTACLE_THRESHOLD_CM) {
                Serial.println(" | No obstacle (beyond threshold).");
            } else if (rawDistance > 0) {
                Serial.println(" | Filter output invalid or initializing (<=0).");
            }
        }

        if (consecutiveReadingsBelowThreshold >= CONSISTENT_READINGS_REQUIRED) {
            Serial.println("OBSTACLE CONFIRMED. Restart board to resume.");
            esc.writeMicroseconds(ESC_STOP_PULSE);
            digitalWrite(STATUS_LED_PIN, LOW);
            motorStoppedPermanently = true;
        }
    } else {
        Serial.print("### readTFmini FAILED. Bytes available: ");
        Serial.print(tfminiSerial.available());
        Serial.println(". Resetting obstacle counter. ###");
        consecutiveReadingsBelowThreshold = 0;
    }

    delay(100);
}

bool readTFmini(int* distance) {
    *distance = -1;

    unsigned long frameStartTime = millis();
    while(tfminiSerial.available() < LIDAR_FRAME_LENGTH) {
        if (millis() - frameStartTime > 200) { // Timeout (2x frame time at 10Hz)
            Serial.println("readTFmini DEBUG: Timeout waiting for enough bytes for a frame.");
            return false;
        }
        delay(1);
    }

    uint8_t firstByte = 0, secondByte = 0;
    unsigned long syncStartTime = millis();
    bool synced = false;
    while(millis() - syncStartTime < 100) {
        if (tfminiSerial.available() >= 2) {
            firstByte = tfminiSerial.read();
            if (firstByte == LIDAR_HEADER) {
                secondByte = tfminiSerial.read();
                if (secondByte == LIDAR_HEADER) {
                    synced = true;
                    break;
                }
            }
        } else {
            delay(1);
        }
    }

    if (!synced) {
        Serial.println("readTFmini DEBUG: Failed to sync with LiDAR headers.");
        // Clear buffer on sync fail to prevent reading stale data from partial previous frame
        while(tfminiSerial.available()) tfminiSerial.read();
        return false;
    }

    uint8_t buffer[LIDAR_FRAME_LENGTH];
    buffer[0] = LIDAR_HEADER;
    buffer[1] = LIDAR_HEADER;

    size_t bytesRead = tfminiSerial.readBytes(&buffer[2], LIDAR_FRAME_LENGTH - 2);

    if (bytesRead != (LIDAR_FRAME_LENGTH - 2)) {
        Serial.println("readTFmini DEBUG: Incomplete LiDAR frame read.");
        return false;
    }

    uint8_t calculatedChecksum = 0;
    for (int i = 0; i < LIDAR_FRAME_LENGTH - 1; i++) {
        calculatedChecksum += buffer[i];
    }

    if (buffer[LIDAR_FRAME_LENGTH - 1] != calculatedChecksum) {
        Serial.println("readTFmini DEBUG: Checksum error!");
        Serial.print("Buffer: "); for(int i=0; i<LIDAR_FRAME_LENGTH; i++) { Serial.print(buffer[i], HEX); Serial.print(" "); } Serial.println();
        return false;
    }

    *distance = buffer[2] + (buffer[3] << 8);
    return true;
}

// Median Filter
float applyMedianFilter(int rawDistance) {
    distanceBuffer[bufferIndex] = rawDistance;
    bufferIndex = (bufferIndex + 1) % FILTER_WINDOW_SIZE;

    if (samplesCollected < FILTER_WINDOW_SIZE) {
        samplesCollected++;
    }

    int sortCount = (samplesCollected < FILTER_WINDOW_SIZE) ? samplesCollected : FILTER_WINDOW_SIZE;

    if (sortCount == 0) {
        return 0.0f;
    }

    int tempBuffer[sortCount];
    for (int i = 0; i < sortCount; i++) {
        int oldestDataIndex;
        if (samplesCollected < FILTER_WINDOW_SIZE) {
             oldestDataIndex = i; 
             tempBuffer[i] = distanceBuffer[i]; 
        } 
        else 
        {
            oldestDataIndex = (bufferIndex + i) % FILTER_WINDOW_SIZE;
            tempBuffer[i] = distanceBuffer[oldestDataIndex];
        }
    }


    // Sort the temporary array
    for (int i = 0; i < sortCount - 1; i++) {
        for (int j = 0; j < sortCount - 1 - i; j++) {
            if (tempBuffer[j] > tempBuffer[j + 1]) {
                int temp = tempBuffer[j];
                tempBuffer[j] = tempBuffer[j + 1];
                tempBuffer[j + 1] = temp;
            }
        }
    }

    // Calculate median
    int middleIndex = sortCount / 2;
    if (sortCount % 2 == 1) { 
        return (float)tempBuffer[middleIndex];
    } else { 
        if (sortCount > 0) { 
            return (float)(tempBuffer[middleIndex - 1] + tempBuffer[middleIndex]) / 2.0f;
        } else {
            return 0.0f;
        }
    }
}
