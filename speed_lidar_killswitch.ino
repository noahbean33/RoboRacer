/************************************************
 * Obstacle Detection and Motor Control with TFmini Plus LiDAR and Kill Switch
 * Outputs PWM at 100 until an obstacle is detected within 100 cm or kill switch is unblocked
 * PWM stops permanently until board is restarted
 * LED indicates PWM state: ON when PWM is active, OFF when stopped
 ************************************************/

// --- Pin Definitions ---
const uint8_t TFMINI_RX_PIN = 2;      // LiDAR TX (green wire) to Arduino Pin 2 (RX)
const uint8_t TFMINI_TX_PIN = 3;      // LiDAR RX (white wire) to Arduino Pin 3 (TX) - Use voltage divider!
const uint8_t PWM_PIN = 9;            // PWM output pin (PWM-capable: 3, 5, 6, 9, 10, 11)
const uint8_t STATUS_LED_PIN = 13;    // LED to indicate PWM state (built-in LED on Pin 13)
const uint8_t KILL_SWITCH_PIN = 4;    // Kill switch (KY-010 signal pin)

// --- LiDAR Communication Constants ---
const long TFMINI_BAUD_RATE = 115200;
const uint8_t LIDAR_FRAME_LENGTH = 9;
const uint8_t LIDAR_HEADER = 0x59;

// --- Application Constants ---
const int OBSTACLE_THRESHOLD_CM = 100;      // Distance threshold in cm (1 meter)
const int PWM_ON_VALUE = 100;               // PWM duty cycle (0-255) when active
const int FILTER_WINDOW_SIZE = 15;          // Size of the median filter window
const int CONSISTENT_READINGS_REQUIRED = 10; // Number of consecutive readings below threshold to confirm obstacle

// --- Global Variables ---
#include <SoftwareSerial.h>
SoftwareSerial tfminiSerial(TFMINI_RX_PIN, TFMINI_TX_PIN);

int distanceBuffer[FILTER_WINDOW_SIZE]; // Circular buffer for median filter
int bufferIndex = 0;
int samplesCollected = 0;
int consecutiveReadingsBelowThreshold = 0;
bool pwmStopped = false; // Flag to indicate if PWM has been stopped

void setup() {
    Serial.begin(115200);
    while (!Serial) { ; }
    Serial.println("Initializing Obstacle Detection System...");

    pinMode(PWM_PIN, OUTPUT);
    analogWrite(PWM_PIN, PWM_ON_VALUE);
    Serial.print("PWM started on Pin ");
    Serial.print(PWM_PIN);
    Serial.print(" with value ");
    Serial.println(PWM_ON_VALUE);

    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, HIGH);
    Serial.print("Status LED initialized on Pin ");
    Serial.println(STATUS_LED_PIN);

    pinMode(KILL_SWITCH_PIN, INPUT);
    Serial.print("Kill Switch initialized on Pin ");
    Serial.print(KILL_SWITCH_PIN);
    Serial.print(", Initial State: ");
    Serial.println(digitalRead(KILL_SWITCH_PIN) == HIGH ? "Blocked" : "Unblocked");

    tfminiSerial.begin(TFMINI_BAUD_RATE);
    if (!tfminiSerial) {
        Serial.println("SoftwareSerial for TFmini failed to start!");
        while (1);
    }
    Serial.println("TFmini SoftwareSerial Initialized.");

    while (tfminiSerial.available()) { tfminiSerial.read(); }

    uint8_t factoryReset[] = {0x5A, 0x04, 0x10, 0x6E};
    tfminiSerial.write(factoryReset, sizeof(factoryReset));
    Serial.println("Sent factory reset command");
    delay(200);

    uint8_t setFrameRate[] = {0x5A, 0x06, 0x03, 0x0A, 0x00, 0x5D};
    tfminiSerial.write(setFrameRate, sizeof(setFrameRate));
    Serial.println("Sent command to set frame rate to 10 Hz");
    delay(200);

    uint8_t enableDataOutput[] = {0x5A, 0x05, 0x07, 0x01, 0x50};
    tfminiSerial.write(enableDataOutput, sizeof(enableDataOutput));
    Serial.println("Sent command to enable data output");
    delay(200);

    uint8_t saveSettings[] = {0x5A, 0x04, 0x11, 0x6E};
    tfminiSerial.write(saveSettings, sizeof(saveSettings));
    Serial.println("Sent command to save settings");
    delay(200);

    while (tfminiSerial.available()) { tfminiSerial.read(); }

    Serial.println("Waiting for LiDAR to initialize...");
    delay(1000);

    for (int i = 0; i < FILTER_WINDOW_SIZE; i++) { distanceBuffer[i] = 0; }

    Serial.println("System Initialized. Starting detection loop.");
}

void loop() {
    if (pwmStopped) {
        digitalWrite(STATUS_LED_PIN, LOW);
        delay(1000);
        return;
    }

    if (digitalRead(KILL_SWITCH_PIN) == LOW) {
        analogWrite(PWM_PIN, 0);
        digitalWrite(STATUS_LED_PIN, LOW);
        pwmStopped = true;
        Serial.println("!!! KILL SWITCH ACTIVATED !!! PWM STOPPED. Restart the board to resume.");
        return;
    }

    int rawDistance = -1;
    if (readTFmini(&rawDistance)) {
        float filteredDistance = applyMedianFilter(rawDistance);
        Serial.print("Raw Distance: ");
        Serial.print(rawDistance);
        Serial.print(" cm, Filtered Distance: ");
        Serial.print(filteredDistance, 1);
        Serial.println(" cm");

        if (filteredDistance > 0 && filteredDistance <= OBSTACLE_THRESHOLD_CM) {
            consecutiveReadingsBelowThreshold++;
            Serial.print("Obstacle detected (");
            Serial.print(consecutiveReadingsBelowThreshold);
            Serial.print("/");
            Serial.print(CONSISTENT_READINGS_REQUIRED);
            Serial.println(" consecutive readings below threshold)");
        } else {
            consecutiveReadingsBelowThreshold = 0;
            Serial.println("No obstacle detected.");
        }

        if (consecutiveReadingsBelowThreshold >= CONSISTENT_READINGS_REQUIRED) {
            analogWrite(PWM_PIN, 0);
            digitalWrite(STATUS_LED_PIN, LOW);
            pwmStopped = true;
            Serial.println("!!! OBSTACLE CONFIRMED !!! PWM STOPPED. Restart the board to resume.");
        }
    } else {
        Serial.print("Failed to read from LiDAR. Bytes available: ");
        Serial.println(tfminiSerial.available());
        consecutiveReadingsBelowThreshold = 0;
    }

    delay(100);
}

bool readTFmini(int* distance) {
    *distance = -1;
    for (int retry = 0; retry < 3; retry++) {
        unsigned long startTime = millis();
        while (true) {
            if (tfminiSerial.available() < 1) {
                if (millis() - startTime > 500) { return false; }
                delay(1);
                continue;
            }

            uint8_t firstByte = tfminiSerial.read();
            if (firstByte != LIDAR_HEADER) { continue; }

            startTime = millis();
            while (tfminiSerial.available() < 1) {
                if (millis() - startTime > 500) { return false; }
                delay(1);
            }

            uint8_t secondByte = tfminiSerial.read();
            if (secondByte != LIDAR_HEADER) { continue; }

            uint8_t buffer[LIDAR_FRAME_LENGTH];
            buffer[0] = LIDAR_HEADER;
            buffer[1] = LIDAR_HEADER;

            startTime = millis();
            while (tfminiSerial.available() < (LIDAR_FRAME_LENGTH - 2)) {
                if (millis() - startTime > 500) { return false; }
                delay(1);
            }

            for (int i = 2; i < LIDAR_FRAME_LENGTH; i++) {
                buffer[i] = tfminiSerial.read();
            }

            uint8_t calculatedChecksum = 0;
            for (int i = 0; i < LIDAR_FRAME_LENGTH - 1; i++) {
                calculatedChecksum += buffer[i];
            }
            if (buffer[LIDAR_FRAME_LENGTH - 1] != calculatedChecksum) {
                Serial.println("Checksum error!");
                continue;
            }

            *distance = buffer[2] + (buffer[3] << 8);
            return true;
        }
    }
    return false;
}

float applyMedianFilter(int rawDistance) {
    distanceBuffer[bufferIndex] = rawDistance;
    bufferIndex = (bufferIndex + 1) % FILTER_WINDOW_SIZE;
    if (samplesCollected < FILTER_WINDOW_SIZE) { samplesCollected++; }

    if (samplesCollected == 0) { return (float)rawDistance; }

    int tempBuffer[FILTER_WINDOW_SIZE];
    for (int i = 0; i < samplesCollected; i++) {
        tempBuffer[i] = distanceBuffer[i];
    }

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
