/************************************************
 * Line-Following with Five IR Sensors
 * Using a Majority-Vote Software Filter
 *
 * Physical Layout (Facing the line):
 *   Sensor 5 -- Sensor 4 -- Sensor 3 -- Sensor 2 -- Sensor 1
 *
 *   - Sensor 1 is the farthest RIGHT
 *   - Sensor 5 is the farthest LEFT
 *   - Sensor 3 sits at the exact middle of the line
 ************************************************/

// ========== Pin Definitions ==========
const int irSensorPin1 = 2;  // Far right
const int irSensorPin2 = 3;
const int irSensorPin3 = 4;  // Center
const int irSensorPin4 = 5;
const int irSensorPin5 = 6;  // Far left

// ========== Filter Configuration ==========
#define FILTER_SIZE 5  // Number of samples for each sensor's circular buffer

// Buffers for each sensor
int s1Buffer[FILTER_SIZE] = {0};
int s2Buffer[FILTER_SIZE] = {0};
int s3Buffer[FILTER_SIZE] = {0};
int s4Buffer[FILTER_SIZE] = {0};
int s5Buffer[FILTER_SIZE] = {0};

// Write index for each buffer (which element to overwrite next)
int s1Index = 0;
int s2Index = 0;
int s3Index = 0;
int s4Index = 0;
int s5Index = 0;

// Function Prototypes
void addToBuffer(int buffer[], int &index, int newValue);
int getFilteredValue(const int buffer[]);

void setup() {
  Serial.begin(115200);

  pinMode(irSensorPin1, INPUT);
  pinMode(irSensorPin2, INPUT);
  pinMode(irSensorPin3, INPUT);
  pinMode(irSensorPin4, INPUT);
  pinMode(irSensorPin5, INPUT);

  Serial.println("Line-Following IR Sensor Array Initialized");
}

void loop() {
  // 1) Read the raw digital values from each sensor
  int rawS1 = digitalRead(irSensorPin1);
  int rawS2 = digitalRead(irSensorPin2);
  int rawS3 = digitalRead(irSensorPin3);
  int rawS4 = digitalRead(irSensorPin4);
  int rawS5 = digitalRead(irSensorPin5);

  // 2) Add each raw reading to its corresponding buffer
  addToBuffer(s1Buffer, s1Index, rawS1);
  addToBuffer(s2Buffer, s2Index, rawS2);
  addToBuffer(s3Buffer, s3Index, rawS3);
  addToBuffer(s4Buffer, s4Index, rawS4);
  addToBuffer(s5Buffer, s5Index, rawS5);

  // 3) Compute the filtered (majority vote) value for each sensor
  int filtS1 = getFilteredValue(s1Buffer);
  int filtS2 = getFilteredValue(s2Buffer);
  int filtS3 = getFilteredValue(s3Buffer);
  int filtS4 = getFilteredValue(s4Buffer);
  int filtS5 = getFilteredValue(s5Buffer);

  // 4) Print Raw vs Filtered data
  Serial.print("Raw => S1: "); Serial.print(rawS1);
  Serial.print(" S2: ");       Serial.print(rawS2);
  Serial.print(" S3: ");       Serial.print(rawS3);
  Serial.print(" S4: ");       Serial.print(rawS4);
  Serial.print(" S5: ");       Serial.print(rawS5);

  Serial.print("  |  Filtered => S1: "); Serial.print(filtS1);
  Serial.print(" S2: ");                Serial.print(filtS2);
  Serial.print(" S3: ");                Serial.print(filtS3);
  Serial.print(" S4: ");                Serial.print(filtS4);
  Serial.print(" S5: ");                Serial.print(filtS5);

  Serial.println();

  // 5) line detection logic using FILTERED values
  //    (0 = white line, 1 = black background)
  if (filtS1 == 0 && filtS2 == 1 && filtS3 == 1 && filtS4 == 1 && filtS5 == 0) {
    Serial.println("Center on the line");
  } 
  else if (filtS1 == 0 || filtS2 == 0) {
    Serial.println("Line is towards the LEFT side");
  } 
  else if (filtS4 == 0 || filtS5 == 0) {
    Serial.println("Line is towards the RIGHT side");
  } 
  else {
    Serial.println("No clear line detected or ambiguous state");
  }

  Serial.println("---------------------------------------");
  delay(500);
}

// ========== Helper Functions ==========

/**
 * Add a new value into a circular buffer. 
 * 'index' will move forward by 1 (mod FILTER_SIZE).
 */
void addToBuffer(int buffer[], int &index, int newValue) {
  buffer[index] = newValue;
  index = (index + 1) % FILTER_SIZE;  // Wrap around
}

/**
 * Return a "majority vote" from the given buffer.
 * If more than half the samples are HIGH (1), return 1.
 * Otherwise, return 0.
 */
int getFilteredValue(const int buffer[]) {
  int sum = 0;
  for(int i = 0; i < FILTER_SIZE; i++){
    sum += buffer[i];
  }
  
  // If sum > FILTER_SIZE/2, majority are 1's
  if(sum > (FILTER_SIZE / 2)) {
    return 1;
  } else {
    return 0;
  }
}
