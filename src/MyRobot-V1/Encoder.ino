/**
 * @file Encoder.ino
 * @brief Encoder reading for wheel rotation tracking
 *
 * Encoder A (left): Pin 2 (digital read)
 * Encoder B (right): Pin 4 (digital read)
 */

// Encoder pins
const int encoderA_pin = 2;  // left wheel encoder
const int encoderB_pin = 4;  // right wheel encoder

// Encoder counters
volatile long encoderA_count = 0;
volatile long encoderB_count = 0;

// Previous states for edge detection
int encoderA_lastState = HIGH;
int encoderB_lastState = HIGH;

/**
 * @brief Initialize encoder pins
 */
void initializeEncoders() {
  pinMode(encoderA_pin, INPUT_PULLUP);
  pinMode(encoderB_pin, INPUT_PULLUP);
  
  encoderA_lastState = digitalRead(encoderA_pin);
  encoderB_lastState = digitalRead(encoderB_pin);
  
  Serial.println("Encoders initialized on pins 2 (left) and 4 (right)");
}

/**
 * @brief Read and update encoder counts (call frequently in loop)
 */
void updateEncoders() {
  // Read encoder A (left wheel)
  int stateA = digitalRead(encoderA_pin);
  if (stateA != encoderA_lastState && stateA == LOW) {
    encoderA_count++;
  }
  encoderA_lastState = stateA;
  
  // Read encoder B (right wheel)
  int stateB = digitalRead(encoderB_pin);
  if (stateB != encoderB_lastState && stateB == LOW) {
    encoderB_count++;
  }
  encoderB_lastState = stateB;
}

/**
 * @brief Reset encoder counts to zero
 */
void resetEncoders() {
  encoderA_count = 0;
  encoderB_count = 0;
  Serial.println("Encoders reset");
}

/**
 * @brief Get left wheel encoder count
 */
long getEncoderA() {
  return encoderA_count;
}

/**
 * @brief Get right wheel encoder count
 */
long getEncoderB() {
  return encoderB_count;
}

/**
 * @brief Print encoder telemetry in CSV format
 */
void printEncoderTelemetry() {
  Serial.print(millis());
  Serial.print(",");
  Serial.print(encoderA_count);
  Serial.print(",");
  Serial.println(encoderB_count);
}
