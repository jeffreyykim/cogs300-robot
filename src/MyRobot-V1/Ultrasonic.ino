/**
 * @file Ultrasonic.ino
 * @brief HC-SR04 ultrasonic distance sensor driver — two sensors.
 *
 * Front sensor (follow-me, forward-facing):
 *   TRIG -> Arduino pin 11
 *   ECHO -> 330Ω resistor -> Arduino pin 12
 *
 * Side sensor (wall-follow, sideways-facing):
 *   TRIG -> Arduino pin 13
 *   ECHO -> 330Ω resistor -> Arduino pin A0
 *
 * Wiring for each sensor:
 *   VCC  -> 5V
 *   GND  -> GND
 *   TRIG -> Arduino pin (direct, signal out from Arduino)
 *   ECHO -> 330Ω series resistor -> Arduino pin  (limits current into the pin)
 */

// Front sensor pins
const int ULTRASONIC_FRONT_TRIG = 11;
const int ULTRASONIC_FRONT_ECHO = 12;

// Side sensor pins
const int ULTRASONIC_SIDE_TRIG  = 13;
const int ULTRASONIC_SIDE_ECHO  = A0;

const float          ULTRASONIC_MAX_CM      = 300.0;
const float          ULTRASONIC_MIN_CM      = 2.0;
const unsigned long  ULTRASONIC_TIMEOUT_US  = 20000UL;  // ~340 cm max

static bool g_sonarTelemetryEnabled = false;

/**
 * @brief Initialize both ultrasonic sensor pins.
 */
void initializeUltrasonic() {
  pinMode(ULTRASONIC_FRONT_TRIG, OUTPUT);
  pinMode(ULTRASONIC_FRONT_ECHO, INPUT);
  digitalWrite(ULTRASONIC_FRONT_TRIG, LOW);

  pinMode(ULTRASONIC_SIDE_TRIG, OUTPUT);
  pinMode(ULTRASONIC_SIDE_ECHO, INPUT);
  digitalWrite(ULTRASONIC_SIDE_TRIG, LOW);

  Serial.println("Ultrasonics initialized (front: trig=11 echo=12 | side: trig=13 echo=A0)");
}

/**
 * @brief Internal helper — fire one pulse on the given trig/echo pair.
 * @return Distance in cm, or -1.0 if out of range / no echo.
 */
static float readSensor(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, ULTRASONIC_TIMEOUT_US);
  if (duration == 0) return -1.0;

  float cm = (duration / 2.0) * 0.0343;
  if (cm < ULTRASONIC_MIN_CM || cm > ULTRASONIC_MAX_CM) return -1.0;
  return cm;
}

/**
 * @brief Read the front-facing (follow-me) sensor.
 * @return Distance in cm, or -1.0 if out of range.
 */
float getFrontDistanceCm() {
  return readSensor(ULTRASONIC_FRONT_TRIG, ULTRASONIC_FRONT_ECHO);
}

/**
 * @brief Read the side-facing (wall-follow) sensor.
 * @return Distance in cm, or -1.0 if out of range.
 */
float getSideDistanceCm() {
  return readSensor(ULTRASONIC_SIDE_TRIG, ULTRASONIC_SIDE_ECHO);
}

/**
 * @brief Enable or disable sonar CSV telemetry streaming.
 */
void setSonarTelemetryEnabled(bool en) {
  g_sonarTelemetryEnabled = en;
  if (en) Serial.println(F("t_ms,front_cm,side_cm"));  // CSV header
}

bool isSonarTelemetryEnabled() {
  return g_sonarTelemetryEnabled;
}

/**
 * @brief Print one reading from both sensors in CSV format.
 */
void printUltrasonicTelemetry() {
  Serial.print(millis());
  Serial.print(",");
  Serial.print(getFrontDistanceCm());
  Serial.print(",");
  Serial.println(getSideDistanceCm());
}

/**
 * @brief Call from main loop to stream sonar data when enabled.
 */
void sonarTick() {
  if (g_sonarTelemetryEnabled) {
    printUltrasonicTelemetry();
  }
}
