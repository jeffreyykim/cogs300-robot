/**
 * @file Motor.ino
 * @brief H-bridge motor control with PWM speed + direction support.
 *
 * Motor A: enA (PWM pin 9), in1 (pin 8), in2 (pin 7)
 * Motor B: enB (PWM pin 3), in3 (pin 6), in4 (pin 5)
 */

// Motor A pins (left motor)
int enA = 9;   // PWM
int in1 = 8;
int in2 = 7;

// Motor B pins (right motor)
int enB = 3;   // PWM
int in3 = 6;
int in4 = 5;

// Optional tuning
int MIN_SPEED = 0; // disabled to allow low-speed testing

// ---- helpers ----
int clampSpeed(int s) {
  if (s < 0) return 0;
  if (s > 255) return 255;
  return s;
}

int applyMinSpeed(int s) {
  s = clampSpeed(s);
  if (s > 0 && s < MIN_SPEED) return MIN_SPEED;
  return s;
}

/**
 * @brief Initialize motor control pins.
 */
void initializeMotors() {
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // start stopped
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, LOW);
}

/**
 * @brief Drives a single motor at specified speed and direction.
 * @param in1,in2 direction pins
 * @param en PWM enable pin
 * @param speed 0-255
 * @param forward true = forward, false = backward
 */
void driveMotor(int in1, int in2, int en, int speed, bool forward) {
  speed = applyMinSpeed(speed);

  if (forward) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }

  analogWrite(en, speed);
}

/**
 * @brief Stop a single motor.
 */
void stopMotor(int in1, int in2, int en) {
  // Coast stop (works with most H-bridges)
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(en, 0);
}

/**
 * @brief Drive the robot forward.
 */
void forward(int speed = 200) {
  Serial.print("FWD: Motor A (left) @ ");
  Serial.print(speed);
  Serial.print(", Motor B (right) @ ");
  Serial.println(speed);
  driveMotor(in1, in2, enA, speed, true);
  driveMotor(in3, in4, enB, speed, true);
}

/**
 * @brief Drive the robot backward.
 */
void backward(int speed = 200) {
  Serial.print("REV: Motor A (left) @ ");
  Serial.print(speed);
  Serial.print(", Motor B (right) @ ");
  Serial.println(speed);
  driveMotor(in1, in2, enA, speed, false);
  driveMotor(in3, in4, enB, speed, false);
}

/**
 * @brief Stop the robot.
 */
void stop() {
  stopMotor(in1, in2, enA);
  stopMotor(in3, in4, enB);
}

/**
 * @brief Turn the robot left while driving forward.
 */
void turnLeft(int baseSpeed = 150, int speedBoost = 50) {
  int leftSpeed  = clampSpeed(baseSpeed);
  int rightSpeed = clampSpeed(baseSpeed + speedBoost);
  driveMotor(in1, in2, enA, leftSpeed, true);
  driveMotor(in3, in4, enB, rightSpeed, true);
}

/**
 * @brief Turn the robot right while driving forward.
 */
void turnRight(int baseSpeed = 150, int speedBoost = 50) {
  int leftSpeed  = clampSpeed(baseSpeed + speedBoost);
  int rightSpeed = clampSpeed(baseSpeed);
  driveMotor(in1, in2, enA, leftSpeed, true);
  driveMotor(in3, in4, enB, rightSpeed, true);
}

/**
 * @brief Turn in place (spin).
 */
void turnInPlace(int speed = 180, bool clockwise = true) {
  speed = clampSpeed(speed);
  if (clockwise) {
    driveMotor(in1, in2, enA, speed, true);
    driveMotor(in3, in4, enB, speed, false);
  } else {
    driveMotor(in1, in2, enA, speed, false);
    driveMotor(in3, in4, enB, speed, true);
  }
}

/**
 * @brief Test Motor A (left) individually - forward
 */
void testMotorAForward(int speed = 150) {
  Serial.println("Testing Motor A (left) FORWARD");
  driveMotor(in1, in2, enA, speed, true);
}

/**
 * @brief Test Motor A (left) individually - backward
 */
void testMotorABackward(int speed = 150) {
  Serial.println("Testing Motor A (left) BACKWARD");
  driveMotor(in1, in2, enA, speed, false);
}

/**
 * @brief Test Motor B (right) individually - forward
 */
void testMotorBForward(int speed = 150) {
  Serial.println("Testing Motor B (right) FORWARD");
  driveMotor(in3, in4, enB, speed, true);
}

/**
 * @brief Test Motor B (right) individually - backward
 */
void testMotorBBackward(int speed = 150) {
  Serial.println("Testing Motor B (right) BACKWARD");
  driveMotor(in3, in4, enB, speed, false);
}
