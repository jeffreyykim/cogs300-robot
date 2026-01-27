/**
 * @file Motor.ino
 * @brief H-bridge motor control with speed and direction support.
 *
 * Provides an interface for driving two DC motors using PWM for speed control
 * and direction pins for forward/backward control. Includes high-level functions
 * for robot movement: forward, backward, left turn, right turn, and in-place turn.
 *
 * Motor A: enA (PWM pin 9), in1 (pin 8), in2 (pin 7)
 * Motor B: enB (PWM pin 3), in3 (pin 6), in4 (pin 5)
 *
 * @author Paul Bucci
 * @date 2026
 */

// Motor A pins (left motor)
int enA = 9;   // Enable pin for Motor A — must be a PWM-capable pin
int in1 = 8;   // Direction control pin 1 for Motor A
int in2 = 7;   // Direction control pin 2 for Motor A

// Motor B pins (right motor)
int enB = 3;   // Enable pin for Motor B — must be a PWM-capable pin
int in3 = 6;   // Direction control pin 1 for Motor B
int in4 = 5;   // Direction control pin 2 for Motor B

/**
 * @brief Initialize motor control pins.
 */
void initializeMotors() {
    // Set motor control pins as outputs
    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    
    pinMode(enB, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
}


/**
 * @brief Drives a DC motor in a fixed direction using an H-bridge.
 *
 * @param in1 GPIO pin connected to motor driver input 1 (direction control)
 * @param in2 GPIO pin connected to motor driver input 2 (direction control)
 * @param enA GPIO pin connected to motor driver enable pin (motor on/off)
 */
void drive(int in1, int in2, int enA) {
    digitalWrite(in1, LOW);   // Direction control: IN1
    digitalWrite(in2, HIGH);  // Direction control: IN2 (sets rotation direction)
    digitalWrite(enA, HIGH);  // Enable motor driver
}

void stop(int in1, int in2, int enA) {
    digitalWrite(in1, LOW);   // Direction control: IN1
    digitalWrite(in2, HIGH);  // Direction control: IN2 (sets rotation direction)
    digitalWrite(enA, LOW);   // Disable motor driver
}

/**
 * @brief Drives a single motor at specified speed and direction.
 */
void driveMotor(int in1, int in2, int enA, int speed, bool forward) {
    if (forward) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    } else {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    }
    analogWrite(enA, speed);
}

/**
 * @brief Stop a single motor immediately.
 */
void stopMotor(int in1, int in2, int enA) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(enA, 0);
}

/**
 * @brief Drive the robot forward.
 */
void forward(int speed = 200) {
    driveMotor(in1, in2, enA, speed, true);
    driveMotor(in3, in4, enB, speed, true);
}

/**
 * @brief Drive the robot backward.
 */
void backward(int speed = 200) {
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
    int leftSpeed = baseSpeed;
    int rightSpeed = baseSpeed + speedBoost;
    driveMotor(in1, in2, enA, leftSpeed, true);
    driveMotor(in3, in4, enB, rightSpeed, true);
}

/**
 * @brief Turn the robot right while driving forward.
 */
void turnRight(int baseSpeed = 150, int speedBoost = 50) {
    int leftSpeed = baseSpeed + speedBoost;
    int rightSpeed = baseSpeed;
    driveMotor(in1, in2, enA, leftSpeed, true);
    driveMotor(in3, in4, enB, rightSpeed, true);
}

/**
 * @brief Turn the robot in place (spin).
 */
void turnInPlace(int speed = 180, bool clockwise = true) {
    if (clockwise) {
        driveMotor(in1, in2, enA, speed, true);
        driveMotor(in3, in4, enB, speed, false);
    } else {
        driveMotor(in1, in2, enA, speed, false);
        driveMotor(in3, in4, enB, speed, true);
    }
}