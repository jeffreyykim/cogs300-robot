/**
 * @file Motor.ino
 * @brief Simple H-bridge motor control helpers.
 *
 * Provides a minimal interface for driving a DC motor using
 * digital GPIO pins (e.g. Arduino-style platforms).
 *
 * The motor direction is controlled via two input pins,
 * while a separate enable pin turns the motor on or off.
 *
 * @author Paul Bucci
 * @date 2026
 */


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