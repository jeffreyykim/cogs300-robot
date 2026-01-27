// MyRobot-V1.ino
// This file must be named the same as your sketch folder

// Forward declarations for motor control functions
void initializeMotors();
void forward(int speed);
void backward(int speed);
void stop();
void turnLeft(int baseSpeed, int speedBoost);
void turnRight(int baseSpeed, int speedBoost);
void turnInPlace(int speed, bool clockwise);

void setup() {
    // Initialize serial communication for logging
    Serial.begin(9600);
    
    // Initialize motor control pins
    initializeMotors();
}

void loop() {
    // Drive forward for 2 seconds
    logInfo(Serial, "Moving forward");
    forward(200);
    delay(2000);

    // Stop for 1 second
    logInfo(Serial, "Stopping");
    stop();
    delay(1000);

    // Drive backward for 2 seconds
    logInfo(Serial, "Moving backward");
    backward(200);
    delay(2000);

    // Stop for 1 second
    logInfo(Serial, "Stopping");
    stop();
    delay(1000);

    // Turn left for 2 seconds
    logInfo(Serial, "Turning left");
    turnLeft(150, 50);
    delay(2000);

    // Stop for 1 second
    logInfo(Serial, "Stopping");
    stop();
    delay(1000);

    // Turn right for 2 seconds
    logInfo(Serial, "Turning right");
    turnRight(150, 50);
    delay(2000);

    // Stop for 1 second
    logInfo(Serial, "Stopping");
    stop();
    delay(1000);

    // Spin clockwise for 2 seconds
    logInfo(Serial, "Spinning clockwise");
    turnInPlace(180, true);
    delay(2000);

    // Stop for 1 second
    logInfo(Serial, "Stopping");
    stop();
    delay(1000);

    // Spin counter-clockwise for 2 seconds
    logInfo(Serial, "Spinning counter-clockwise");
    turnInPlace(180, false);
    delay(2000);

    // Stop for 2 seconds before repeating
    logInfo(Serial, "Stopping");
    stop();
    delay(2000);
}
