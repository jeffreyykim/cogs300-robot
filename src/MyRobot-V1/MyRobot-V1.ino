#include <Arduino.h>

// Forward declarations for motor control functions
void initializeMotors();
void forward(int speed);
void backward(int speed);
void stop();
void turnLeft(int baseSpeed, int speedBoost);
void turnRight(int baseSpeed, int speedBoost);
void turnInPlace(int speed, bool clockwise);

// From Serial.ino
void logInfo(Stream& out, const char* msg);

// From CommandInterface.ino
void serialInterfaceTick();

void setup() {
  Serial.begin(9600);
  initializeMotors();
  logInfo(Serial, "Robot ready. Type HELP in Serial Monitor.");
}

void loop() {
  serialInterfaceTick();
}
