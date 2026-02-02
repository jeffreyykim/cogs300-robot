#include <Arduino.h>

// Forward declarations for motor control functions
void initializeMotors();
void forward(int speed);
void backward(int speed);
void stop();
void turnLeft(int baseSpeed, int speedBoost);
void turnRight(int baseSpeed, int speedBoost);
void turnInPlace(int speed, bool clockwise);

// From Encoder.ino
void initializeEncoders();
void updateEncoders();

// From Serial.ino
void logInfo(Stream& out, const char* msg);

// From CommandInterface.ino
void serialInterfaceTick();

void setup() {
  Serial.begin(9600);
  initializeMotors();
  initializeEncoders();
  logInfo(Serial, "Robot ready. Type HELP in Serial Monitor.");
}

void loop() {
  updateEncoders();  // read encoder values every loop
  serialInterfaceTick();
}
