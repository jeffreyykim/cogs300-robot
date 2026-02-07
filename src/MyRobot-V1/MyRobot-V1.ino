

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

// From Photocell.ino
void initializePhotocell();
void photocellTick();


// From Serial.ino
void logInfo(Stream& out, const char* msg);

// From CommandInterface.ino
void serialInterfaceTick();

// From WiFiAP.ino
void initializeWifiAp();
void wifiTick();

void setup() {
  Serial.begin(9600);
  delay(2000);  // Give Serial time to initialize
  initializeMotors();
  initializeEncoders();
  initializePhotocell();
  initializeWifiAp();
}

void loop() {
  updateEncoders();        // read encoder values every loop
  serialInterfaceTick();   // handle serial commands
  photocellTick();         // photocell state machine (if enabled)
  wifiTick();              // handle WiFi HTTP requests
}
