

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

// From Ultrasonic.ino
void initializeUltrasonic();
void sonarTick();

// From Photocell.ino
void initializePhotocell();
void photocellTick();

// From WallFollow.ino
void wallFollowTick();

// From Serial.ino
void logInfo(Stream& out, const char* msg);

// From CommandInterface.ino
void serialInterfaceTick();

// From WiFiAP.ino
void initializeWifiAp();
void wifiTick();

// From ObjectDetect.ino
void objectDetectInit();
void objectDetectTick();

void setup() {
  Serial.begin(9600);
  delay(2000);  // Give Serial time to initialize
  initializeMotors();
  initializeEncoders();
  initializeUltrasonic();
  initializePhotocell();
  initializeWifiAp();
  objectDetectInit();
}

void loop() {
  updateEncoders();        // read encoder values every loop
  sonarTick();             // stream sonar CSV if enabled
  photocellTick();         // photocell state machine
  wallFollowTick();        // P-controller: follow-me / wall-follow
  objectDetectTick();      // Lab 07: sweep + Bayes object tracking
  serialInterfaceTick();   // handle serial commands
  wifiTick();              // handle WiFi HTTP requests
}
