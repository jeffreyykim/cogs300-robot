/**
 * @file CommandInterface.ino
 * @brief Serial Monitor command interface + test runner for COGS 300 robots.
 *
 * Usage (Serial Monitor, newline):
 *   HELP
 *   PING
 *   STOP
 *   SPEED 180
 *   FWD 800 200
 *   REV 800
 *   LEFT 600 150 50
 *   RIGHT 600
 *   SPINL 500 180
 *   SPINR 500
 *   TEST ALL
 *   LOOP 5 TEST ALL
 *   SONAR READ
 *   SONAR TELEM ON
 *   FOLLOW ON
 *   FOLLOW SET 25
 *   WALLFOLLOW ON
 *   WALLFOLLOW SIDE L
 *   WALLFOLLOW SET 20
 */

#include <Arduino.h>

// You already have these somewhere:
void logInfo(Stream& out, const char* msg);
void logError(Stream& out, const char* msg);

// Motor functions (from Motor.ino)
void forward(int speed);
void backward(int speed);
void stop();
void turnLeft(int baseSpeed, int speedBoost);
void turnRight(int baseSpeed, int speedBoost);
void turnInPlace(int speed, bool clockwise);
void testMotorAForward(int speed);
void testMotorABackward(int speed);
void testMotorBForward(int speed);
void testMotorBBackward(int speed);
void setMotorTrimLeft(int trim);
void setMotorTrimRight(int trim);
int getMotorTrimLeft();
int getMotorTrimRight();

// Encoder functions (from Encoder.ino)
void resetEncoders();
void printEncoderTelemetry();
long getEncoderA();
long getEncoderB();

// Ultrasonic functions (from Ultrasonic.ino)
float getFrontDistanceCm();
float getSideDistanceCm();
void  printUltrasonicTelemetry();
void  setSonarTelemetryEnabled(bool en);
bool  isSonarTelemetryEnabled();

// Photocell functions (from Photocell.ino)
void initializePhotocell();
int  getPhotocellMeasurement();
int  getPhotocellThreshold();
void setPhotocellThreshold(int t);
int  getPhotocellState();
bool isPhotocellEnabled();
void setPhotocellEnabled(bool en);
bool isLineFollowEnabled();
void setLineFollowEnabled(bool en);

// Wall-follow functions (from WallFollow.ino)
void  setFollowMeEnabled(bool en);
void  setWallFollowEnabled(bool en);
bool  isFollowMeEnabled();
bool  isWallFollowEnabled();
void  setWallFollowSetPoint(float cm);
float getWallFollowSetPoint();
void  setWallFollowKp(float kp);
float getWallFollowKp();
void  setWallFollowBaseSpeed(int sp);
int   getWallFollowBaseSpeed();
void  setWallOnLeft(bool left);
bool  isWallOnLeft();
void  setDriveAndAvoidEnabled(bool en);
bool  isDriveAndAvoidEnabled();
void  setObstacleThreshold(float cm);
float getObstacleThreshold();
void  setObstacleTurnMs(int ms);
int   getObstacleTurnMs();

// Object-detect functions (from ObjectDetect.ino)
void  setObjectDetectEnabled(bool en);
bool  isObjectDetectEnabled();
void  setOdSpinSpeed(int sp);
int   getOdSpinSpeed();
void  setOdMsPerStep(int ms);
int   getOdMsPerStep();
void  setOdSettleMs(int ms);
int   getOdSettleMs();
void  setOdDriveSpeed(int sp);
int   getOdDriveSpeed();
void  setOdDriveMs(int ms);
int   getOdDriveMs();
void  setOdObjectMaxCm(float c);
float getOdObjectMaxCm();
void  odPrintMap();
void  odPrintStatus();

// -------- settings --------
static int g_defaultSpeed = 200;
static int g_baseSpeed = 150;
static int g_boost = 50;
static bool g_telemetryEnabled = false;  // CSV telemetry streaming

// -------- line buffer --------
static String g_line;

// -------- timed action state --------
static bool g_actionRunning = false;
static unsigned long g_actionEndMs = 0;

static void startTimed(unsigned long ms) {
  g_actionRunning = true;
  g_actionEndMs = millis() + ms;
}

static void tickTimedStop() {
  if (g_actionRunning && (long)(millis() - g_actionEndMs) >= 0) {
    stop();
    g_actionRunning = false;
    logInfo(Serial, "DONE (timed action finished)");
  }
}

static void tickTelemetry() {
  if (g_telemetryEnabled) {
    printEncoderTelemetry();  // prints CSV: t_ms,encoderA,encoderB
  }
}

static int clamp255(int v) {
  if (v < 0) return 0;
  if (v > 255) return 255;
  return v;
}

// -------- parsing helpers --------
static String nextWord(const String& s, int& i) {
  while (i < (int)s.length() && s[i] == ' ') i++;
  int start = i;
  while (i < (int)s.length() && s[i] != ' ') i++;
  return s.substring(start, i);
}

static bool nextInt(const String& s, int& i, int& out) {
  while (i < (int)s.length() && s[i] == ' ') i++;
  if (i >= (int)s.length()) return false;

  bool neg = false;
  if (s[i] == '-') { neg = true; i++; }

  long val = 0;
  bool any = false;
  while (i < (int)s.length() && isDigit(s[i])) {
    any = true;
    val = val * 10 + (s[i] - '0');
    i++;
  }
  if (!any) return false;
  out = neg ? -val : val;
  return true;
}

// -------- help --------
static void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  HELP"));
  Serial.println(F("  PING"));
  Serial.println(F("  STOP"));
  Serial.println(F("  SPEED <0-255>              (sets default motor speed)"));
  Serial.println(F("  TRIM L|R <delta>           (adjust motor trim, +/-)"));
  Serial.println(F("  TRIM SHOW                  (show current trim)"));
  Serial.println(F("  TURNSET <base> <boost>     (sets default turn params)"));
  Serial.println(F("  FWD <ms> [speed]"));
  Serial.println(F("  REV <ms> [speed]"));
  Serial.println(F("  LEFT <ms> [base] [boost]"));
  Serial.println(F("  RIGHT <ms> [base] [boost]"));
  Serial.println(F("  SPINL <ms> [speed]"));
  Serial.println(F("  SPINR <ms> [speed]"));
  Serial.println(F("  TEST ALL"));
  Serial.println(F("  TEST MA_FWD [speed]        (test Motor A forward)"));
  Serial.println(F("  TEST MA_REV [speed]        (test Motor A backward)"));
  Serial.println(F("  TEST MB_FWD [speed]        (test Motor B forward)"));
  Serial.println(F("  TEST MB_REV [speed]        (test Motor B backward)"));
  Serial.println(F("  LOOP <n> TEST ALL"));
  Serial.println(F("  TELEMETRY ON|OFF           (stream encoder CSV data)"));
  Serial.println(F("  ENCODERS                   (show encoder counts)"));
  Serial.println(F("  RESET_ENC                  (reset encoder counts)"));
  Serial.println(F("  SONAR READ                 (single distance reading)"));
  Serial.println(F("  SONAR TELEM ON|OFF         (stream sonar CSV)"));
  Serial.println(F("  FOLLOW ON|OFF              (follow-me mode, front sensor)"));
  Serial.println(F("  FOLLOW SET <cm>            (follow-me target distance)"));
  Serial.println(F("  FOLLOW KP <gain*10>        (P gain x10, e.g. 60 = 6.0)"));
  Serial.println(F("  WALLFOLLOW ON|OFF          (wall-follow mode, side sensor)"));
  Serial.println(F("  WALLFOLLOW SIDE L|R        (which side the wall is on)"));
  Serial.println(F("  WALLFOLLOW SET <cm>        (wall distance target)"));
  Serial.println(F("  WALLFOLLOW SPEED <0-255>   (base forward speed)"));
  Serial.println(F("  WALLFOLLOW KP <gain*10>    (P gain x10, e.g. 60 = 6.0)"));
  Serial.println(F("  OBSTACLE SET <cm>          (front obstacle turn threshold)"));
  Serial.println(F("  OBSTACLE READ              (show current settings)"));
  Serial.println(F("  OBSTACLE TURN <ms>         (spin duration per obstacle, default 300)"));
  Serial.println(F("  AVOID ON|OFF               (drive forward, turn right on obstacle)"));
  Serial.println(F("  PHOTO START|STOP           (arm/disarm stop-on-second-line)"));
  Serial.println(F("  PHOTO FOLLOW ON|OFF        (line-follow mode)"));
  Serial.println(F("  PHOTO THRESH <0-1023>      (set brightness threshold)"));
  Serial.println(F("  PHOTO READ                 (print measurement, threshold, state)"));
  Serial.println(F("  SCAN START|STOP            (object-detect sweep loop)"));
  Serial.println(F("  SCAN READ                  (print depth map + Bayes probs)"));
  Serial.println(F("  SCAN STATUS                (print all SCAN parameters)"));
  Serial.println(F("  SCAN SPEED  <0-255>        (spin speed for sweep)"));
  Serial.println(F("  SCAN STEP   <ms>           (ms to rotate 22.5 deg, calibrate!)"));
  Serial.println(F("  SCAN SETTLE <ms>           (settle wait before each reading)"));
  Serial.println(F("  SCAN DRIVE  <ms>           (drive duration per approach)"));
  Serial.println(F("  SCAN DSPEED <0-255>        (drive speed when approaching)"));
  Serial.println(F("  SCAN MAXCM  <cm>           (close-reading threshold)"));
  Serial.println();
  Serial.println(F("Serial Monitor: 9600 baud, Newline line ending"));
}

// -------- tests --------
static void testAllOnce() {
  Serial.println();
  logInfo(Serial, "=== TEST ALL START ===");

  logInfo(Serial, "TEST: forward 600ms");
  forward(g_defaultSpeed);
  delay(600);
  stop();
  delay(200);

  logInfo(Serial, "TEST: backward 600ms");
  backward(g_defaultSpeed);
  delay(600);
  stop();
  delay(200);

  logInfo(Serial, "TEST: left turn 600ms");
  turnLeft(g_baseSpeed, g_boost);
  delay(600);
  stop();
  delay(200);

  logInfo(Serial, "TEST: right turn 600ms");
  turnRight(g_baseSpeed, g_boost);
  delay(600);
  stop();
  delay(200);

  logInfo(Serial, "TEST: spin left 450ms");
  turnInPlace(180, false); // false = CCW (spin left) in your implementation
  delay(450);
  stop();
  delay(200);

  logInfo(Serial, "TEST: spin right 450ms");
  turnInPlace(180, true);  // true = CW
  delay(450);
  stop();

  logInfo(Serial, "=== TEST ALL END ===");
  Serial.println();
}

// -------- command handler --------
static void handleCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  String u = cmd;
  u.toUpperCase();

  int i = 0;
  String c1 = nextWord(u, i);

  if (c1 == "HELP") { printHelp(); return; }

  if (c1 == "PING") {
    Serial.print(F("PONG uptime(ms)="));
    Serial.print(millis());
    Serial.print(F(" speed="));
    Serial.print(g_defaultSpeed);
    Serial.print(F(" turnBase="));
    Serial.print(g_baseSpeed);
    Serial.print(F(" boost="));
    Serial.println(g_boost);
    return;
  }

  if (c1 == "STOP") {
    stop();
    g_actionRunning = false;
    logInfo(Serial, "OK STOP");
    return;
  }

  if (c1 == "SPEED") {
    int sp;
    if (!nextInt(u, i, sp)) { logError(Serial, "SPEED needs <0-255>"); return; }
    g_defaultSpeed = clamp255(sp);
    logInfo(Serial, "OK updated default speed");
    Serial.print(F("defaultSpeed=")); Serial.println(g_defaultSpeed);
    return;
  }

  if (c1 == "TURNSET") {
    int b, k;
    if (!nextInt(u, i, b) || !nextInt(u, i, k)) { logError(Serial, "TURNSET needs <base> <boost>"); return; }
    g_baseSpeed = clamp255(b);
    g_boost = clamp255(k);
    logInfo(Serial, "OK updated turn parameters");
    return;
  }

  if (c1 == "TRIM") {
    String c2 = nextWord(u, i);
    if (c2 == "SHOW") {
      Serial.print(F("trimLeft="));
      Serial.print(getMotorTrimLeft());
      Serial.print(F(" trimRight="));
      Serial.println(getMotorTrimRight());
      return;
    }
    if (c2 == "L" || c2 == "R") {
      int d;
      if (!nextInt(u, i, d)) { logError(Serial, "TRIM L|R needs <delta>"); return; }
      if (c2 == "L") {
        setMotorTrimLeft(d);
        Serial.print(F("trimLeft="));
        Serial.println(getMotorTrimLeft());
      } else {
        setMotorTrimRight(d);
        Serial.print(F("trimRight="));
        Serial.println(getMotorTrimRight());
      }
      logInfo(Serial, "OK trim updated");
      return;
    }
    logError(Serial, "Usage: TRIM L|R <delta> or TRIM SHOW");
    return;
  }

  // Movement: <ms> [speed/base] [boost]
  if (c1 == "FWD" || c1 == "REV" || c1 == "LEFT" || c1 == "RIGHT" || c1 == "SPINL" || c1 == "SPINR") {
    int ms;
    if (!nextInt(u, i, ms) || ms <= 0) { logError(Serial, "Usage: <CMD> <ms> ..."); return; }

    if (c1 == "FWD") {
      int sp = g_defaultSpeed;
      nextInt(u, i, sp);
      sp = clamp255(sp);
      logInfo(Serial, "OK FWD");
      forward(sp);
      startTimed((unsigned long)ms);
      return;
    }

    if (c1 == "REV") {
      int sp = g_defaultSpeed;
      nextInt(u, i, sp);
      sp = clamp255(sp);
      logInfo(Serial, "OK REV");
      backward(sp);
      startTimed((unsigned long)ms);
      return;
    }

    if (c1 == "LEFT") {
      int base = g_baseSpeed, boost = g_boost;
      nextInt(u, i, base);
      nextInt(u, i, boost);
      base = clamp255(base);
      boost = clamp255(boost);
      logInfo(Serial, "OK LEFT");
      turnLeft(base, boost);
      startTimed((unsigned long)ms);
      return;
    }

    if (c1 == "RIGHT") {
      int base = g_baseSpeed, boost = g_boost;
      nextInt(u, i, base);
      nextInt(u, i, boost);
      base = clamp255(base);
      boost = clamp255(boost);
      logInfo(Serial, "OK RIGHT");
      turnRight(base, boost);
      startTimed((unsigned long)ms);
      return;
    }

    if (c1 == "SPINL") {
      int sp = g_defaultSpeed;
      nextInt(u, i, sp);
      sp = clamp255(sp);
      logInfo(Serial, "OK SPINL");
      turnInPlace(sp, true); // CW
      startTimed((unsigned long)ms);
      return;
    }

    if (c1 == "SPINR") {
      int sp = g_defaultSpeed;
      nextInt(u, i, sp);
      sp = clamp255(sp);
      logInfo(Serial, "OK SPINR");
      turnInPlace(sp, false); // CCW
      startTimed((unsigned long)ms);
      return;
    }
  }

  if (c1 == "TEST") {
    String c2 = nextWord(u, i);
    if (c2 == "ALL") { testAllOnce(); return; }
    
    // Individual motor tests
    if (c2 == "MA_FWD") {
      int sp = g_defaultSpeed;
      nextInt(u, i, sp);
      sp = clamp255(sp);
      testMotorAForward(sp);
      return;
    }
    if (c2 == "MA_REV") {
      int sp = g_defaultSpeed;
      nextInt(u, i, sp);
      sp = clamp255(sp);
      testMotorABackward(sp);
      return;
    }
    if (c2 == "MB_FWD") {
      int sp = g_defaultSpeed;
      nextInt(u, i, sp);
      sp = clamp255(sp);
      testMotorBForward(sp);
      return;
    }
    if (c2 == "MB_REV") {
      int sp = g_defaultSpeed;
      nextInt(u, i, sp);
      sp = clamp255(sp);
      testMotorBBackward(sp);
      return;
    }
    
    logError(Serial, "Usage: TEST ALL or TEST MA_FWD/MA_REV/MB_FWD/MB_REV [speed]");
    return;
  }

  if (c1 == "LOOP") {
    int n;
    if (!nextInt(u, i, n) || n <= 0) { logError(Serial, "Usage: LOOP <n> TEST ALL"); return; }
    String c2 = nextWord(u, i);
    String c3 = nextWord(u, i);
    if (c2 == "TEST" && c3 == "ALL") {
      for (int k = 1; k <= n; k++) {
        Serial.print(F("## LOOP ")); Serial.print(k); Serial.print(F("/")); Serial.println(n);
        testAllOnce();
        delay(250);
      }
      return;
    }
    logError(Serial, "Usage: LOOP <n> TEST ALL");
    return;
  }

  if (c1 == "TELEMETRY") {
    String c2 = nextWord(u, i);
    if (c2 == "ON") {
      g_telemetryEnabled = true;
      Serial.println("t_ms,encoderA,encoderB");  // CSV header
      logInfo(Serial, "Telemetry ON (CSV streaming)");
      return;
    }
    if (c2 == "OFF") {
      g_telemetryEnabled = false;
      logInfo(Serial, "Telemetry OFF");
      return;
    }
    logError(Serial, "Usage: TELEMETRY ON|OFF");
    return;
  }

  if (c1 == "ENCODERS") {
    Serial.print("Encoder A (left): ");
    Serial.print(getEncoderA());
    Serial.print(" | Encoder B (right): ");
    Serial.println(getEncoderB());
    return;
  }

  if (c1 == "RESET_ENC") {
    resetEncoders();
    logInfo(Serial, "OK encoder counts reset");
    return;
  }

  if (c1 == "SONAR") {
    String c2 = nextWord(u, i);
    if (c2 == "READ") {
      float front = getFrontDistanceCm();
      float side  = getSideDistanceCm();
      Serial.print(F("front_cm="));
      if (front < 0) Serial.print(F("out_of_range"));
      else           Serial.print(front);
      Serial.print(F(" side_cm="));
      if (side < 0) Serial.println(F("out_of_range"));
      else          Serial.println(side);
      return;
    }
    if (c2 == "TELEM") {
      String c3 = nextWord(u, i);
      if (c3 == "ON")  { setSonarTelemetryEnabled(true);  logInfo(Serial, "Sonar telemetry ON");  return; }
      if (c3 == "OFF") { setSonarTelemetryEnabled(false); logInfo(Serial, "Sonar telemetry OFF"); return; }
    }
    logError(Serial, "Usage: SONAR READ | SONAR TELEM ON|OFF");
    return;
  }

  if (c1 == "FOLLOW") {
    String c2 = nextWord(u, i);
    if (c2 == "ON")  { setFollowMeEnabled(true);  logInfo(Serial, "Follow-me ON");  return; }
    if (c2 == "OFF") { setFollowMeEnabled(false); logInfo(Serial, "Follow-me OFF"); return; }
    if (c2 == "SET") {
      int cm;
      if (!nextInt(u, i, cm)) { logError(Serial, "FOLLOW SET needs <cm>"); return; }
      setWallFollowSetPoint((float)cm);
      Serial.print(F("followSetPoint=")); Serial.println(cm);
      return;
    }
    if (c2 == "KP") {
      int kp10;
      if (!nextInt(u, i, kp10)) { logError(Serial, "FOLLOW KP needs <gain*10>"); return; }
      setWallFollowKp(kp10 / 10.0);
      Serial.print(F("kp=")); Serial.println(kp10 / 10.0);
      return;
    }
    logError(Serial, "Usage: FOLLOW ON|OFF|SET <cm>|KP <gain*10>");
    return;
  }

  if (c1 == "WALLFOLLOW") {
    String c2 = nextWord(u, i);
    if (c2 == "ON")  { setWallFollowEnabled(true);  logInfo(Serial, "Wall-follow ON");  return; }
    if (c2 == "OFF") { setWallFollowEnabled(false); logInfo(Serial, "Wall-follow OFF"); return; }
    if (c2 == "SIDE") {
      String c3 = nextWord(u, i);
      if (c3 == "L") { setWallOnLeft(true);  logInfo(Serial, "Wall side: LEFT");  return; }
      if (c3 == "R") { setWallOnLeft(false); logInfo(Serial, "Wall side: RIGHT"); return; }
      logError(Serial, "Usage: WALLFOLLOW SIDE L|R");
      return;
    }
    if (c2 == "SET") {
      int cm;
      if (!nextInt(u, i, cm)) { logError(Serial, "WALLFOLLOW SET needs <cm>"); return; }
      setWallFollowSetPoint((float)cm);
      Serial.print(F("wallSetPoint=")); Serial.println(cm);
      return;
    }
    if (c2 == "SPEED") {
      int sp;
      if (!nextInt(u, i, sp)) { logError(Serial, "WALLFOLLOW SPEED needs <0-255>"); return; }
      setWallFollowBaseSpeed(clamp255(sp));
      Serial.print(F("wallBaseSpeed=")); Serial.println(getWallFollowBaseSpeed());
      return;
    }
    if (c2 == "KP") {
      int kp10;
      if (!nextInt(u, i, kp10)) { logError(Serial, "WALLFOLLOW KP needs <gain*10>"); return; }
      setWallFollowKp(kp10 / 10.0);
      Serial.print(F("kp=")); Serial.println(kp10 / 10.0);
      return;
    }
    logError(Serial, "Usage: WALLFOLLOW ON|OFF|SIDE L|R|SET <cm>|SPEED <0-255>|KP <gain*10>");
    return;
  }

  if (c1 == "OBSTACLE") {
    String c2 = nextWord(u, i);
    if (c2 == "READ") {
      Serial.print(F("obstacleThreshold_cm="));
      Serial.print(getObstacleThreshold());
      Serial.print(F(" turnDuration_ms="));
      Serial.println(getObstacleTurnMs());
      return;
    }
    if (c2 == "SET") {
      int cm;
      if (!nextInt(u, i, cm)) { logError(Serial, "OBSTACLE SET needs <cm>"); return; }
      setObstacleThreshold((float)cm);
      Serial.print(F("obstacleThreshold_cm=")); Serial.println(cm);
      return;
    }
    if (c2 == "TURN") {
      int ms;
      if (!nextInt(u, i, ms)) { logError(Serial, "OBSTACLE TURN needs <ms>"); return; }
      setObstacleTurnMs(ms);
      Serial.print(F("turnDuration_ms=")); Serial.println(ms);
      return;
    }
    logError(Serial, "Usage: OBSTACLE SET <cm> | OBSTACLE TURN <ms> | OBSTACLE READ");
    return;
  }

  if (c1 == "AVOID") {
    String c2 = nextWord(u, i);
    if (c2 == "ON")  { setDriveAndAvoidEnabled(true);  logInfo(Serial, "Drive-and-avoid ON");  return; }
    if (c2 == "OFF") { setDriveAndAvoidEnabled(false); logInfo(Serial, "Drive-and-avoid OFF"); return; }
    logError(Serial, "Usage: AVOID ON|OFF");
    return;
  }

  if (c1 == "PHOTO") {
    String c2 = nextWord(u, i);
    if (c2 == "START") {
      setPhotocellEnabled(true);
      logInfo(Serial, "Stop-on-line armed — robot must be moving");
      return;
    }
    if (c2 == "STOP") {
      setPhotocellEnabled(false);
      setLineFollowEnabled(false);
      logInfo(Serial, "Photocell disarmed");
      return;
    }
    if (c2 == "FOLLOW") {
      String c3 = nextWord(u, i);
      if (c3 == "ON")  { setLineFollowEnabled(true);  logInfo(Serial, "Line-follow ON");  return; }
      if (c3 == "OFF") { setLineFollowEnabled(false); logInfo(Serial, "Line-follow OFF"); return; }
      logError(Serial, "Usage: PHOTO FOLLOW ON|OFF");
      return;
    }
    if (c2 == "THRESH") {
      int t;
      if (!nextInt(u, i, t)) { logError(Serial, "PHOTO THRESH needs <0-1023>"); return; }
      setPhotocellThreshold(t);
      Serial.print(F("photocellThreshold=")); Serial.println(t);
      return;
    }
    if (c2 == "READ") {
      Serial.print(F("measurement="));
      Serial.print(getPhotocellMeasurement());
      Serial.print(F(" threshold="));
      Serial.print(getPhotocellThreshold());
      Serial.print(F(" stopOnLine="));
      Serial.print(isPhotocellEnabled() ? 1 : 0);
      Serial.print(F(" lineFollow="));
      Serial.print(isLineFollowEnabled() ? 1 : 0);
      Serial.print(F(" state="));
      Serial.println(getPhotocellState());
      return;
    }
    logError(Serial, "Usage: PHOTO START|STOP|FOLLOW ON|OFF|THRESH <0-1023>|READ");
    return;
  }

  if (c1 == "SCAN") {
    String c2 = nextWord(u, i);
    if (c2 == "START") {
      setObjectDetectEnabled(true);
      logInfo(Serial, "Object-detect ON");
      return;
    }
    if (c2 == "STOP") {
      setObjectDetectEnabled(false);
      logInfo(Serial, "Object-detect OFF");
      return;
    }
    if (c2 == "READ") {
      odPrintMap();
      return;
    }
    if (c2 == "STATUS") {
      odPrintStatus();
      return;
    }
    if (c2 == "SPEED") {
      int sp;
      if (!nextInt(u, i, sp)) { logError(Serial, "SCAN SPEED needs <0-255>"); return; }
      setOdSpinSpeed(clamp255(sp));
      Serial.print(F("odSpinSpeed=")); Serial.println(getOdSpinSpeed());
      return;
    }
    if (c2 == "STEP") {
      int ms;
      if (!nextInt(u, i, ms)) { logError(Serial, "SCAN STEP needs <ms>"); return; }
      setOdMsPerStep(ms);
      Serial.print(F("odMsPerStep=")); Serial.println(getOdMsPerStep());
      return;
    }
    if (c2 == "SETTLE") {
      int ms;
      if (!nextInt(u, i, ms)) { logError(Serial, "SCAN SETTLE needs <ms>"); return; }
      setOdSettleMs(ms);
      Serial.print(F("odSettleMs=")); Serial.println(getOdSettleMs());
      return;
    }
    if (c2 == "DRIVE") {
      int ms;
      if (!nextInt(u, i, ms)) { logError(Serial, "SCAN DRIVE needs <ms>"); return; }
      setOdDriveMs(ms);
      Serial.print(F("odDriveMs=")); Serial.println(getOdDriveMs());
      return;
    }
    if (c2 == "DSPEED") {
      int sp;
      if (!nextInt(u, i, sp)) { logError(Serial, "SCAN DSPEED needs <0-255>"); return; }
      setOdDriveSpeed(clamp255(sp));
      Serial.print(F("odDriveSpeed=")); Serial.println(getOdDriveSpeed());
      return;
    }
    if (c2 == "MAXCM") {
      int cm;
      if (!nextInt(u, i, cm)) { logError(Serial, "SCAN MAXCM needs <cm>"); return; }
      setOdObjectMaxCm((float)cm);
      Serial.print(F("odObjectMaxCm=")); Serial.println(getOdObjectMaxCm(), 1);
      return;
    }
    logError(Serial, "Usage: SCAN START|STOP|READ|STATUS|SPEED|STEP|SETTLE|DRIVE|DSPEED|MAXCM");
    return;
  }

  logError(Serial, "Unknown command. Type HELP.");
}

// Exposed wrapper for other modules (e.g., WiFi AP control)
void commandInterfaceHandleCommand(const String& cmd) {
  handleCommand(cmd);
}

// -------- called from main loop --------
void serialInterfaceTick() {
  tickTimedStop();
  tickTelemetry();  // stream encoder data if enabled

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      handleCommand(g_line);
      g_line = "";
    } else if (c != '\r') {
      g_line += c;
      if (g_line.length() > 120) {
        g_line = "";
        logError(Serial, "Line too long");
      }
    }
  }
}
