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

// Encoder functions (from Encoder.ino)
void resetEncoders();
void printEncoderTelemetry();
long getEncoderA();
long getEncoderB();

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
      turnInPlace(sp, false); // CCW
      startTimed((unsigned long)ms);
      return;
    }

    if (c1 == "SPINR") {
      int sp = g_defaultSpeed;
      nextInt(u, i, sp);
      sp = clamp255(sp);
      logInfo(Serial, "OK SPINR");
      turnInPlace(sp, true); // CW
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

  logError(Serial, "Unknown command. Type HELP.");
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
