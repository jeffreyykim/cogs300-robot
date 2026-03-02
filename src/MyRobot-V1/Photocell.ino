/**
 * @file Photocell.ino
 * @brief IR reflectance sensor for floor/tape detection.
 *
 * Wiring (IR sensor module, e.g. TCRT5000):
 *   VCC -> 5V
 *   GND -> GND
 *   AO  -> A1   (analog output — used for threshold detection)
 *
 * Lower reading  = less IR reflected = darker surface (tape/line)
 * Higher reading = more IR reflected = lighter surface (floor)
 *
 * State machine (two parallel lines):
 *   State 0 — idle, waiting to be armed
 *   State 1 — armed, driving forward, watching for line 1
 *   State 2 — crossed line 1 (now over dark floor), watching for line 2
 *   State 3 — reached line 2, stopped
 */

const int PHOTOCELL_PIN = A1;

static int  ph_threshold = 600;   // below this = dark (tape/line)
static int  ph_state     = 0;     // 0=idle, 1=driving, 2=between lines, 3=stopped
static bool ph_enabled   = false;
static bool ph_lineFollow = false; // line-follow (bang-bang) mode

// -------- forward declarations --------
void forward(int speed);
void backward(int speed);
void stop();
void logInfo(Stream& out, const char* msg);

// -------- public API --------
void initializePhotocell() {
  pinMode(PHOTOCELL_PIN, INPUT);
  Serial.println("IR sensor initialized on A1");
}

int  getPhotocellMeasurement()    { return analogRead(PHOTOCELL_PIN); }
int  getPhotocellThreshold()      { return ph_threshold; }
void setPhotocellThreshold(int t) { ph_threshold = t; }
int  getPhotocellState()          { return ph_state; }
bool isPhotocellEnabled()         { return ph_enabled; }

void setPhotocellEnabled(bool en) {
  ph_enabled = en;
  ph_state   = en ? 1 : 0;
  if (en) forward(160);
  else    stop();
}

bool isLineFollowEnabled() { return ph_lineFollow; }

void setLineFollowEnabled(bool en) {
  ph_lineFollow = en;
  if (en) forward(150);
  else    stop();
}

// -------- line-follow --------
static void tickLineFollow() {
  int m = getPhotocellMeasurement();
  bool onLine = (m < ph_threshold);
  if (onLine) {
    forward(150);
  } else {
    stop();
  }
}

/**
 * @brief Call every loop iteration.
 *
 * State 1: driving — waiting to cross line 1 (bright)
 * State 2: between lines — waiting for floor to go dark, then bright again (line 2)
 * State 3: stopped on line 2
 *
 * Debounce: line 1 is confirmed once we see dark floor again after it,
 * so a single wide line doesn't accidentally count as two lines.
 */
void photocellTick() {
  if (ph_lineFollow) {
    tickLineFollow();
    return;
  }
  if (!ph_enabled) return;

  int m = getPhotocellMeasurement();
  bool onLine = (m < ph_threshold);

  if (ph_state == 1) {
    // Driving — waiting to hit line 1
    if (onLine) {
      ph_state = 2;
      logInfo(Serial, "IR: line 1 detected, continuing...");
    }
  } else if (ph_state == 2) {
    // Past line 1 — wait for floor to go dark first (debounce),
    // then detect line 2
    static bool seenDark = false;
    if (!onLine) {
      seenDark = true;  // confirmed we left line 1
    }
    if (seenDark && onLine) {
      stop();
      ph_state   = 3;
      ph_enabled = false;
      seenDark   = false;  // reset for next use
      logInfo(Serial, "IR: line 2 detected, stopped.");
    }
  }
  // State 3: done
}
