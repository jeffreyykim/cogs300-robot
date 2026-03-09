/**
 * @file ObjectDetect.ino
 * @brief Lab 07 – Object detection via whole-robot sweep + Bayes filter.
 *
 * Strategy
 * --------
 * The robot uses the front-facing ultrasonic sensor and its own drive motors
 * to perform a 180° clockwise sweep in OD_NUM_STEPS equal angular steps.
 * Each step records one distance measurement into a depth map.  After the
 * sweep, a Bayes filter and local-minimum detector identify the bin most
 * likely to contain the object (vs. a flat wall).  The robot then spins back
 * (CCW) to face that bearing and drives forward for od_driveMs milliseconds
 * before repeating the whole cycle.
 *
 * Object vs. wall discrimination
 * --------------------------------
 * Wall   → depth map values form a smooth arc (all increase or all decrease).
 * Object → creates a local minimum "bump" that is lower than its neighbours.
 *
 * Bayes filter update rule (per bin, per sweep)
 * -----------------------------------------------
 *   P_new = P(close | H) × P_old
 *           ──────────────────────
 *           P(close | H) × P_old + P(close | ¬H) × (1 - P_old)
 *
 *   where H = "object present in this bin".
 *   P(close | object) = OD_LHOBJ     (high – object is close)
 *   P(close | wall)   = OD_LHWALL    (low  – wall is far at that angle)
 *
 * Tune OD_MS_PER_STEP so that spinning at OD_SPIN_SPEED for that many
 * milliseconds rotates the robot exactly OD_STEP_DEG degrees.
 *
 * Serial commands (via CommandInterface):
 *   SCAN START          – begin sweep/track loop
 *   SCAN STOP           – stop and motors off
 *   SCAN READ           – print current depth map & Bayes probs
 *   SCAN STATUS         – print all tunable parameters
 *   SCAN SPEED  <0-255> – spin-in-place PWM speed
 *   SCAN STEP   <ms>    – ms to rotate one step (22.5° calibration)
 *   SCAN SETTLE <ms>    – vibration-settle wait before each reading
 *   SCAN DRIVE  <ms>    – drive time per approach step
 *   SCAN DSPEED <0-255> – forward drive speed
 *   SCAN MAXCM  <cm>    – distance threshold "close = possible object"
 *   SCAN TOUCH  <cm>    – stop and declare success when this close (default 8)
 */

// ---- forward declarations (other .ino files) --------
float getFrontDistanceCm();
void  forward(int speed);
void  stop();
void  turnInPlace(int speed, bool clockwise);
void  setFollowMeEnabled(bool en);
void  setWallFollowEnabled(bool en);
void  setDriveAndAvoidEnabled(bool en);

// ---- sweep / depth-map configuration ----
static const int   OD_NUM_STEPS  = 8;       // 8 × 22.5° = 180° sweep
static const float OD_STEP_DEG   = 22.5f;

static int   od_spinSpeed  = 160;   // PWM for in-place spin (tune per robot)
static int   od_msPerStep  = 125;   // ms to spin exactly 22.5° (calibrate!)
static int   od_settleMs   = 80;    // vibration settle before each reading (ms)
static int   od_driveSpeed  = 180;   // forward speed when approaching object
static int   od_driveMs     = 700;   // how long to drive per approach burst (ms)
static float od_objectMaxCm  = 80.0f; // distances below this count as "close"
static float od_touchCm      = 8.0f;  // stop and declare success when this close
static float od_driveMinProb = 0.65f; // min Bayes prob required before driving

// ---- Bayes filter likelihoods ----
static const float OD_LHOBJ  = 0.80f; // P(close reading | object present)
static const float OD_LHWALL = 0.25f; // P(close reading | wall = no object)

// ---- runtime state ----
static float od_depthMap[OD_NUM_STEPS];   // measured distance per bin (cm, -1 = OOR)
static float od_bayesProb[OD_NUM_STEPS];  // P(object present) per bin [0,1]
static bool  od_enabled  = false;

// ---- state machine ----
enum OdState : uint8_t {
  OD_IDLE      = 0,
  OD_SPIN_STEP,   // spinning CW for one angular step
  OD_SETTLE,      // waiting for vibrations to settle
  OD_ALIGNING,    // spinning CCW to face the target bin
  OD_DRIVING,     // driving straight toward the target
};

static OdState       od_state     = OD_IDLE;
static int           od_stepIdx   = 0;       // current sweep bin (0..NUM_STEPS-1)
static int           od_targetIdx = -1;      // best-target bin after sweep
static unsigned long od_timerMs   = 0;


// ─────────────────────────────────────────────────────────────────────────────
// Internal helpers
// ─────────────────────────────────────────────────────────────────────────────

/** Bayesian update for one bin given a new distance measurement. */
static void bayesUpdate(int idx, float dist) {
  float p = od_bayesProb[idx];

  if (dist < 0.0f) {
    // No echo – gentle pull back toward prior (0.5)
    p = p * 0.88f + 0.06f;
  } else if (dist <= od_objectMaxCm) {
    // Close reading: evidence for object
    float num   = OD_LHOBJ * p;
    float denom = num + OD_LHWALL * (1.0f - p);
    p = num / denom;
  } else {
    // Far reading: evidence against object (update with complement likelihoods)
    float lhFarObj  = 1.0f - OD_LHOBJ;
    float lhFarWall = 1.0f - OD_LHWALL;
    float num   = lhFarObj * p;
    float denom = num + lhFarWall * (1.0f - p);
    p = num / denom;
  }

  // Clamp to avoid degeneracy
  if (p < 0.02f) p = 0.02f;
  if (p > 0.98f) p = 0.98f;

  od_bayesProb[idx] = p;
}

/**
 * Find the best target bin after a sweep.
 *
 * Priority:
 *   1. Local minimum in depth map AND Bayes probability >= 0.5
 *      (object creates a "bump" in the scan – smaller than both neighbours)
 *   2. Any local minimum regardless of Bayes probability
 *   3. Bayes-weighted closest valid reading
 *
 * Returns -1 if no valid readings exist.
 */
static int findBestTarget() {
  // -- pass 1: local minima that Bayes also endorses --
  int   localMinIdx  = -1;
  float localMinDist = 9999.0f;

  for (int i = 0; i < OD_NUM_STEPS; i++) {
    float d = od_depthMap[i];
    if (d < 0.0f) continue;

    float dL = (i > 0)             ? od_depthMap[i - 1] : 9999.0f;
    float dR = (i < OD_NUM_STEPS-1)? od_depthMap[i + 1] : 9999.0f;

    // Treat OOR neighbours as "far" so endpoints can qualify
    if (dL < 0.0f) dL = 9999.0f;
    if (dR < 0.0f) dR = 9999.0f;

    bool isLocalMin = (d < dL && d < dR);
    if (isLocalMin && od_bayesProb[i] >= 0.50f && d < localMinDist) {
      localMinDist = d;
      localMinIdx  = i;
    }
  }
  if (localMinIdx >= 0) return localMinIdx;

  // -- pass 2: any local minimum (Bayes not strong enough yet) --
  float localMinDist2 = 9999.0f;
  int   localMinIdx2  = -1;
  for (int i = 0; i < OD_NUM_STEPS; i++) {
    float d  = od_depthMap[i];
    if (d < 0.0f) continue;
    float dL = (i > 0)              ? od_depthMap[i-1] : 9999.0f;
    float dR = (i < OD_NUM_STEPS-1) ? od_depthMap[i+1] : 9999.0f;
    if (dL < 0.0f) dL = 9999.0f;
    if (dR < 0.0f) dR = 9999.0f;
    if (d < dL && d < dR && d < localMinDist2) {
      localMinDist2 = d;
      localMinIdx2  = i;
    }
  }
  if (localMinIdx2 >= 0) return localMinIdx2;

  // -- pass 3: Bayes-weighted closest --
  float bestScore = -1.0f;
  int   bestIdx   = -1;
  for (int i = 0; i < OD_NUM_STEPS; i++) {
    float d = od_depthMap[i];
    if (d < 0.0f) continue;
    float score = od_bayesProb[i] / (d + 1.0f);
    if (score > bestScore) {
      bestScore = score;
      bestIdx   = i;
    }
  }
  return bestIdx;
}

/** Print depth map and Bayes probabilities as a CSV table. */
static void printDepthMap() {
  Serial.println(F("--- depth map ---"));
  Serial.println(F("idx,deg,dist_cm,bayes_prob"));
  for (int i = 0; i < OD_NUM_STEPS; i++) {
    Serial.print(i);
    Serial.print(F(","));
    Serial.print((int)(i * OD_STEP_DEG));
    Serial.print(F(","));
    if (od_depthMap[i] < 0.0f) Serial.print(F("OOR"));
    else                        Serial.print(od_depthMap[i], 1);
    Serial.print(F(","));
    Serial.println(od_bayesProb[i], 3);
  }
  Serial.println(F("-----------------"));
}

/** Kick off a fresh sweep from step 0. */
static void startSweep() {
  od_stepIdx = 0;
  od_state   = OD_SPIN_STEP;
  od_timerMs = millis();
  turnInPlace(od_spinSpeed, true);   // CW
  Serial.println(F("ObjectDetect: sweep START"));
}


// ─────────────────────────────────────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────────────────────────────────────

/** Call once in setup(). */
void objectDetectInit() {
  for (int i = 0; i < OD_NUM_STEPS; i++) {
    od_depthMap[i]  = -1.0f;
    od_bayesProb[i] = 0.5f;
  }
}

/** Enable / disable the object-detection mode. */
void setObjectDetectEnabled(bool en) {
  od_enabled = en;
  if (en) {
    // Disable other drive modes so they don't fight us
    setFollowMeEnabled(false);
    setWallFollowEnabled(false);
    setDriveAndAvoidEnabled(false);
    startSweep();
  } else {
    od_state = OD_IDLE;
    stop();
    Serial.println(F("ObjectDetect: OFF"));
  }
}

bool isObjectDetectEnabled() { return od_enabled; }

/** Main tick – call every loop iteration. */
void objectDetectTick() {
  if (!od_enabled) return;

  unsigned long now = millis();

  switch (od_state) {

    // ── spinning one angular step (CW) ──────────────────────────────────────
    case OD_SPIN_STEP:
      if ((long)(now - od_timerMs) >= (long)od_msPerStep) {
        stop();
        od_timerMs = millis();
        od_state   = OD_SETTLE;
      }
      break;

    // ── waiting for vibrations before sampling ───────────────────────────────
    case OD_SETTLE: {
      if ((long)(now - od_timerMs) < (long)od_settleMs) break;

      // Take reading & update Bayes
      float d = getFrontDistanceCm();
      od_depthMap[od_stepIdx] = d;
      bayesUpdate(od_stepIdx, d);

      od_stepIdx++;

      if (od_stepIdx < OD_NUM_STEPS) {
        // More steps remaining – spin next step
        od_timerMs = millis();
        od_state   = OD_SPIN_STEP;
        turnInPlace(od_spinSpeed, true);  // CW
      } else {
        // Sweep complete – analyse and align
        printDepthMap();
        od_targetIdx = findBestTarget();

        if (od_targetIdx < 0) {
          Serial.println(F("ObjectDetect: no valid readings – re-scanning"));
          // Spin all the way back CCW then start fresh
          int returnSteps = OD_NUM_STEPS;
          od_timerMs = millis();
          od_state   = OD_ALIGNING;
          od_targetIdx = 0;  // will end at step 0 = start heading
          turnInPlace(od_spinSpeed, false);   // CCW
          // Store total align time in od_stepIdx (reuse variable)
          od_stepIdx = returnSteps;
        } else {
          bool confident = (od_bayesProb[od_targetIdx] >= od_driveMinProb);

          Serial.print(F("ObjectDetect: target step="));
          Serial.print(od_targetIdx);
          Serial.print(F("  ("));
          Serial.print((int)(od_targetIdx * OD_STEP_DEG));
          Serial.println(F(" deg CW from start)"));
          Serial.print(F("  dist="));
          Serial.print(od_depthMap[od_targetIdx], 1);
          Serial.print(F("  prob="));
          Serial.println(od_bayesProb[od_targetIdx], 3);

          // Not confident enough – spin back and re-scan
          if (!confident) {
            Serial.println(F("ObjectDetect: prob too low – re-scanning"));
            int returnSteps = OD_NUM_STEPS - 1;
            od_stepIdx  = returnSteps;
            od_timerMs  = millis();
            od_state    = OD_ALIGNING;
            turnInPlace(od_spinSpeed, false);  // CCW back to start
            break;
          }

          // After a full CW sweep we are now (OD_NUM_STEPS-1) steps CW of start.
          // Need to spin (OD_NUM_STEPS-1 - targetIdx) steps CCW to face target.
          int returnSteps = (OD_NUM_STEPS - 1) - od_targetIdx;
          od_stepIdx = returnSteps;  // reuse as countdown

          if (returnSteps == 0) {
            // Already facing target
            od_state   = OD_DRIVING;
            od_timerMs = millis();
            forward(od_driveSpeed);
            Serial.println(F("ObjectDetect: already aligned – driving"));
          } else {
            od_timerMs = millis();
            od_state   = OD_ALIGNING;
            turnInPlace(od_spinSpeed, false);  // CCW
          }
        }
      }
      break;
    }

    // ── spinning back CCW to face the target bin ─────────────────────────────
    case OD_ALIGNING: {
      // Each od_msPerStep ms = one step of CCW rotation
      if ((long)(now - od_timerMs) < (long)od_msPerStep) break;

      od_stepIdx--;
      if (od_stepIdx <= 0) {
        stop();
        od_state   = OD_DRIVING;
        od_timerMs = millis();
        forward(od_driveSpeed);
        Serial.println(F("ObjectDetect: aligned – driving"));
      } else {
        // One more step – reset timer and keep spinning
        od_timerMs = millis();
      }
      break;
    }

    // ── driving toward the object ────────────────────────────────────────────
    case OD_DRIVING: {
      // Timed burst expired – re-scan
      if ((long)(now - od_timerMs) >= (long)od_driveMs) {
        stop();
        Serial.println(F("ObjectDetect: drive done – re-scanning"));
        startSweep();
      }
      break;
    }

    case OD_IDLE:
    default:
      break;
  }
}


// ─────────────────────────────────────────────────────────────────────────────
// Setters / getters (used by CommandInterface)
// ─────────────────────────────────────────────────────────────────────────────

void  setOdSpinSpeed(int sp)   { od_spinSpeed  = sp; }
int   getOdSpinSpeed()          { return od_spinSpeed; }

void  setOdMsPerStep(int ms)   { od_msPerStep  = ms; }
int   getOdMsPerStep()          { return od_msPerStep; }

void  setOdSettleMs(int ms)    { od_settleMs   = ms; }
int   getOdSettleMs()           { return od_settleMs; }

void  setOdDriveSpeed(int sp)  { od_driveSpeed = sp; }
int   getOdDriveSpeed()         { return od_driveSpeed; }

void  setOdDriveMs(int ms)     { od_driveMs    = ms; }
int   getOdDriveMs()            { return od_driveMs; }

void  setOdObjectMaxCm(float c){ od_objectMaxCm = c; }
float getOdObjectMaxCm()        { return od_objectMaxCm; }

void  setOdTouchCm(float c)    { od_touchCm = c; }
float getOdTouchCm()            { return od_touchCm; }

void  setOdDriveMinProb(float p){ od_driveMinProb = p; }
float getOdDriveMinProb()        { return od_driveMinProb; }

/** Print depth map + Bayes probabilities (for manual inspection). */
void odPrintMap() {
  printDepthMap();
}

/** Print all tunable parameters. */
void odPrintStatus() {
  Serial.print(F("od_enabled="));     Serial.println(od_enabled   ? "ON" : "OFF");
  Serial.print(F("state="));          Serial.println((int)od_state);
  Serial.print(F("spinSpeed="));      Serial.println(od_spinSpeed);
  Serial.print(F("msPerStep="));      Serial.println(od_msPerStep);
  Serial.print(F("settleMs="));       Serial.println(od_settleMs);
  Serial.print(F("driveSpeed="));     Serial.println(od_driveSpeed);
  Serial.print(F("driveMs="));        Serial.println(od_driveMs);
  Serial.print(F("objectMaxCm="));    Serial.println(od_objectMaxCm, 1);  Serial.print(F("touchCm=="));         Serial.println(od_touchCm, 1);
  Serial.print(F("driveMinProb=="));     Serial.println(od_driveMinProb, 2);  Serial.print(F("numSteps="));       Serial.println(OD_NUM_STEPS);
  Serial.print(F("stepDeg="));        Serial.println(OD_STEP_DEG, 1);
}
