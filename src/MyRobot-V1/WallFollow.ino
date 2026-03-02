/**
 * @file WallFollow.ino
 * @brief P-controller wall-following and follow-me behaviours.
 *
 * Two modes (mutually exclusive):
 *
 * FOLLOW-ME  — ultrasonic faces forward.  Robot maintains a set distance
 *              from an object in front by driving forward / backward.
 *
 * WALL-FOLLOW — ultrasonic faces sideways (left or right).  Robot drives
 *               forward at a constant base speed while a P-controller
 *               adjusts differential steering to hold a set distance from
 *               the wall.
 *
 * P-controller equation:
 *   error  = measurement - set_point
 *   output = Kp * error
 *
 * Corner handling (wall-follow):
 *   • Inside corner  — wall suddenly gets closer (error goes negative).
 *     The controller steers away automatically.
 *   • Outside corner — wall disappears (getDistanceCm() returns -1).
 *     Robot drives straight briefly, then steers toward the wall side.
 *   • Gradual curves  — handled continuously by the P-controller.
 */

// -------- forward declarations (other .ino files) --------
float getFrontDistanceCm();  // front sensor (follow-me)
float getSideDistanceCm();   // side sensor  (wall-follow)
void  setMotorSpeeds(int leftSpeed, int rightSpeed);
void  turnInPlace(int speed, bool clockwise);
void  forward(int speed);
void  backward(int speed);
void  stop();

// -------- tunable parameters --------
static float wf_setPointCm  = 25.0;   // target distance in cm
static float wf_kp          = 6.0;    // proportional gain
static int   wf_baseSpeed   = 160;    // forward speed used in wall-follow mode
static int   wf_maxSpeed    = 220;    // absolute speed ceiling
static float wf_deadBandCm     = 2.0;    // follow-me dead-band (±cm around set point)
static float wf_obstacleCm         = 10.0;   // front obstacle threshold (cm)
static int   wf_obstacleTurnMs     = 300;    // how long to spin right per obstacle tick (ms)

// -------- mode flags --------
static bool wf_followMeEnabled   = false;
static bool wf_wallFollowEnabled = false;
static bool wf_driveAndAvoid     = false;  // drive forward, turn right on front obstacle
static bool wf_wallOnLeft        = true;   // true = sensor/wall on left side

// Tracks how long we've lost the wall (ms) for outside-corner recovery
static unsigned long wf_lostWallMs = 0;
static const unsigned long WF_LOST_WALL_COAST_MS = 400;  // drive straight this long before turning in

// -------- setters / getters --------
void setFollowMeEnabled(bool en) {
  wf_followMeEnabled   = en;
  wf_wallFollowEnabled = false;
  wf_driveAndAvoid     = false;
  if (!en) stop();
}

void setWallFollowEnabled(bool en) {
  wf_wallFollowEnabled = en;
  wf_followMeEnabled   = false;
  wf_driveAndAvoid     = false;
  wf_lostWallMs        = 0;
  if (!en) stop();
}

void setDriveAndAvoidEnabled(bool en) {
  wf_driveAndAvoid     = en;
  wf_followMeEnabled   = false;
  wf_wallFollowEnabled = false;
  if (!en) stop();
}

bool isDriveAndAvoidEnabled() { return wf_driveAndAvoid; }

bool isFollowMeEnabled()   { return wf_followMeEnabled; }
bool isWallFollowEnabled() { return wf_wallFollowEnabled; }

void  setWallFollowSetPoint(float cm) { wf_setPointCm = cm; }
float getWallFollowSetPoint()          { return wf_setPointCm; }

void  setWallFollowKp(float kp) { wf_kp = kp; }
float getWallFollowKp()          { return wf_kp; }

void setWallFollowBaseSpeed(int sp) { wf_baseSpeed = sp; }
int  getWallFollowBaseSpeed()        { return wf_baseSpeed; }

void setWallOnLeft(bool left) { wf_wallOnLeft = left; }
bool isWallOnLeft()            { return wf_wallOnLeft; }

void  setObstacleThreshold(float cm) { wf_obstacleCm = cm; }
float getObstacleThreshold()          { return wf_obstacleCm; }

void setObstacleTurnMs(int ms) { wf_obstacleTurnMs = ms; }
int  getObstacleTurnMs()        { return wf_obstacleTurnMs; }

// -------- internal helpers --------
static int wfClamp(int v, int maxVal) {
  if (v >  maxVal) return  maxVal;
  if (v < -maxVal) return -maxVal;
  return v;
}

// -------- follow-me tick --------
static void tickFollowMe() {
  float dist = getFrontDistanceCm();

  if (dist < 0 || dist < 5.0) {
    // Too close (or sensor can't read) — reverse
    backward(wf_baseSpeed);
  } else if (dist > 5.0) {
    // Too far — drive forward
    forward(wf_baseSpeed);
  } else {
    stop();
  }
}

// -------- wall-follow tick --------
/**
 * Side-facing sensor.  Drives forward at wf_baseSpeed and applies a
 * P-correction to the differential motor speeds to maintain wf_setPointCm
 * from the side wall.
 *
 * Corner cases:
 *   Inside corner  — dist < setPoint → error negative → steers away from wall.
 *   Outside corner — getSideDistanceCm() returns -1  → robot drives straight
 *                    for WF_LOST_WALL_COAST_MS ms, then begins searching
 *                    (turns toward the wall side) until the wall is found again.
 */
static void tickWallFollow() {
  float dist = getSideDistanceCm();

  if (dist < 0) {
    // --- outside corner / lost wall ---
    if (wf_lostWallMs == 0) {
      // First tick without wall — record timestamp
      wf_lostWallMs = millis();
    }

    unsigned long elapsed = millis() - wf_lostWallMs;

    if (elapsed < WF_LOST_WALL_COAST_MS) {
      // Phase 1: drive straight briefly (robot rounds the outside corner)
      setMotorSpeeds(wf_baseSpeed, wf_baseSpeed);
    } else {
      // Phase 2: turn toward the wall side to find it again
      if (wf_wallOnLeft) {
        setMotorSpeeds(wf_baseSpeed / 2, wf_baseSpeed);   // steer left
      } else {
        setMotorSpeeds(wf_baseSpeed, wf_baseSpeed / 2);   // steer right
      }
    }
    return;
  }

  // Wall is visible — reset lost-wall timer
  wf_lostWallMs = 0;

  float error      = dist - wf_setPointCm;  // + = too far from wall, - = too close
  int   correction = wfClamp((int)(wf_kp * error), wf_baseSpeed);

  int leftSpeed, rightSpeed;

  if (wf_wallOnLeft) {
    // error > 0 (drifted right, too far from left wall) → steer left
    //   slow down right wheel more than left, so robot curves left
    leftSpeed  = wf_baseSpeed + correction;
    rightSpeed = wf_baseSpeed - correction;
  } else {
    // Wall on right: mirror
    leftSpeed  = wf_baseSpeed - correction;
    rightSpeed = wf_baseSpeed + correction;
  }

  leftSpeed  = wfClamp(leftSpeed,  wf_maxSpeed);
  rightSpeed = wfClamp(rightSpeed, wf_maxSpeed);

  setMotorSpeeds(leftSpeed, rightSpeed);
}

// -------- obstacle avoidance --------
/**
 * @brief If the front sensor sees an obstacle, spins right for
 *        wf_obstacleTurnMs ms then stops (returns true).
 *        Uses a blocking delay so the turn is a fixed angle, not
 *        continuous spinning.
 */
bool obstacleAvoidTick() {
  float frontDist = getFrontDistanceCm();
  if (frontDist > 0 && frontDist < wf_obstacleCm) {
    turnInPlace(wf_baseSpeed, true);       // spin right
    delay(wf_obstacleTurnMs);              // turn for fixed duration
    stop();
    return true;
  }
  return false;
}

// -------- drive-and-avoid tick --------
/**
 * @brief Drives straight forward.  When the front sensor sees an obstacle
 *        closer than wf_obstacleCm, spins right in place until clear.
 */
static void tickDriveAndAvoid() {
  if (obstacleAvoidTick()) return;   // obstacle — spin right
  setMotorSpeeds(wf_baseSpeed, wf_baseSpeed);  // clear — drive forward
}

// -------- public tick (called from main loop) --------
/**
 * @brief Call every loop iteration.  Obstacle avoidance takes priority,
 *        then dispatches to follow-me or wall-follow.
 */
void wallFollowTick() {
  if (wf_driveAndAvoid)    { tickDriveAndAvoid(); return; }
  if (wf_followMeEnabled)  { tickFollowMe();  return; }
  if (wf_wallFollowEnabled){ tickWallFollow(); return; }
}
