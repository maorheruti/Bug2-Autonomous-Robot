// movement_core.cpp — “pure” Nov24 behaviour, with servo-range safety
#include "movement_core.h"
#include "MotorControl.h"
#include "ServoControl.h"
#include <math.h>
#include "DbgLog.h"
#include <string.h>

static uint16_t lastReasonHash = 0;
static void dbg_reason_if_changed(const char* reason, int servoDeg){
  uint16_t h = dbg_hash16(reason);
  if (h != lastReasonHash){
    lastReasonHash = h;
    dbg_push(EV_REASON, servoDeg, h, 0);
  }
}

// ===== Internal state =====
enum Mode { MODE_TRACK = 0, MODE_ALIGN = 1 };

// ===== Movement globals =====
static int         gAngle      = SERVO_CENTER;
static int         gReqAngle   = SERVO_CENTER;
static const char* gReason     = "stopped_by_user";
static Mode        mode        = MODE_TRACK;
static uint32_t    lastTick    = 0;
static bool        gPPControlEnabled = true;  // Flag to disable PP fallback when Bug2 owns robot
static char        gMoveDbg[192] = "";

// ALIGN state
static int      alignDir     = 0;   // +1=left, -1=right
static uint32_t alignStartMs = 0;

// Geometry & tuning (Nov24 values)
static const float WHEELBASE_M     = 0.14f;
static const float MAX_STEER_DEG   = 30.0f;
static const float MIN_LOOKAHEAD_M = 0.15f;
static const float MAX_LOOKAHEAD_M = 0.60f;

// Stop radius
static const float STOP_DIST_M     = 0.45f;

// ALIGN hysteresis
static const float ALIGN_ENTER_DEG = 120.0f;
static const float ALIGN_EXIT_DEG  = 45.0f;
static const float ALIGN_ENTER     = ALIGN_ENTER_DEG * (M_PI / 180.0f);
static const float ALIGN_EXIT      = ALIGN_EXIT_DEG  * (M_PI / 180.0f);
static const uint32_t ALIGN_MIN_MS = 600;

// Simple PWM shaping (Nov24 values)
static const int PWM_BASE     = 200;
static const int PWM_MAX      = 255;
static const int PWM_DIFF_MAX = 80;

// Nov24 baseline: keep the controller crawl slow; motor threshold is hardware-dependent.
static const int      PWM_EFFECTIVE_MIN_BASE = 90;
static const int      PWM_START_KICK_PWM     = 125;
static const uint32_t PWM_START_KICK_MS      = 120;

static char     gLastDriveDir   = 'F';
static uint32_t gDriveKickUntil = 0;
static bool     gDriveWasStopped = true;

// Helper: wrap angle into [-pi, pi]
static float wrap_pi(float a){
  while (a >  M_PI) a -= 2.0f * M_PI;
  while (a < -M_PI) a += 2.0f * M_PI;
  return a;
}

// Clamp helper
static float clampf(float v, float lo, float hi){
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// NEW: shared motor mapping with direction
static inline uint8_t clamp255(int v){ return (uint8_t)constrain(v, 0, 255); }

static void apply_motors_from_angle_scaled_dir(int angleDeg, float speedScale, char dir){
  // keep old behavior shape, but allow slower crawl commands to be distinct.
  speedScale = clampf(speedScale, 0.06f, 1.00f);

  // Normalized steering in [-1, +1] using the current direct servo range.
  const float leftSpan  = (float)max(1, SERVO_CENTER - SERVO_MIN);
  const float rightSpan = (float)max(1, SERVO_MAX - SERVO_CENTER);
  float steerNorm = 0.0f;
  if (angleDeg < SERVO_CENTER) {
    steerNorm = ((float)angleDeg - (float)SERVO_CENTER) / leftSpan;
  } else if (angleDeg > SERVO_CENTER) {
    steerNorm = ((float)angleDeg - (float)SERVO_CENTER) / rightSpan;
  }
  steerNorm = clampf(steerNorm, -1.0f, 1.0f);

  float mag = fabsf(steerNorm);
  int diff  = (int)(PWM_DIFF_MAX * mag);

  int minP = PWM_EFFECTIVE_MIN_BASE;
  if (minP < 80) minP = 80;

  // Interpolate around effective minimum so low requested speeds remain distinct.
  int base = minP + (int)((PWM_BASE - minP) * speedScale);
  if (base < minP) base = minP;

  int pwmLeft, pwmRight;
  if (steerNorm > 0){
    pwmLeft  = base + diff/2;
    pwmRight = base - diff/2;
  } else {
    pwmLeft  = base - diff/2;
    pwmRight = base + diff/2;
  }

  if (pwmLeft  < minP) pwmLeft  = minP;
  if (pwmRight < minP) pwmRight = minP;
  if (pwmLeft  > PWM_MAX) pwmLeft  = PWM_MAX;
  if (pwmRight > PWM_MAX) pwmRight = PWM_MAX;

  const uint32_t now = millis();
  if (gDriveWasStopped || (dir != gLastDriveDir)){
    gDriveKickUntil = now + PWM_START_KICK_MS;
  }
  gDriveWasStopped = false;
  gLastDriveDir = dir;

  if (now < gDriveKickUntil){
    int kickP = PWM_START_KICK_PWM;
    if (kickP < minP) kickP = minP;
    if (pwmLeft  < kickP) pwmLeft  = kickP;
    if (pwmRight < kickP) pwmRight = kickP;
  }

  motor_apply(dir, pwmLeft, dir, pwmRight);
}

// Map steering angle (servo degrees) → motor PWMs and apply (forward)
static void apply_motors_from_angle_scaled(int angleDeg, float speedScale){
  apply_motors_from_angle_scaled_dir(angleDeg, speedScale, 'F');
}

// ===== Public helpers =====
const char* movement_reason(){ return gReason; }
int current_angle(){ return gAngle; }
int movement_last_requested_angle(){ return gReqAngle; }
int movement_last_physical_angle(){ return gAngle; }
const char* movement_dbg_line(){ return gMoveDbg; }

void movement_set_pp_debug(const char* label,
                           const Pose& r,
                           const Pose& t,
                           float distM,
                           int requestedAngleDeg,
                           int physicalAngleDeg,
                           float speedScale)
{
  const float bearing = atan2f(t.y - r.y, t.x - r.x);
  const float alphaDeg = wrap_pi(bearing - r.yaw) * 180.0f / (float)M_PI;
  snprintf(gMoveDbg, sizeof(gMoveDbg),
           "%s dist=%.2f alpha=%.0f req=%d phys=%d sp=%.2f r(%.2f,%.2f,%.2f) tgt(%.2f,%.2f)",
           label ? label : "pp",
           distM, alphaDeg, requestedAngleDeg, physicalAngleDeg, speedScale,
           r.x, r.y, r.yaw, t.x, t.y);
}

void movement_stop_reason(const char* reason, bool centerServo){
  gReqAngle = SERVO_CENTER;
  gAngle  = servo_shape_autonomous_angle(SERVO_CENTER);
  gReason = reason ? reason : "";
  gMoveDbg[0] = '\0';
  dbg_reason_if_changed(gReason, gAngle);
  if (centerServo) servo_set_angle(gAngle);
  motor_stop();
  gDriveWasStopped = true;
}

void movement_servo_only(int angleDeg, const char* reason){
  angleDeg = constrain(angleDeg, SERVO_MIN, SERVO_MAX);
  gReqAngle = angleDeg;
  gAngle   = angleDeg;
  gReason  = reason ? reason : "";
  gMoveDbg[0] = '\0';
  dbg_reason_if_changed(gReason, gAngle);

  servo_set_angle(gAngle);
  motor_stop();
  gDriveWasStopped = true;
}

void movement_servo_calibration(int angleDeg, const char* reason){
  angleDeg = constrain(angleDeg, SERVO_CAL_MIN, SERVO_CAL_MAX);
  gReqAngle = angleDeg;
  gAngle   = angleDeg;
  gReason  = reason ? reason : "";
  gMoveDbg[0] = '\0';
  dbg_reason_if_changed(gReason, gAngle);

  servo_set_angle_calibration(gAngle);
  motor_stop();
  gDriveWasStopped = true;
}

void movement_apply_angle(int angleDeg, const char* reason, float speedScale){
  angleDeg = constrain(angleDeg, SERVO_MIN, SERVO_MAX);
  gReqAngle = angleDeg;
  gAngle   = servo_shape_autonomous_angle(angleDeg);
  gReason  = reason ? reason : "";
  dbg_reason_if_changed(gReason, gAngle);

  servo_set_angle(gAngle);
  apply_motors_from_angle_scaled(gAngle, speedScale);
}

// Reverse actuation helper (servo + reverse motors)
void movement_apply_reverse(int angleDeg, const char* reason, float speedScale){
  angleDeg = constrain(angleDeg, SERVO_MIN, SERVO_MAX);
  gReqAngle = angleDeg;
  gAngle   = servo_shape_autonomous_angle(angleDeg);
  gReason  = reason ? reason : "";
  dbg_reason_if_changed(gReason, gAngle);

  servo_set_angle(gAngle);
  apply_motors_from_angle_scaled_dir(gAngle, speedScale, 'B');
}

// === compute pure pursuit angle (no actuation) ===
bool movement_pp_compute_angle(const Pose& r, const Pose& t,
                               int& outAngleDeg, float& outDistM)
{
  if (t.stamp_ms == 0) return false;

  float dx   = t.x - r.x;
  float dy   = t.y - r.y;
  float dist = sqrtf(dx*dx + dy*dy);
  outDistM = dist;

  float bearing = atan2f(dy, dx);
  float alpha   = wrap_pi(bearing - r.yaw);

  float Ld = dist;
  if (Ld < MIN_LOOKAHEAD_M) Ld = MIN_LOOKAHEAD_M;
  if (Ld > MAX_LOOKAHEAD_M) Ld = MAX_LOOKAHEAD_M;

  float num   = 2.0f * WHEELBASE_M * sinf(alpha);
  float delta = atanf(num / Ld);

  const float maxSteerRad = MAX_STEER_DEG * (M_PI / 180.0f);
  if (delta >  maxSteerRad) delta =  maxSteerRad;
  if (delta < -maxSteerRad) delta = -maxSteerRad;

  float norm = delta / maxSteerRad;         // [-1,1]
  const float leftSpan  = (float)max(1, SERVO_CENTER - SERVO_MIN);
  const float rightSpan = (float)max(1, SERVO_MAX - SERVO_CENTER);
  const float span = (norm < 0.0f) ? leftSpan : rightSpan;
  int angle = (int)roundf((float)SERVO_CENTER + norm * span);

  if (angle < SERVO_MIN) angle = SERVO_MIN;
  if (angle > SERVO_MAX) angle = SERVO_MAX;

  outAngleDeg = angle;
  return true;
}

void movement_setup(){
  gReqAngle    = SERVO_CENTER;
  gAngle       = servo_shape_autonomous_angle(SERVO_CENTER);
  gReason      = "boot_idle";
  gMoveDbg[0]  = '\0';
  dbg_reason_if_changed(gReason, gAngle);
  mode         = MODE_TRACK;
  alignDir     = 0;
  alignStartMs = 0;
  lastTick     = millis();
  servo_set_angle(SERVO_CENTER);
  motor_stop();
  gDriveWasStopped = true;
}

// ===== Main control loop (UNCHANGED BEHAVIOR) =====
void movement_loop_tick(){
  uint32_t now = millis();
  if (now - lastTick < 50) return;
  lastTick = now;

  // Check if PP control is enabled - skip if Bug2 is using robot
  if (!gPPControlEnabled) {
    movement_stop_reason("pp_control_disabled");
    return;
  }

  bool started = snapshotBool(gStarted);

  static bool wasStopped = false;

  if (!started){
    if (!wasStopped){
      gReqAngle = SERVO_CENTER;
      gAngle = servo_shape_autonomous_angle(SERVO_CENTER);

      if (strcmp(gReason, "reached_goal") != 0 &&
          strcmp(gReason, "no_pose") != 0)
      {
        gReason = "stopped_by_user";
      }

      mode = MODE_TRACK;
      servo_set_angle(gAngle);
      motor_stop();
      wasStopped = true;   // remember we already applied stop once
    }
    return;
  }

  // we are started now
  wasStopped = false;

  Pose r = snapshotPose(gRobotLatest);
  Pose t = snapshotPose(gTargetLatest);
  if (t.stamp_ms == 0){
    movement_stop_reason("no_pose");
    return;
  }

  float dx   = t.x - r.x;
  float dy   = t.y - r.y;
  float dist = sqrtf(dx*dx + dy*dy);

  if (dist < STOP_DIST_M){
    movement_stop_reason("reached_goal");

    dbg_push(EV_MARK, 507, 0, 0);
    record_stop_cause(507, 0, dbg_hash16("reached_goal"), "reached_goal");

    dbg_push(EV_MARK, 520, 9, 0);
    noInterrupts();
    gStarted = false;
    interrupts();
    return;
  }

  float bearing = atan2f(dy, dx);
  float alpha   = wrap_pi(bearing - r.yaw);

  if (mode == MODE_ALIGN) {
    if (fabsf(alpha) < ALIGN_EXIT && (now - alignStartMs) > ALIGN_MIN_MS) {
      mode     = MODE_TRACK;
      alignDir = 0;
    }
  } else {
    if (fabsf(alpha) > ALIGN_ENTER) {
      mode         = MODE_ALIGN;
      alignDir     = (alpha > 0.0f) ? +1 : -1;
      alignStartMs = now;
    }
  }

  if (mode == MODE_ALIGN){
    int angle = (alignDir > 0) ? SERVO_MIN : SERVO_MAX;
    movement_set_pp_debug("start_align", r, t, dist, angle,
                          servo_shape_autonomous_angle(angle), 1.0f);
    movement_apply_angle(angle, "align_turn", 1.0f);
    return;
  }

  int angle;
  float dist2;
  if (!movement_pp_compute_angle(r, t, angle, dist2)){
    movement_stop_reason("no_pose");
    return;
  }
  movement_set_pp_debug("start_pp", r, t, dist2, angle,
                        servo_shape_autonomous_angle(angle), 1.0f);
  movement_apply_angle(angle, "running_pure_pursuit", 1.0f);
}

// PP Control enable/disable functions
void movement_set_pp_control_enabled(bool enabled) {
  gPPControlEnabled = enabled;
}

bool movement_is_pp_control_enabled() {
  return gPPControlEnabled;
}
