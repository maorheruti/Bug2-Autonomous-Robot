// bug2_core.cpp  Zone-Combo Bug2 State Machine (v3)
//
// Minimal implementation: WF decisions are a priority-ordered list of
// zone-combo checks.  Each combo directly sets servo angle + speed.
// No waypoint generation, no VFH scoring, no step executor.
// PP used only in GTG for goal approach.
//
// TEST workflow:
//   Each combo has a bug2_stop("TEST:xxx") line.  When hit the robot
//   stops so you can observe orientation vs obstacle.  Once verified,
//   comment out / move the stop line  re-upload  test next combo.
//
// Zone map (robot-centric, sensor FOV center angles):
//
//   L3: +87  (left rear)           R0: -87  (right rear)
//   L2: +76  (left side)           R1: -76  (right side)
//   L1: +64  (left fwd-oblique)    R2: -64  (right fwd-oblique)
//   L0: +53  (left front-oblique)  R3: -53  (right front-oblique)
//   M3: +23  (front-left)          M0: -23  (front-right)
//   M2: +8   (front center-left)   M1: -8   (front center-right)
//
// Zone naming in code:  zL[0..3] = L0..L3,  zM[0..3] = M0..M3,
//                       zR[0..3] = R0..R3
//
// Backup: v2 VFH code  bug2_core_v2_vfh_01mar2026.cpp/h  (01 Mar 2026)
// ============================================================================

// ---------------------------------------------------------------------------
// BUG2_NO_POSE  sensor-only testing mode (no OptiTrack / no pose data)
//
//   1 = sensor-only: GTG drives straight, no PP, no goal-reached,
//       no stall detection, no M-line exit.  Stop manually via /bug2stop.
//   0 = full Bug2 with pose (OptiTrack / M-line enabled)
// ---------------------------------------------------------------------------
#define BUG2_NO_POSE  0

#include "bug2_core.h"
#include "movement_core.h"
#if !BUG2_NO_POSE
#include "pure_pursuit.h"
#endif
#include "ServoControl.h"
#include "globals.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// ---------------------------------------------------------------------------
// Debug / decision log  (same interface as v1/v2)
// ---------------------------------------------------------------------------
static char gBug2Dbg[768] = {0};
const char* bug2_dbg_line() { return gBug2Dbg; }

static char gBug2DecisionLog[512] = {0};
static uint16_t gLogIdx = 0;
const char* bug2_decision_log() { return gBug2DecisionLog; }

static void log_decision(const char* event) {
  if (!event) return;
  const uint32_t ms = millis();
  const size_t cap = sizeof(gBug2DecisionLog);
  size_t rem = cap - gLogIdx;
  int n = snprintf(gBug2DecisionLog + gLogIdx, rem, "[%lu]%s|",
                   (unsigned long)ms, event);
  if (n < 0) return;
  if ((size_t)n >= rem) {
    gLogIdx = 0;
    snprintf(gBug2DecisionLog, cap, "[%lu]%s|", (unsigned long)ms, event);
    gLogIdx = (uint16_t)strlen(gBug2DecisionLog);
    return;
  }
  gLogIdx += (uint16_t)n;
}

static void set_wf_recovery_dbg(const Pose& robot, bool isLeft, const char* mode,
                                int servoCmd, uint16_t dFront, uint16_t dSideMin,
                                const uint16_t zL[4], const uint16_t zM[4], const uint16_t zR[4]) {
  snprintf(gBug2Dbg, sizeof(gBug2Dbg),
           "r(%.2f,%.2f) WF side=%c mode=%s servo=%d dFront=%u dSide=%u L=[%u,%u,%u,%u] M=[%u,%u,%u,%u] R=[%u,%u,%u,%u]",
           robot.x, robot.y, isLeft ? 'L' : 'R', mode, servoCmd,
           (unsigned)dFront, (unsigned)dSideMin,
           (unsigned)zL[0], (unsigned)zL[1], (unsigned)zL[2], (unsigned)zL[3],
           (unsigned)zM[0], (unsigned)zM[1], (unsigned)zM[2], (unsigned)zM[3],
           (unsigned)zR[0], (unsigned)zR[1], (unsigned)zR[2], (unsigned)zR[3]);
}

// ---------------------------------------------------------------------------
// Pure pursuit params (used only in GTG when pose is available)
// ---------------------------------------------------------------------------
#if !BUG2_NO_POSE
static const PurePursuitParams BUG2_PP = {
  0.14f,   // wheelbase_m
  30.0f,   // max_steer_deg
  0.15f,   // min_lookahead_m
  0.60f,   // max_lookahead_m
  SERVO_CENTER, // servo_center_deg
  30,           // nominal servo half range
  SERVO_MIN,    // servo_min_deg
  SERVO_MAX     // servo_max_deg
};
#endif

// ---------------------------------------------------------------------------
// Servo limits
// ---------------------------------------------------------------------------
static constexpr int SERVO_LEFT     = SERVO_MIN;     // max left
static constexpr int SERVO_RIGHT    = SERVO_MAX;     // max right
static constexpr int SERVO_STRAIGHT = SERVO_CENTER;  // center

// ---------------------------------------------------------------------------
// Zone thresholds (mm)  adjust from test runs
//
//   ZONE_EMERGENCY   too close, must reverse
//   ZONE_CLOSE       obstacle nearby, steer hard away
//   ZONE_MID         moderate proximity
//   ZONE_FAR         wall is visible but not close
//   ZONE_WALL        anything under this counts as "wall visible"
// ---------------------------------------------------------------------------
static constexpr uint16_t ZONE_EMERGENCY  = 150;
static constexpr uint16_t ZONE_CLOSE      = 300;
static constexpr uint16_t ZONE_MID        = 520;
static constexpr uint16_t ZONE_FAR        = 800;
static constexpr uint16_t ZONE_WALL       = 1200;

// GTG: trigger WF before the front sensor is already in the hard block zone.
// With the drivetrain wiring fixed the robot closes the last ~200mm quickly, so
// the old late handoff entered WF already boxed in.
static constexpr uint16_t GTG_TRIGGER_MM     = 900;
static constexpr uint16_t GTG_TRIGGER_EARLY_MM = 1050;
static constexpr uint16_t GTG_PRE_BIAS_MM    = 900;
static constexpr int      GTG_FRONT_NUDGE_MAX_DEG = 8;
static constexpr uint16_t GTG_SIDE_TRIGGER_MM = 600;
static constexpr uint16_t GTG_SIDE_NEAR_ENTER_MM = 720;
static constexpr uint16_t GTG_SIDE_DANGER_MM = 180;
static constexpr int      GTG_STEER_BAND_DEG = 8;
static constexpr float    GTG_FRONT_FAST_MMPS = 220.0f;
static constexpr int      GTG_BIAS_SLEW_DEG = 6;
static constexpr float    GTG_SIDE_BIAS_K = 3.2f;

// WF target / tuning
static constexpr float WF_TARGET_MM = 260.0f;
static constexpr float WF_DIST_K = 3.2f;
static constexpr float WF_YAW_K  = 2.2f;
static constexpr float WF_BEAR_K = 0.12f;
static constexpr float WF_OPP_K  = 3.5f;
static constexpr float WF_FRONT_K = 4.0f;

// Dead-end recovery pulses
static constexpr uint32_t WF_DEADEND_REV_MS  = 450;
static constexpr uint32_t WF_DEADEND_TURN_MS = 300;
static constexpr uint16_t WF_SIDE_HARD_MM    = 160;
static constexpr uint16_t WF_SIDE_CRIT_MM    = 75;
static constexpr uint8_t  WF_DEND_ENTER_COUNT = 3;
static constexpr uint16_t WF_EMERG_REV_FRONT_MM = 70;
static constexpr uint16_t WF_EMERG_REV_NEAR_MM = 145;
static constexpr uint16_t WF_EMERG_REV_FORCE_MM = 55;
static constexpr uint32_t WF_EMERG_FBLK_GUARD_MS = 350;
static constexpr uint8_t  WF_SIDE_SETTLE_TICKS = 1;
static constexpr uint8_t  WF_SIDE_HARD_COOLDOWN_TICKS = 2;
// Ackermann settle pulses (TRACK/OUTER only):
// issue a short steer pulse, then a brief neutral settle tick, then re-evaluate.
static constexpr uint8_t  WF_ALIGN_HOLD_TICKS_MILD = 1;
static constexpr uint8_t  WF_ALIGN_HOLD_TICKS_STRONG = 2;
static constexpr uint8_t  WF_ALIGN_NEUTRAL_TICKS = 1;
static constexpr int      WF_ALIGN_TRIGGER_DEG = 8;
static constexpr int      WF_ALIGN_STRONG_DEG = 16;
static constexpr int      WF_ALIGN_MIN_DELTA_DEG = 5;
static constexpr uint16_t WF_ALIGN_FRONT_MIN_MM = 320;
static constexpr uint16_t WF_ALIGN_SIDE_NEAR_MM = 220;
static constexpr float    WF_EQ_K = 5.0f;
static constexpr float    WF_EQ_ERR_SPAN_MM = 110.0f;
static constexpr uint16_t WF_EQ_ALIGN_MM = 70;
static constexpr uint16_t WF_EQ_CAPTURE_MM = 90;
// Side-aware align shaping:
// - far from followed side -> stronger pulse toward followed side
// - near target side distance -> softer response + deadband to reduce chatter
static constexpr float    WF_ALIGN_NEAR_DB_MM = 35.0f;
static constexpr float    WF_ALIGN_NEAR_SOFT_SCALE = 0.55f;
static constexpr int      WF_ALIGN_NEAR_NEUTRAL_DEG = 3;
static constexpr uint16_t WF_ALIGN_PAIR_SETTLE_MM = 12;
static constexpr float    WF_ALIGN_TRIM_MM = 14.0f;
static constexpr int      WF_ALIGN_TRIM_MIN_DEG = 2;
static constexpr int      WF_ALIGN_TRIM_MAX_DEG = 4;
static constexpr float    WF_ALIGN_FAR_SPAN_MM = 220.0f;
static constexpr int      WF_ALIGN_FAR_BIAS_MIN_DEG = 3;
static constexpr int      WF_ALIGN_FAR_BIAS_MAX_DEG = 10;
static constexpr float    WF_FBLK_NOSE_TARGET_BASE_MM = 340.0f;
static constexpr float    WF_FBLK_NOSE_TARGET_SIDE_GAIN = 0.30f;
static constexpr float    WF_FBLK_NOSE_TARGET_MIN_MM = 320.0f;
static constexpr float    WF_FBLK_NOSE_TARGET_MAX_MM = 460.0f;
static constexpr float    WF_FBLK_NOSE_NEED_SPAN_MM = 160.0f;
static constexpr float    WF_FBLK_PAIR_NEED_SPAN_MM = 60.0f;
static constexpr float    WF_FBLK_AWAY_MIN_DEG = 4.0f;
static constexpr float    WF_FBLK_AWAY_MAX_DEG = 28.0f;
static constexpr uint16_t WF_FRONT_CONV_START_MM = 650;
static constexpr uint16_t WF_FRONT_CONV_STRONG_MM = 420;
static constexpr float    WF_FRONT_CONV_MAX_DEG = 16.0f;
static constexpr uint16_t WF_FRONT_MID_PROJ_GAP_MM = 65;
static constexpr uint16_t WF_FRONT_MID_PROJ_SOFT_GAP_MM = 80;
static constexpr uint16_t WF_FRONT_MID_PROJ_NEAR_MARGIN_MM = 45;
static constexpr float    WF_TRACK_PROJ_FRONT_CAP_DEG = 3.5f;
static constexpr float    WF_TRACK_BROAD_FRONT_CAP_DEG = 6.0f;
static constexpr float    WF_TRACK_INWARD_FRONT_CAP_DEG = 7.0f;
static constexpr float    WF_FBLK_PROJ_AWAY_CAP_DEG = 6.5f;
static constexpr float    WF_TRACK_SIDE_NEAR_AWAY_DEG = 14.0f;
static constexpr float    WF_TRACK_SIDE_VERY_NEAR_AWAY_DEG = 18.0f;
static constexpr float    WF_TRACK_SIDE_CRIT_AWAY_DEG = 22.0f;
static constexpr float    WF_TRACK_FRONT_NEAR_AWAY_DEG = 11.0f;
static constexpr float    WF_TRACK_FRONT_CLOSE_AWAY_DEG = 15.0f;
static constexpr float    WF_FBLK_SIDE_NEAR_AWAY_DEG = 14.0f;
static constexpr float    WF_FBLK_SIDE_VERY_NEAR_AWAY_DEG = 18.0f;
static constexpr float    WF_FBLK_SIDE_CRIT_AWAY_DEG = 24.0f;
static constexpr float    WF_INNER_PRECAUTION_MAX_DEG = 9.0f;
static constexpr float    WF_INNER_PRECAUTION_MIN_DEG = 3.0f;
static constexpr float    WF_INNER_TRACK_FLOOR_GAIN = 0.90f;
static constexpr float    WF_INNER_FBLK_CAP_PAD_DEG = 2.0f;
static constexpr float    WF_INNER_FBLK_MAX_CAP_DEG = 12.0f;
static constexpr float    WF_INNER_FBLK_CLOSE_CAP_DEG = 15.0f;
static constexpr float    WF_INNER_PRE_CUE_MAX_TOWARD_DEG = 0.5f;
static constexpr float    WF_INNER_PRE_CUE_MIN_AWAY_DEG = 2.0f;
static constexpr float    WF_INNER_PRE_CUE_MAX_AWAY_DEG = 4.5f;
static constexpr uint32_t WF_POST_FBLK_DAMP_MS = 450;
static constexpr uint32_t WF_EMERG_LAST_FWD_MS = 1100;
static constexpr uint32_t WF_EMERG_LAST_REV_MS = 650;
static constexpr uint16_t WF_FRONT_OPEN_SEEK_MARGIN_MM = 40;
static constexpr uint16_t WF_FRONT_OPEN_SEEK_CAPTURE_MARGIN_MM = 80;
static constexpr uint32_t WF_RECOVER_TOWARD_MS = 650;
static constexpr float    WF_GRIP_SIDE_CLOSE_PAD_MM = 70.0f;
static constexpr uint16_t WF_PEELOFF_FRONT_CLEAR_MM = 520;
static constexpr uint16_t WF_PEELOFF_FRONT_ROOM_MM = 620;
static constexpr uint32_t WF_OUTER_ALL_CLEAR_HOLD_MS = 900;
static constexpr float    WF_OUTER_ALL_CLEAR_MIN_DEG = 22.0f;
static constexpr uint16_t WF_FBLK_BROAD_EARLY_MM = 520;
static constexpr uint16_t WF_FBLK_BROAD_STRONG_MM = 420;
static constexpr uint16_t WF_FBLK_BROAD_MAX_MM = 320;
static constexpr float    WF_FBLK_BROAD_EARLY_AWAY_DEG = 18.0f;
static constexpr float    WF_FBLK_BROAD_STRONG_AWAY_DEG = 24.0f;
static constexpr float    WF_FBLK_BROAD_MAX_AWAY_DEG = 28.0f;
// Full-loss reacquire behavior:
// when followed side is fully lost (nF=0, dSide invalid), turn hard toward it
// and prevent opposite drift until side evidence returns.
static constexpr float    WF_REACQ_HARD_TOWARD_DEG = 22.0f;
static constexpr float    WF_REACQ_HARD_TOWARD_NEAR_DEG = 20.0f;
static constexpr uint16_t WF_REACQ_HARD_FRONT_MM = 320;
static constexpr float    WF_REACQ_CAPTURE_BAND_MM = 240.0f;
static constexpr uint16_t WF_RECOVER_TRACK_FRONT_CLEAR_MM = 300;
static constexpr uint16_t WF_RECOVER_TRACK_SIDE_BAND_MM = 120;
static constexpr float    WF_RECOVER_FULLLOSS_TOWARD_DEG = 26.0f;
static constexpr float    WF_RECOVER_FULLLOSS_TOWARD_NEAR_DEG = 22.0f;
static constexpr uint32_t WF_SEEK_TOWARD_LATCH_MS = 900;
static constexpr uint32_t WF_OUTER_EDGE_GRIP_MS = 2400;
static constexpr float    WF_OUTER_EDGE_TRACK_TOWARD_DEG = 12.0f;
static constexpr float    WF_OUTER_EDGE_PARTIAL_TOWARD_DEG = 22.0f;
static constexpr float    WF_OUTER_EDGE_FULL_TOWARD_DEG = 24.0f;
static constexpr uint16_t WF_SIDE_TRIPLET_ALIGN_SPAN_MM = 90;
static constexpr uint16_t WF_SIDE_TRIPLET_MIN_MM = 165;
static constexpr float    WF_FBLK_TRIPLET_AWAY_CAP_DEG = 6.0f;
static constexpr uint32_t WF_ACQ_FRONT_GUARD_MS = 900;
static constexpr uint8_t  WF_ACQ_FRONT_CONFIRM_TICKS = 3;
static constexpr uint16_t WF_ACQ_FRONT_EDGE_MM = 330;
static constexpr uint16_t WF_ACQ_FRONT_CORE_MM = 360;
static constexpr uint16_t WF_ACQ_FRONT_CRIT_MM = 190;
static constexpr float    WF_ACQ_FRONT_BROAD_CONF = 0.52f;
static constexpr int      WF_ACQ_SUPPRESS_AWAY_CAP_DEG = 4;
static constexpr int      WF_ACQ_REAL_FRONT_AWAY_CAP_DEG = 16;
static constexpr float    WF_SEEK_TOWARD_PARTIAL_DEG = 22.0f;
static constexpr float    WF_SEEK_TOWARD_FULL_DEG = 24.0f;
static constexpr uint32_t WF_ANTI_SWING_MS = 950;
static constexpr float    WF_ANTI_SWING_TRIGGER_DEG = 12.0f;
static constexpr float    WF_ANTI_SWING_SEEK_CAP_DEG = 12.0f;
static constexpr float    WF_SEEK_OVERSHOOT_YAW_DEG = 95.0f;
static constexpr float    WF_SEEK_OVERSHOOT_DIST_M = 0.75f;
// When controller requests near-straight after a hard steer, unwind quickly so
// the robot does not keep carving into a wall due to slew-limited lag.
static constexpr int      WF_UNWIND_NEAR_CENTER_DEG = 4;
static constexpr int      WF_UNWIND_FROM_DEG = 14;
static constexpr int      WF_UNWIND_STEP_DEG = 18;
static constexpr int      WF_UNWIND_STEP_FRONT_DEG = 22;
static constexpr int      WF_UNWIND_STEP_ACTIVE_DEG = 16;
static constexpr int      WF_UNWIND_STEP_ACTIVE_FRONT_DEG = 24;
static constexpr int      WF_UNWIND_STEP_CROSS_DEG = 20;

// Mode stability defaults: hysteresis + minimum dwell
static constexpr uint16_t DEF_WF_MODE_DWELL_MS = 320;

// Front-block hysteresis defaults
static constexpr float    DEF_WF_FBLK_ENTER_CORE_CONF = 0.65f;
static constexpr float    DEF_WF_FBLK_EXIT_CORE_CONF  = 0.45f;
static constexpr uint16_t DEF_WF_FBLK_ENTER_FRONT_MM  = 420;
static constexpr uint16_t DEF_WF_FBLK_EXIT_FRONT_MM   = 540;

// Dead-end hysteresis defaults
static constexpr float DEF_WF_DEND_ENTER_FRONT_CONF  = 0.62f;
static constexpr float DEF_WF_DEND_ENTER_OPP_CONF    = 0.48f;
static constexpr float DEF_WF_DEND_ENTER_FOLLOW_CONF = 0.25f;
static constexpr float DEF_WF_DEND_EXIT_FRONT_CONF   = 0.46f;
static constexpr float DEF_WF_DEND_EXIT_OPP_CONF     = 0.34f;

// Runtime-tunable values (defaults initialized from constants above).
static uint16_t gZoneEmergency = ZONE_EMERGENCY;
static uint16_t gZoneClose = ZONE_CLOSE;
static uint16_t gZoneMid = ZONE_MID;
static float gWfTargetMm = WF_TARGET_MM;
static float gWfDistK = WF_DIST_K;
static float gWfYawK = WF_YAW_K;
static float gWfBearK = WF_BEAR_K;
static float gWfOppK = WF_OPP_K;
static float gWfFrontK = WF_FRONT_K;
static uint16_t gWfDeadendRevMs = (uint16_t)WF_DEADEND_REV_MS;
static uint16_t gWfDeadendTurnMs = (uint16_t)WF_DEADEND_TURN_MS;
static uint16_t gWfModeDwellMs = DEF_WF_MODE_DWELL_MS;
static float gWfFblkEnterCoreConf = DEF_WF_FBLK_ENTER_CORE_CONF;
static float gWfFblkExitCoreConf = DEF_WF_FBLK_EXIT_CORE_CONF;
static uint16_t gWfFblkEnterFrontMm = DEF_WF_FBLK_ENTER_FRONT_MM;
static uint16_t gWfFblkExitFrontMm = DEF_WF_FBLK_EXIT_FRONT_MM;
static float gWfDendEnterFrontConf = DEF_WF_DEND_ENTER_FRONT_CONF;
static float gWfDendEnterOppConf = DEF_WF_DEND_ENTER_OPP_CONF;
static float gWfDendEnterFollowConf = DEF_WF_DEND_ENTER_FOLLOW_CONF;
static float gWfDendExitFrontConf = DEF_WF_DEND_EXIT_FRONT_CONF;
static float gWfDendExitOppConf = DEF_WF_DEND_EXIT_OPP_CONF;

// Speed presets
static constexpr float SPD_CRAWL  = 0.12f;
static constexpr float SPD_ESCAPE = 0.16f;
static constexpr float SPD_SLOW   = 0.18f;
static constexpr float SPD_MED    = 0.25f;

// Goal reached radius
static constexpr float GOAL_REACHED_M = 0.30f;

// M-line exit
static constexpr float MLINE_TOL_M       = 0.20f;

// Stall detection
static constexpr float    STALL_DIST_M  = 0.05f;
static constexpr uint32_t STALL_TIME_MS = 4000;

// Stuck recovery
static constexpr uint32_t STUCK_REV_MS     = 1500;
static constexpr uint32_t STUCK_TIMEOUT_MS = 8000;

// Wall-lost limit before STUCK
static constexpr uint8_t WF_LOST_LIMIT = 25;

// Side choose thresholds
static constexpr uint16_t SIDE_CHOOSE_MM      = 1500;
static constexpr uint16_t SIDE_FORCE_NEAR_MM  = 900;
static constexpr uint16_t SIDE_FRONT_MARGIN_MM = 150;
static constexpr uint16_t SIDE_SWITCH_MARGIN_MM = 180;
static constexpr uint32_t SIDE_STICKY_MS = 5000;

// ---------------------------------------------------------------------------
// Internal state
// ---------------------------------------------------------------------------
enum FollowSide : uint8_t { FOLLOW_LEFT = 0, FOLLOW_RIGHT = 1 };

enum FreshState : uint8_t {
  FS_IDLE = 0,
  FS_GTG,
  FS_WF,
  FS_REACHED,
  FS_STUCK
};

static FreshState fs = FS_IDLE;
static Bug2State  gS = BUG2_IDLE;
static Bug2RunMode gBug2Mode = BUG2_MODE_FULL;

static bool  gHasGoal = false;
static float gGoalX = 0.0f, gGoalY = 0.0f;
static float gStartX = 0.0f, gStartY = 0.0f;
static float gHitGoalDistM = 1e9f;
static float gHitX = 0.0f, gHitY = 0.0f;
static bool  gHitValid = false;
static bool  gLeaveCandValid = false;
static float gLeaveCandX = 0.0f, gLeaveCandY = 0.0f;
static uint32_t gLeaveCandMs = 0;

static FollowSide gSide = FOLLOW_RIGHT;
static uint32_t   gSideLastSetMs = 0;
static uint32_t   stEnterMs   = 0;
static uint8_t    wallLostCnt = 0;
static int        gGtgServoCmd = SERVO_STRAIGHT;

static uint32_t stuckPhaseMs   = 0;
static bool     stuckReversing = false;

// Stall detection
static float    stallLastX = 0.0f;
static float    stallLastY = 0.0f;
static uint32_t stallMs    = 0;

// Front trend (mm/s, positive = closing in)
static uint16_t gPrevFrontMm = 0xFFFF;
static uint32_t gPrevFrontMs = 0;
static float    gFrontClosingMms = 0.0f;
static uint16_t gPrevM0Mm = 0xFFFF;
static uint16_t gPrevM3Mm = 0xFFFF;
static uint32_t gPrevM0Ms = 0;
static uint32_t gPrevM3Ms = 0;
static float    gM0ClosingMms = 0.0f;
static float    gM3ClosingMms = 0.0f;

enum WfMode : uint8_t {
  WF_TRACK = 0,
  WF_INNER_CORNER,
  WF_OUTER_SEEK,
  WF_FRONT_BLOCK,
  WF_DEADEND
};

static WfMode gWfMode = WF_TRACK;
static bool gWfDeadendActive = false;
static uint32_t gWfDeadendMs = 0;
static uint32_t gWfModeSinceMs = 0;
static uint8_t gWfDendCandCnt = 0;
static int gWfServoCmd = SERVO_STRAIGHT;
static int8_t gWfFblkSign = 0;
static bool gWfCapturedWall = false;
static uint8_t gWfAlignHoldTicks = 0;
static uint8_t gWfAlignNeutralTicks = 0;
static int gWfAlignServo = SERVO_STRAIGHT;
static uint8_t gWfSideSettleTicks = 0;
static uint8_t gWfSideHardCooldownTicks = 0;
static int8_t gWfEmergCarrySign = 0;
static uint32_t gWfEmergCarryUntilMs = 0;
static int8_t gWfEmergLastRevServoSign = 0;
static uint32_t gWfEmergLastRevMs = 0;
static uint32_t gWfRecoverTowardUntilMs = 0;
static uint32_t gWfSeekTowardUntilMs = 0;
static uint32_t gWfOuterEdgeGripUntilMs = 0;
static uint32_t gWfAcquireGuardUntilMs = 0;
static uint8_t gWfAcqFrontConfirmTicks = 0;
static uint32_t gWfPostFblkDampUntilMs = 0;
static uint32_t gWfAntiSwingUntilMs = 0;
static bool gWfLossAnchorValid = false;
static float gWfLossX = 0.0f;
static float gWfLossY = 0.0f;
static float gWfLossYaw = 0.0f;
static uint32_t gWfLossMs = 0;
static bool gWfSeekYawTrackValid = false;
static float gWfSeekLastYaw = 0.0f;
static float gWfSeekYawAccumRad = 0.0f;
static uint32_t gWfLastSensorFrame = 0;

static const char* wf_mode_name(WfMode m) {
  switch (m) {
    case WF_TRACK:        return "TRACK";
    case WF_INNER_CORNER: return "INNER_CORNER";
    case WF_OUTER_SEEK:   return "OUTER_SEEK";
    case WF_FRONT_BLOCK:  return "FRONT_BLOCK";
    case WF_DEADEND:      return "DEAD_END";
    default:              return "?";
  }
}

static bool wf_front_block_enter(float frontCoreConf, uint16_t dFrontCoreMm) {
  const bool dFrontValid = (dFrontCoreMm != 0xFFFF) && (dFrontCoreMm != 0) && (dFrontCoreMm < 8190);
  return (frontCoreConf >= gWfFblkEnterCoreConf) ||
         (dFrontValid && dFrontCoreMm <= gWfFblkEnterFrontMm);
}

static bool wf_front_block_keep(float frontCoreConf, uint16_t dFrontCoreMm) {
  const bool dFrontValid = (dFrontCoreMm != 0xFFFF) && (dFrontCoreMm != 0) && (dFrontCoreMm < 8190);
  return (frontCoreConf >= gWfFblkExitCoreConf) ||
         (dFrontValid && dFrontCoreMm <= gWfFblkExitFrontMm);
}

static bool wf_deadend_enter(float frontCoreConf, float oppConf, float followConf) {
  return (frontCoreConf >= gWfDendEnterFrontConf) &&
         (oppConf >= gWfDendEnterOppConf) &&
         (followConf >= gWfDendEnterFollowConf);
}

static bool wf_deadend_keep(float frontCoreConf, float oppConf) {
  return (frontCoreConf >= gWfDendExitFrontConf) &&
         (oppConf >= gWfDendExitOppConf);
}

// ---------------------------------------------------------------------------
// Math helpers
// ---------------------------------------------------------------------------
static inline int clampi(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}
static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}
static inline float wrap_pi(float a) {
  while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
  while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
  return a;
}
static inline float dist2(float x1, float y1, float x2, float y2) {
  float dx = x2 - x1, dy = y2 - y1;
  return sqrtf(dx * dx + dy * dy);
}
static inline bool is_valid_mm(uint16_t d) {
  return (d != 0xFFFF) && (d != 0) && (d < 8190);
}

static void wf_reset_loss_anchor() {
  gWfLossAnchorValid = false;
  gWfSeekYawTrackValid = false;
  gWfSeekYawAccumRad = 0.0f;
}

static char wf_loop_bucket(float absCumYawDeg) {
  if (absCumYawDeg < 90.0f) return '0';
  if (absCumYawDeg < 150.0f) return 'Q';
  if (absCumYawDeg < 240.0f) return 'H';
  if (absCumYawDeg < 330.0f) return '3';
  return 'F';
}

static void wf_note_loss_anchor(const Pose& robot, uint32_t nowMs, bool isLeft,
                                uint16_t dFront, uint16_t dSideMin, uint8_t nFollow) {
  gWfLossAnchorValid = true;
  gWfLossX = robot.x;
  gWfLossY = robot.y;
  gWfLossYaw = robot.yaw;
  gWfLossMs = nowMs;
  gWfSeekYawTrackValid = true;
  gWfSeekLastYaw = robot.yaw;
  gWfSeekYawAccumRad = 0.0f;

  char ev[128];
  snprintf(ev, sizeof(ev),
           "WF_LOSS_AT x=%.2f y=%.2f yaw=%.2f s=%c dF=%u dS=%u nF=%u",
           robot.x, robot.y, robot.yaw, isLeft ? 'L' : 'R',
           (unsigned)dFront, (unsigned)dSideMin, (unsigned)nFollow);
  log_decision(ev);
}

static void wf_update_loss_metrics(const Pose& robot, uint32_t nowMs,
                                   float& lossDistM, float& lossYawDeg,
                                   float& lossCumYawDeg, uint32_t& lossAgeMs,
                                   char& lossBucket) {
  lossDistM = 0.0f;
  lossYawDeg = 0.0f;
  lossCumYawDeg = 0.0f;
  lossAgeMs = 0;
  lossBucket = '-';
  if (!gWfLossAnchorValid) return;

  if (!gWfSeekYawTrackValid) {
    gWfSeekYawTrackValid = true;
    gWfSeekLastYaw = robot.yaw;
  } else {
    gWfSeekYawAccumRad += wrap_pi(robot.yaw - gWfSeekLastYaw);
    gWfSeekLastYaw = robot.yaw;
  }

  lossDistM = dist2(robot.x, robot.y, gWfLossX, gWfLossY);
  lossYawDeg = wrap_pi(robot.yaw - gWfLossYaw) * 180.0f / (float)M_PI;
  lossCumYawDeg = gWfSeekYawAccumRad * 180.0f / (float)M_PI;
  lossAgeMs = nowMs - gWfLossMs;
  lossBucket = wf_loop_bucket(fabsf(lossCumYawDeg));
}

// ---------------------------------------------------------------------------
// Sensor helpers
// ---------------------------------------------------------------------------
static uint16_t min4_valid(const uint16_t z[4]) {
  uint16_t best = 0xFFFF;
  for (uint8_t i = 0; i < 4; i++)
    if (is_valid_mm(z[i]) && z[i] < best) best = z[i];
  return best;
}

static uint8_t count_wall_zones(const uint16_t z[4], uint16_t maxDist) {
  uint8_t n = 0;
  for (uint8_t i = 0; i < 4; i++)
    if (is_valid_mm(z[i]) && z[i] <= maxDist) n++;
  return n;
}

static uint16_t min_valid_idx(const uint16_t z[4], uint8_t i0, uint8_t i1,
                              uint8_t i2 = 255, uint8_t i3 = 255) {
  uint16_t best = 0xFFFF;
  const uint8_t idx[4] = { i0, i1, i2, i3 };
  for (uint8_t k = 0; k < 4; k++) {
    const uint8_t i = idx[k];
    if (i > 3) continue;
    if (is_valid_mm(z[i]) && z[i] < best) best = z[i];
  }
  return best;
}

static void update_front_trend(uint16_t dFront, uint32_t nowMs) {
  if (gPrevFrontMs != 0 && nowMs > gPrevFrontMs &&
      is_valid_mm(gPrevFrontMm) && is_valid_mm(dFront)) {
    const float dt = (float)(nowMs - gPrevFrontMs) / 1000.0f;
    if (dt > 0.01f) {
      float closing = ((float)gPrevFrontMm - (float)dFront) / dt;
      if (closing < 0.0f) closing = 0.0f;
      gFrontClosingMms = 0.78f * gFrontClosingMms + 0.22f * closing;
    }
  } else {
    gFrontClosingMms *= 0.90f;
  }
  gPrevFrontMm = dFront;
  gPrevFrontMs = nowMs;
}

static void update_closing_trend(uint16_t dNow, uint32_t nowMs,
                                 uint16_t& prevMm, uint32_t& prevMs,
                                 float& closingMms) {
  if (prevMs != 0 && nowMs > prevMs &&
      is_valid_mm(prevMm) && is_valid_mm(dNow)) {
    const float dt = (float)(nowMs - prevMs) / 1000.0f;
    if (dt > 0.01f) {
      float closing = ((float)prevMm - (float)dNow) / dt;
      if (closing < 0.0f) closing = 0.0f;
      closingMms = 0.78f * closingMms + 0.22f * closing;
    }
  } else {
    closingMms *= 0.90f;
  }
  prevMm = dNow;
  prevMs = nowMs;
}

static uint16_t gtg_trigger_front_mm(const uint16_t zM[4], int steerServo) {
  // Zone selection depends on steering direction:
  // straight: center pair M1,M2
  // steer-right: right-front pair M0,M1
  // steer-left: left-front pair M2,M3
  //
  // Fallbacks are important when one pair is invalid (8191/open):
  // steer pair -> core pair -> any MID zone.
  uint16_t steerPair = 0xFFFF;
  if (steerServo > (SERVO_STRAIGHT + GTG_STEER_BAND_DEG)) {
    steerPair = min_valid_idx(zM, 0, 1);
  } else if (steerServo < (SERVO_STRAIGHT - GTG_STEER_BAND_DEG)) {
    steerPair = min_valid_idx(zM, 2, 3);
  } else {
    steerPair = min_valid_idx(zM, 1, 2);
  }
  if (is_valid_mm(steerPair)) return steerPair;
  uint16_t corePair = min_valid_idx(zM, 1, 2);
  if (is_valid_mm(corePair)) return corePair;
  return min4_valid(zM);
}

static float clamp01(float v) {
  if (v < 0.0f) return 0.0f;
  if (v > 1.0f) return 1.0f;
  return v;
}

static float close_conf(uint16_t d, uint16_t nearMm, uint16_t farMm) {
  if (!is_valid_mm(d)) return 0.0f;
  if (d <= nearMm) return 1.0f;
  if (d >= farMm) return 0.0f;
  return (float)(farMm - d) / (float)(farMm - nearMm);
}

static float wf_side_away_floor(uint16_t dSideMin,
                                float nearDeg,
                                float veryNearDeg,
                                float critDeg) {
  if (!is_valid_mm(dSideMin)) return 0.0f;
  if (dSideMin <= 150) return critDeg;
  if (dSideMin <= 190) return veryNearDeg;
  if (dSideMin <= 260) return nearDeg;
  if (dSideMin <= 340) {
    const float k = clamp01((340.0f - (float)dSideMin) / 80.0f);
    return nearDeg * k;
  }
  return 0.0f;
}

static float wf_front_away_floor(uint16_t frontMm) {
  if (!is_valid_mm(frontMm)) return 0.0f;
  if (frontMm <= 210) return 27.0f;
  if (frontMm <= 260) return 25.0f;
  if (frontMm <= 340) return 22.0f;
  if (frontMm <= 420) return 19.0f;
  if (frontMm <= 520) return 16.0f;
  if (frontMm <= 650) return 13.0f;
  if (frontMm <= 820) return 8.0f;
  return 0.0f;
}

static float wf_track_precaution_away_floor(uint16_t frontMm, uint16_t dSideMin,
                                            bool active, float uInnerPrecaution) {
  float floorDeg = wf_side_away_floor(dSideMin,
                                      WF_TRACK_SIDE_NEAR_AWAY_DEG,
                                      WF_TRACK_SIDE_VERY_NEAR_AWAY_DEG,
                                      WF_TRACK_SIDE_CRIT_AWAY_DEG);
  if (is_valid_mm(frontMm)) {
    floorDeg = fmaxf(floorDeg, fminf(22.0f, wf_front_away_floor(frontMm)));
    if (frontMm <= 340) floorDeg = fmaxf(floorDeg, WF_TRACK_FRONT_CLOSE_AWAY_DEG);
    else if (frontMm <= 520) floorDeg = fmaxf(floorDeg, WF_TRACK_FRONT_NEAR_AWAY_DEG);
  }
  if (active) {
    floorDeg = fmaxf(floorDeg,
                     fminf(14.0f,
                           fmaxf(WF_INNER_PRECAUTION_MIN_DEG,
                                 fabsf(uInnerPrecaution) * WF_INNER_TRACK_FLOOR_GAIN + 2.0f)));
  }
  return floorDeg;
}

static float wf_fblk_proj_cap(uint16_t frontMm, uint16_t dSideMin) {
  float capDeg = WF_FBLK_PROJ_AWAY_CAP_DEG;
  const float frontFloor = wf_front_away_floor(frontMm);
  const float sideFloor = wf_side_away_floor(dSideMin,
                                             WF_FBLK_SIDE_NEAR_AWAY_DEG,
                                             WF_FBLK_SIDE_VERY_NEAR_AWAY_DEG,
                                             WF_FBLK_SIDE_CRIT_AWAY_DEG);
  capDeg = fmaxf(capDeg, fmaxf(frontFloor, sideFloor));
  return fminf(capDeg, WF_FBLK_AWAY_MAX_DEG);
}

static float wf_inner_fblk_cap(uint16_t frontMm, uint16_t dSideMin,
                               float uInnerPrecaution) {
  float capDeg = clampf(fabsf(uInnerPrecaution) + WF_INNER_FBLK_CAP_PAD_DEG,
                        8.0f, WF_INNER_FBLK_MAX_CAP_DEG);
  capDeg = fmaxf(capDeg, wf_fblk_proj_cap(frontMm, dSideMin));
  return fminf(capDeg, WF_FBLK_AWAY_MAX_DEG);
}

static float mean_conf_idx(const uint16_t z[4], const uint8_t* idx, uint8_t n,
                           uint16_t nearMm, uint16_t farMm) {
  if (n == 0) return 0.0f;
  float s = 0.0f;
  uint8_t c = 0;
  for (uint8_t k = 0; k < n; k++) {
    uint8_t i = idx[k];
    if (i > 3) continue;
    s += close_conf(z[i], nearMm, farMm);
    c++;
  }
  if (c == 0) return 0.0f;
  return s / (float)c;
}

// Weighted bearing of active zones; returns NAN if no valid contribution.
static float weighted_bearing_deg(const uint16_t* vals, const float* degs, uint8_t n,
                                  uint16_t nearMm, uint16_t farMm) {
  float sx = 0.0f, sy = 0.0f;
  bool any = false;
  for (uint8_t i = 0; i < n; i++) {
    float w = close_conf(vals[i], nearMm, farMm);
    if (w <= 0.0f) continue;
    float a = degs[i] * ((float)M_PI / 180.0f);
    sx += w * cosf(a);
    sy += w * sinf(a);
    any = true;
  }
  if (!any) return NAN;
  return atan2f(sy, sx) * 180.0f / (float)M_PI;
}

struct SideWallFit {
  bool valid;
  float distMm;      // Perpendicular distance from robot origin to inferred wall line.
  float tangentDeg;  // Wall tangent angle in robot frame (deg, around 0 when parallel).
  float quality;     // [0..1], confidence from point count + geometric spread.
  uint8_t points;
};

// Fit a side wall line from follow-side zones (+ front-side MID zones) using weighted TLS.
// This gives smoother distance/orientation errors than relying on 2-zone min differences only.
static SideWallFit estimate_side_wall_fit(const uint16_t zFollow[4], const uint16_t zM[4], bool isLeft) {
  SideWallFit out;
  out.valid = false;
  out.distMm = (float)ZONE_WALL;
  out.tangentDeg = 0.0f;
  out.quality = 0.0f;
  out.points = 0;

  static const float kFollowDegL[4] = { 53.0f, 64.0f, 76.0f, 87.0f };
  static const float kFollowDegR[4] = { -87.0f, -76.0f, -64.0f, -53.0f };
  static const float kFrontSideDegL[2] = { 8.0f, 23.0f };
  static const float kFrontSideDegR[2] = { -23.0f, -8.0f };
  static const uint8_t kFrontSideIdxL[2] = { 2, 3 };
  static const uint8_t kFrontSideIdxR[2] = { 0, 1 };

  const float* followDeg = isLeft ? kFollowDegL : kFollowDegR;
  const float* frontSideDeg = isLeft ? kFrontSideDegL : kFrontSideDegR;
  const uint8_t* frontSideIdx = isLeft ? kFrontSideIdxL : kFrontSideIdxR;

  float x[6] = {0};
  float y[6] = {0};
  float w[6] = {0};
  uint8_t n = 0;

  for (uint8_t i = 0; i < 4; i++) {
    uint16_t d = zFollow[i];
    if (!is_valid_mm(d)) continue;
    float baseW = close_conf(d, 220, 1200);
    if (baseW <= 0.0f) continue;
    // De-emphasize front-oblique zones; favor true side-facing zones.
    float sideFactor = clampf((fabsf(followDeg[i]) - 45.0f) / 45.0f, 0.20f, 1.00f);
    float wi = baseW * sideFactor;
    if (wi <= 0.0f) continue;
    float a = followDeg[i] * ((float)M_PI / 180.0f);
    x[n] = (float)d * cosf(a);
    y[n] = (float)d * sinf(a);
    w[n] = wi;
    n++;
  }

  for (uint8_t k = 0; k < 2; k++) {
    uint16_t d = zM[frontSideIdx[k]];
    if (!is_valid_mm(d)) continue;
    float baseW = close_conf(d, 220, 900);
    if (baseW <= 0.0f) continue;
    float wi = 0.45f * baseW;
    if (wi <= 0.0f) continue;
    float a = frontSideDeg[k] * ((float)M_PI / 180.0f);
    x[n] = (float)d * cosf(a);
    y[n] = (float)d * sinf(a);
    w[n] = wi;
    n++;
  }

  if (n < 2) return out;

  float sw = 0.0f, mx = 0.0f, my = 0.0f;
  for (uint8_t i = 0; i < n; i++) {
    sw += w[i];
    mx += w[i] * x[i];
    my += w[i] * y[i];
  }
  if (sw <= 1e-3f) return out;
  mx /= sw;
  my /= sw;

  float sxx = 0.0f, sxy = 0.0f, syy = 0.0f;
  for (uint8_t i = 0; i < n; i++) {
    float dx = x[i] - mx;
    float dy = y[i] - my;
    sxx += w[i] * dx * dx;
    sxy += w[i] * dx * dy;
    syy += w[i] * dy * dy;
  }
  if ((sxx + syy) <= 1e-3f) return out;

  // Principal direction of weighted covariance gives wall tangent.
  float theta = 0.5f * atan2f(2.0f * sxy, (sxx - syy));
  float nx = -sinf(theta);
  float ny =  cosf(theta);
  float signedDist = nx * mx + ny * my;
  float distMm = fabsf(signedDist);
  if (!(distMm >= 40.0f && distMm <= 2200.0f)) return out;

  float tangentDeg = theta * 180.0f / (float)M_PI;
  while (tangentDeg >  90.0f) tangentDeg -= 180.0f;
  while (tangentDeg < -90.0f) tangentDeg += 180.0f;

  float spreadMm = sqrtf((sxx + syy) / sw);
  float qPts = clamp01((float)n / 4.0f);
  float qSpread = clamp01(spreadMm / 140.0f);
  float quality = qPts * qSpread;

  out.valid = true;
  out.distMm = distMm;
  out.tangentDeg = tangentDeg;
  out.quality = quality;
  out.points = n;
  return out;
}

// Zone-level boolean: is zone reading valid and under threshold?
static inline bool zv(uint16_t d, uint16_t thr) {
  return is_valid_mm(d) && d < thr;
}

// ---------------------------------------------------------------------------
// Side choice
// ---------------------------------------------------------------------------
static FollowSide choose_side(const uint16_t zM[4], uint16_t dFront,
                                const uint16_t zL[4], const uint16_t zR[4]) {
  const uint16_t lAll = min4_valid(zL);
  const uint16_t rAll = min4_valid(zR);
  bool lSees = is_valid_mm(lAll) && lAll < SIDE_CHOOSE_MM;
  bool rSees = is_valid_mm(rAll) && rAll < SIDE_CHOOSE_MM;

  if (is_valid_mm(dFront)) {
    if (lSees && lAll > SIDE_FORCE_NEAR_MM && lAll > (uint16_t)(dFront + SIDE_FRONT_MARGIN_MM)) lSees = false;
    if (rSees && rAll > SIDE_FORCE_NEAR_MM && rAll > (uint16_t)(dFront + SIDE_FRONT_MARGIN_MM)) rSees = false;
  }

  if (lSees && !rSees) return FOLLOW_LEFT;
  if (rSees && !lSees) return FOLLOW_RIGHT;

  // MID weighted angle
  static const float kZDeg[4] = { -22.5f, -7.5f, +7.5f, +22.5f };
  float sumW = 0, sumA = 0;
  for (uint8_t i = 0; i < 4; i++) {
    if (!is_valid_mm(zM[i])) continue;
    float w = 1.0f / (float)zM[i];
    sumW += w; sumA += w * kZDeg[i];
  }
  if (sumW > 0) {
    float deg = sumA / sumW;
    if (deg >  3.0f) return FOLLOW_LEFT;
    if (deg < -3.0f) return FOLLOW_RIGHT;
  }

  if (is_valid_mm(lAll) && is_valid_mm(rAll))
    return (lAll <= rAll) ? FOLLOW_LEFT : FOLLOW_RIGHT;

  return gSide;  // keep prior
}

static FollowSide choose_side_sticky(const uint16_t zM[4], uint16_t dFront,
                                     const uint16_t zL[4], const uint16_t zR[4],
                                     uint32_t nowMs) {
  FollowSide picked = choose_side(zM, dFront, zL, zR);
  if (picked == gSide) return picked;

  const uint16_t curMin = (gSide == FOLLOW_LEFT) ? min4_valid(zL) : min4_valid(zR);
  const uint16_t newMin = (picked == FOLLOW_LEFT) ? min4_valid(zL) : min4_valid(zR);
  const bool curSees = is_valid_mm(curMin) && curMin < SIDE_CHOOSE_MM;
  const bool newSees = is_valid_mm(newMin) && newMin < SIDE_CHOOSE_MM;
  const bool withinSticky = (nowMs - gSideLastSetMs) < SIDE_STICKY_MS;
  const bool strongBetter =
      (!curSees && newSees) ||
      (curSees && newSees && ((int)newMin + (int)SIDE_SWITCH_MARGIN_MM < (int)curMin));

  if (withinSticky && !strongBetter) return gSide;
  return picked;
}

// +1 means "command right-turn", -1 means "command left-turn".
// Pick turn-away direction from instantaneous clearance, with side fallback.
static int away_sign_from_clearance(const uint16_t zL[4], const uint16_t zR[4], FollowSide sideFallback) {
  const uint16_t lMin = min4_valid(zL);
  const uint16_t rMin = min4_valid(zR);
  const bool lValid = is_valid_mm(lMin);
  const bool rValid = is_valid_mm(rMin);

  if (lValid && rValid) return (lMin <= rMin) ? +1 : -1;
  if (lValid) return +1;
  if (rValid) return -1;
  return (sideFallback == FOLLOW_LEFT) ? +1 : -1;
}

// +1 means "turn right", -1 means "turn left", 0 means "no strong asymmetry".
// Uses MID-front asymmetry so FRONT_BLOCK can prefer the open side.
static uint16_t front_half_effective_mm(uint16_t center, uint16_t edge) {
  const bool cValid = is_valid_mm(center);
  const bool eValid = is_valid_mm(edge);
  if (cValid && eValid) {
    // Prefer center zone, blend edge mildly.
    return (uint16_t)clampi((int)roundf(0.70f * (float)center + 0.30f * (float)edge), 1, 8190);
  }
  if (cValid) return center;
  if (eValid) {
    // Edge-only detection is less reliable for turn commitment.
    return (uint16_t)clampi((int)edge + 140, 1, 8190);
  }
  return 0xFFFF;
}

static int front_open_turn_sign(const uint16_t zM[4]) {
  const bool coreRightValid = is_valid_mm(zM[1]);
  const bool coreLeftValid  = is_valid_mm(zM[2]);
  // Without any core-front evidence (M1/M2), edge-only asymmetry is too noisy
  // for directional commitments.
  if (!coreRightValid && !coreLeftValid) return 0;

  const uint16_t rFront = front_half_effective_mm(zM[1], zM[0]); // right half, center-weighted
  const uint16_t lFront = front_half_effective_mm(zM[2], zM[3]); // left half, center-weighted
  const bool rValid = is_valid_mm(rFront);
  const bool lValid = is_valid_mm(lFront);
  static constexpr int FRONT_ASYM_MM = 45;

  if (lValid && rValid) {
    if ((int)lFront + FRONT_ASYM_MM < (int)rFront) return +1;
    if ((int)rFront + FRONT_ASYM_MM < (int)lFront) return -1;
    return 0;
  }
  if (lValid && !rValid) return +1;
  if (rValid && !lValid) return -1;
  return 0;
}

// ---------------------------------------------------------------------------
// M-line exit check (Bug2 compliance)
// ---------------------------------------------------------------------------
static float point_line_distance(float px, float py,
                                 float x1, float y1, float x2, float y2) {
  float dx = x2 - x1, dy = y2 - y1;
  float den = sqrtf(dx * dx + dy * dy);
  if (den < 1e-4f) return 1e9f;
  return fabsf(dy * px - dx * py + x2 * y1 - y2 * x1) / den;
}

static bool should_leave_wall(const Pose& robot, uint16_t dFront) {
  if (!gHitValid) return false;
  if (is_valid_mm(dFront) && dFront <= 800) return false;
  float dGoal = dist2(robot.x, robot.y, gGoalX, gGoalY);
  if (dGoal > (gHitGoalDistM - 0.10f)) return false;
  float mErr = point_line_distance(robot.x, robot.y,
                                   gStartX, gStartY, gGoalX, gGoalY);
  if (mErr > MLINE_TOL_M) return false;
  return true;
}

static void note_hit_point(const Pose& robot, float dGoal) {
  gHitGoalDistM = dGoal;
  gHitX = robot.x;
  gHitY = robot.y;
  gHitValid = true;

  char ev[96];
  snprintf(ev, sizeof(ev),
           "HIT_POINT x=%.2f y=%.2f dg=%.2f",
           robot.x, robot.y, dGoal);
  log_decision(ev);
}

static void note_leave_point(const Pose& robot) {
  const uint32_t nowMs = millis();
  const bool farFromLast = !gLeaveCandValid ||
                           (dist2(robot.x, robot.y, gLeaveCandX, gLeaveCandY) > 0.20f);
  const bool longSinceLast = !gLeaveCandValid ||
                             ((uint32_t)(nowMs - gLeaveCandMs) > 1200);
  if (!(farFromLast || longSinceLast)) return;

  gLeaveCandValid = true;
  gLeaveCandX = robot.x;
  gLeaveCandY = robot.y;
  gLeaveCandMs = nowMs;

  char ev[128];
  snprintf(ev, sizeof(ev),
           "LEAVE_POINT x=%.2f y=%.2f hit=(%.2f,%.2f)",
           robot.x, robot.y, gHitX, gHitY);
  log_decision(ev);
}

// ---------------------------------------------------------------------------
// set_state
// ---------------------------------------------------------------------------
static void set_state(FreshState s) {
  fs = s;
  stEnterMs = millis();
  wallLostCnt = 0;
  if (s == FS_GTG) gGtgServoCmd = SERVO_STRAIGHT;
  stallMs = millis();
  stallLastX = 0; stallLastY = 0;
  gWfDeadendActive = false;
  gWfDeadendMs = 0;
  gWfMode = WF_TRACK;
  gWfModeSinceMs = stEnterMs;
  gWfDendCandCnt = 0;
  gWfServoCmd = SERVO_STRAIGHT;
  gWfFblkSign = 0;
  gWfCapturedWall = false;
  gWfAlignHoldTicks = 0;
  gWfAlignNeutralTicks = 0;
  gWfAlignServo = SERVO_STRAIGHT;
  gWfSideSettleTicks = 0;
  gWfSideHardCooldownTicks = 0;
  gWfEmergCarrySign = 0;
  gWfEmergCarryUntilMs = 0;
  gWfEmergLastRevServoSign = 0;
  gWfEmergLastRevMs = 0;
  gWfRecoverTowardUntilMs = 0;
  gWfSeekTowardUntilMs = 0;
  gWfOuterEdgeGripUntilMs = 0;
  gWfAcquireGuardUntilMs = 0;
  gWfAcqFrontConfirmTicks = 0;
  gWfPostFblkDampUntilMs = 0;
  gWfAntiSwingUntilMs = 0;
  wf_reset_loss_anchor();
  gWfLastSensorFrame = 0;
  gPrevFrontMm = 0xFFFF;
  gPrevFrontMs = 0;
  gFrontClosingMms = 0.0f;
  gPrevM0Mm = 0xFFFF;
  gPrevM3Mm = 0xFFFF;
  gPrevM0Ms = 0;
  gPrevM3Ms = 0;
  gM0ClosingMms = 0.0f;
  gM3ClosingMms = 0.0f;

  if (s == FS_STUCK) {
    stuckPhaseMs = 0;
    stuckReversing = false;
  }

  switch (s) {
    case FS_IDLE:    gS = BUG2_IDLE; break;
    case FS_GTG:     gS = BUG2_GO_TO_GOAL; break;
    case FS_WF:      gS = BUG2_FOLLOW_WALL; break;
    case FS_REACHED: gS = BUG2_REACHED; break;
    case FS_STUCK:   gS = BUG2_STUCK; break;
  }
}

static const char* fs_name(FreshState s) {
  switch (s) {
    case FS_IDLE:    return "IDLE";
    case FS_GTG:     return "GO_TO_GOAL";
    case FS_WF:      return "WALL_FOLLOW";
    case FS_REACHED: return "DONE";
    case FS_STUCK:   return "STUCK";
    default:         return "?";
  }
}

void bug2_wf_tune_get(Bug2WfTune& out) {
  out.target_mm = gWfTargetMm;
  out.dist_k = gWfDistK;
  out.yaw_k = gWfYawK;
  out.bear_k = gWfBearK;
  out.opp_k = gWfOppK;
  out.front_k = gWfFrontK;
  out.zone_emergency_mm = gZoneEmergency;
  out.zone_close_mm = gZoneClose;
  out.zone_mid_mm = gZoneMid;
  out.deadend_rev_ms = gWfDeadendRevMs;
  out.deadend_turn_ms = gWfDeadendTurnMs;
  out.mode_dwell_ms = gWfModeDwellMs;
  out.fblk_enter_conf = gWfFblkEnterCoreConf;
  out.fblk_exit_conf = gWfFblkExitCoreConf;
  out.fblk_enter_mm = gWfFblkEnterFrontMm;
  out.fblk_exit_mm = gWfFblkExitFrontMm;
  out.dend_enter_front_conf = gWfDendEnterFrontConf;
  out.dend_enter_opp_conf = gWfDendEnterOppConf;
  out.dend_enter_follow_conf = gWfDendEnterFollowConf;
  out.dend_exit_front_conf = gWfDendExitFrontConf;
  out.dend_exit_opp_conf = gWfDendExitOppConf;
}

void bug2_wf_tune_set(const Bug2WfTune& in) {
  gWfTargetMm = clampf(in.target_mm, 120.0f, 600.0f);
  gWfDistK = clampf(in.dist_k, 0.0f, 20.0f);
  gWfYawK = clampf(in.yaw_k, 0.0f, 20.0f);
  gWfBearK = clampf(in.bear_k, 0.0f, 2.0f);
  gWfOppK = clampf(in.opp_k, 0.0f, 20.0f);
  gWfFrontK = clampf(in.front_k, 0.0f, 20.0f);
  gZoneEmergency = (uint16_t)clampi((int)in.zone_emergency_mm, 60, 500);
  gZoneClose = (uint16_t)clampi((int)in.zone_close_mm, 120, 900);
  gZoneMid = (uint16_t)clampi((int)in.zone_mid_mm, 150, 1200);
  if (gZoneEmergency > gZoneClose) gZoneEmergency = gZoneClose;
  if (gZoneClose > gZoneMid) gZoneClose = gZoneMid;
  gWfDeadendRevMs = (uint16_t)clampi((int)in.deadend_rev_ms, 150, 2500);
  gWfDeadendTurnMs = (uint16_t)clampi((int)in.deadend_turn_ms, 150, 2500);
  gWfModeDwellMs = (uint16_t)clampi((int)in.mode_dwell_ms, 0, 1500);
  gWfFblkEnterCoreConf = clampf(in.fblk_enter_conf, 0.0f, 1.0f);
  gWfFblkExitCoreConf = clampf(in.fblk_exit_conf, 0.0f, 1.0f);
  gWfFblkEnterFrontMm = (uint16_t)clampi((int)in.fblk_enter_mm, 120, 1200);
  gWfFblkExitFrontMm = (uint16_t)clampi((int)in.fblk_exit_mm, 120, 1400);
  gWfDendEnterFrontConf = clampf(in.dend_enter_front_conf, 0.0f, 1.0f);
  gWfDendEnterOppConf = clampf(in.dend_enter_opp_conf, 0.0f, 1.0f);
  gWfDendEnterFollowConf = clampf(in.dend_enter_follow_conf, 0.0f, 1.0f);
  gWfDendExitFrontConf = clampf(in.dend_exit_front_conf, 0.0f, 1.0f);
  gWfDendExitOppConf = clampf(in.dend_exit_opp_conf, 0.0f, 1.0f);

  // Keep hysteresis ordering sane.
  if (gWfFblkExitCoreConf > gWfFblkEnterCoreConf) gWfFblkExitCoreConf = gWfFblkEnterCoreConf;
  if (gWfFblkEnterFrontMm > gWfFblkExitFrontMm) gWfFblkEnterFrontMm = gWfFblkExitFrontMm;
  if (gWfDendExitFrontConf > gWfDendEnterFrontConf) gWfDendExitFrontConf = gWfDendEnterFrontConf;
  if (gWfDendExitOppConf > gWfDendEnterOppConf) gWfDendExitOppConf = gWfDendEnterOppConf;
}

// ============================= PUBLIC API ================================

void bug2_setup() {
  gHasGoal = false;
  gHitGoalDistM = 1e9f;
  gHitX = gHitY = 0.0f;
  gHitValid = false;
  gGoalX = gGoalY = 0.0f;
  gStartX = gStartY = 0.0f;
  gLeaveCandValid = false;
  gLeaveCandX = gLeaveCandY = 0.0f;
  gLeaveCandMs = 0;
  gSide = FOLLOW_RIGHT;
  gSideLastSetMs = millis();
  gBug2Mode = BUG2_MODE_FULL;
  set_state(FS_IDLE);
  log_decision("SETUP");
}

void bug2_start(float goalX, float goalY) {
  gGoalX = goalX;
  gGoalY = goalY;
  gHasGoal = true;
  // New run should not inherit previous run's follow side.
  gSide = FOLLOW_RIGHT;
  gSideLastSetMs = millis();
  gLeaveCandValid = false;
  gLeaveCandX = gLeaveCandY = 0.0f;
  gLeaveCandMs = 0;
#if !BUG2_NO_POSE
  Pose r = snapshotPose(gRobotLatest);
  gStartX = r.x;
  gStartY = r.y;
  gHitGoalDistM = 1e9f;
  gHitX = gHitY = 0.0f;
  gHitValid = false;
#endif
  if (gBug2Mode == BUG2_MODE_WF_ONLY) {
    set_state(FS_WF);
    log_decision("START_WF_ONLY");
  } else {
    set_state(FS_GTG);
    log_decision("START");
  }
}

void bug2_stop(const char* why) {
  if (!why) why = "bug2_stop";
  movement_stop_reason(why, true);
  gHasGoal = false;
  set_state(FS_IDLE);
  log_decision("STOP");
}

Bug2State bug2_state() { return gS; }

void bug2_set_mode(Bug2RunMode mode) {
  gBug2Mode = mode;
}

Bug2RunMode bug2_run_mode() { return gBug2Mode; }

bool bug2_has_goal(float& x, float& y) {
  if (!gHasGoal) return false;
  x = gGoalX; y = gGoalY;
  return true;
}

// ============================= MAIN TICK =================================

void bug2_tick(const Pose& robot, bool started,
               uint16_t dL, uint16_t dM, uint16_t dR) {
  // --- not started  idle ---
  if (!started) {
    if (fs != FS_IDLE) set_state(FS_IDLE);
    movement_stop_reason("bug2_idle", true);
    return;
  }
  if (!gHasGoal) {
    movement_stop_reason("bug2_no_goal", true);
    set_state(FS_IDLE);
    return;
  }

  // --- snapshot zone arrays under noInterrupts ---
  uint16_t zL[4], zM[4], zR[4];
  noInterrupts();
  memcpy(zL, (const void*)gSensL_z4.z, sizeof(zL));
  memcpy(zM, (const void*)gSensM_z4.z, sizeof(zM));
  memcpy(zR, (const void*)gSensR_z4.z, sizeof(zR));
  interrupts();

  // --- derived front distance (best of zone or legacy scalar) ---
  const uint16_t frontZ = min4_valid(zM);
  const uint16_t dFront = is_valid_mm(frontZ) ? frontZ : dM;

  const uint32_t nowMs = millis();
  const uint32_t sensFrame = snapshotU32(gSensFrameId);
  const bool freshSensFrame = (sensFrame != gWfLastSensorFrame);
  if (freshSensFrame) gWfLastSensorFrame = sensFrame;
  if ((fs == FS_GTG) ||
      ((fs == FS_WF) && (gBug2Mode == BUG2_MODE_WF_ONLY) &&
       !gWfCapturedWall && ((nowMs - stEnterMs) < SIDE_STICKY_MS))) {
    // Refresh preferred follow side during GTG and at the beginning of direct
    // WF-only runs so startup does not inherit a stale side.
    FollowSide pickedSide = choose_side_sticky(zM, dFront, zL, zR, nowMs);
    if (pickedSide != gSide) {
      gSide = pickedSide;
      gSideLastSetMs = nowMs;
    }
  }

  // --- follow-side info ---
  const bool isLeft = (gSide == FOLLOW_LEFT);
  const uint16_t* zFollow = isLeft ? zL : zR;
  const uint16_t* zAway   = isLeft ? zR : zL;
  const uint16_t dSideAll = min4_valid(zFollow);
  // Prefer side-core zones (index 1/2) over front-oblique for stable wall distance.
  const uint16_t dSideCore = min_valid_idx(zFollow, 1, 2);
  const uint16_t dSideMin = is_valid_mm(dSideCore) ? dSideCore : dSideAll;
  const uint8_t  nFollow  = count_wall_zones(zFollow, ZONE_WALL);

  update_front_trend(dFront, nowMs);
  update_closing_trend(zM[0], nowMs, gPrevM0Mm, gPrevM0Ms, gM0ClosingMms);
  update_closing_trend(zM[3], nowMs, gPrevM3Mm, gPrevM3Ms, gM3ClosingMms);

  // --- debug line (printed every tick) ---
  snprintf(gBug2Dbg, sizeof(gBug2Dbg),
           "%s side=%c dF=%u dS=%u wz=%u lost=%u r(%.2f,%.2f)",
           fs_name(fs),
           isLeft ? 'L' : 'R',
           (unsigned)dFront, (unsigned)dSideMin,
           (unsigned)nFollow, (unsigned)wallLostCnt,
           robot.x, robot.y);

  // --- Position stall detection (requires valid pose) ---
#if !BUG2_NO_POSE
  {
    float sDist = dist2(robot.x, robot.y, stallLastX, stallLastY);
    if (sDist > STALL_DIST_M) {
      stallLastX = robot.x;
      stallLastY = robot.y;
      stallMs = nowMs;
    }
    if ((fs == FS_WF || fs == FS_GTG) && (nowMs - stallMs > STALL_TIME_MS)) {
      log_decision("STALL>STUCK");
      set_state(FS_STUCK);
    }
  }
#endif

  // ===================== STATE MACHINE =====================
  switch (fs) {

  // ---------------------------------------------------------
  case FS_IDLE: {
    if (gBug2Mode == BUG2_MODE_WF_ONLY) {
      set_state(FS_WF);
      log_decision("IDLE>WF_ONLY");
    } else {
      set_state(FS_GTG);
      log_decision("IDLE>GTG");
    }
  } break;

  // ---------------------------------------------------------
  // Go-To-Goal: PP toward goal.  On obstacle  choose side  WF
  // ---------------------------------------------------------
  case FS_GTG: {
#if !BUG2_NO_POSE
    // --- Full pose mode: PP toward goal ---
    Pose goalPose = { gGoalX, gGoalY, 0.0f, 1 };
    int ppAngle = SERVO_STRAIGHT;
    float dGoal = 0.0f;
    if (!pp_compute_angle(robot, goalPose, BUG2_PP, ppAngle, dGoal)) {
      movement_stop_reason("bug2_pp_fail", true);
      break;
    }

    // Goal reached?
    if (dGoal < GOAL_REACHED_M) {
      movement_stop_reason("bug2_reached", true);
      noInterrupts(); gStarted = false; interrupts();
      gHasGoal = false;
      set_state(FS_REACHED);
      log_decision("GOAL_REACHED");
      break;
    }
#else
    // --- No-pose mode: drive straight, no goal-reached check ---
    int ppAngle = SERVO_STRAIGHT;
#endif

    // --- Obstacle / asymmetry anticipation  enter WF ---
    int gtgAngle = ppAngle;
    int gtgOpenSign = front_open_turn_sign(zM);
    const uint16_t gtgFrontRightMm = front_half_effective_mm(zM[1], zM[0]);
    const uint16_t gtgFrontLeftMm  = front_half_effective_mm(zM[2], zM[3]);
    // In near-side corridors, avoid abrupt bias flips across straight unless
    // opposite-side opening is clearly stronger. This prevents GTG from snapping
    // from e.g. +right bias to hard-left before WF handoff.
    if (is_valid_mm(dSideMin) && dSideMin <= (uint16_t)(GTG_SIDE_TRIGGER_MM + 120)) {
      if (!isLeft && gtgOpenSign < 0) {
        bool strongLeftNeed = false;
        if (is_valid_mm(gtgFrontRightMm) && is_valid_mm(gtgFrontLeftMm)) {
          strongLeftNeed = (gtgFrontRightMm <= 340) &&
                           (((int)gtgFrontLeftMm - (int)gtgFrontRightMm) >= 120);
        } else if (is_valid_mm(gtgFrontRightMm) && !is_valid_mm(gtgFrontLeftMm)) {
          strongLeftNeed = (gtgFrontRightMm <= 300);
        }
        if (!strongLeftNeed) gtgOpenSign = 0;
      } else if (isLeft && gtgOpenSign > 0) {
        bool strongRightNeed = false;
        if (is_valid_mm(gtgFrontRightMm) && is_valid_mm(gtgFrontLeftMm)) {
          strongRightNeed = (gtgFrontLeftMm <= 340) &&
                            (((int)gtgFrontRightMm - (int)gtgFrontLeftMm) >= 120);
        } else if (is_valid_mm(gtgFrontLeftMm) && !is_valid_mm(gtgFrontRightMm)) {
          strongRightNeed = (gtgFrontLeftMm <= 300);
        }
        if (!strongRightNeed) gtgOpenSign = 0;
      }
    }
    const bool gtgFrontNudge =
        (gtgOpenSign != 0) &&
        is_valid_mm(dFront) &&
        (dFront <= GTG_TRIGGER_EARLY_MM) &&
        ((gFrontClosingMms > 120.0f) ||
         (dFront <= (uint16_t)(GTG_TRIGGER_MM + 40)));
    if (gtgFrontNudge) {
      int biasMag = clampi((int)((GTG_TRIGGER_EARLY_MM - dFront) / 45),
                           0, GTG_FRONT_NUDGE_MAX_DEG);
      gtgAngle = clampi(ppAngle + gtgOpenSign * biasMag, SERVO_LEFT, SERVO_RIGHT);
    }
#if BUG2_NO_POSE
    // Mild side-distance centering in GTG:
    // if we are far from the chosen follow side, bias gently toward it before crisis.
    if (is_valid_mm(dSideMin) && is_valid_mm(dFront) && dFront <= GTG_PRE_BIAS_MM) {
      int gtgSideSign = isLeft ? +1 : -1; // +1:right command, -1:left command
      float eSide = gWfTargetMm - (float)dSideMin;
      float sideBias = -gtgSideSign * GTG_SIDE_BIAS_K * clampf((-eSide) / 180.0f, 0.0f, 1.4f);
      // If selected side is very open while front is approaching, bias harder toward
      // that side so GTG does not drift straight too long before WF handoff.
      float sideFarBoost = clamp01(((float)dSideMin - (gWfTargetMm + 360.0f)) / 520.0f);
      float frontApproach = clamp01((GTG_PRE_BIAS_MM - (float)dFront) / 360.0f);
      sideBias *= (1.0f + 1.8f * sideFarBoost * frontApproach);
      gtgAngle = clampi((int)roundf((float)gtgAngle + sideBias), SERVO_LEFT, SERVO_RIGHT);
    }
#endif
    int gtgSlew = GTG_BIAS_SLEW_DEG;
    if (is_valid_mm(dFront)) {
      if (dFront < 700) gtgSlew = 4;
      if (dFront < 520) gtgSlew = 3;
    }
    gtgAngle = clampi(gtgAngle, gGtgServoCmd - gtgSlew, gGtgServoCmd + gtgSlew);
    gGtgServoCmd = gtgAngle;

    // Front trigger uses zones based on GTG steering command.
    const uint16_t frontTrigMm = gtg_trigger_front_mm(zM, gtgAngle);
    int gtgTriggerMm = GTG_TRIGGER_MM;
    if (gtgOpenSign != 0) gtgTriggerMm += 90;
    if (gFrontClosingMms >= GTG_FRONT_FAST_MMPS) gtgTriggerMm += 60;
    gtgTriggerMm = clampi(gtgTriggerMm, 500, 900);
    const bool frontTrig = is_valid_mm(frontTrigMm) && frontTrigMm <= (uint16_t)gtgTriggerMm;
    const bool prepTrig = is_valid_mm(dFront) && dFront <= GTG_TRIGGER_EARLY_MM &&
                          (gtgOpenSign != 0) &&
                          ((gFrontClosingMms > 120.0f) || (dFront <= (uint16_t)(GTG_TRIGGER_MM + 40))) &&
                          is_valid_mm(dSideMin) && dSideMin <= GTG_SIDE_TRIGGER_MM;
    const bool sideOpenTrig = is_valid_mm(dSideMin) &&
                              (dSideMin >= (uint16_t)(gWfTargetMm + 520.0f)) &&
                              is_valid_mm(dFront) &&
                              (dFront <= GTG_TRIGGER_EARLY_MM) &&
                              ((gFrontClosingMms > 80.0f) || (dFront <= (uint16_t)(GTG_TRIGGER_MM + 60)));
    const bool sideNearTrig = is_valid_mm(dSideMin) &&
                              (dSideMin <= GTG_SIDE_TRIGGER_MM) &&
                              is_valid_mm(dFront) &&
                              (dFront <= GTG_SIDE_NEAR_ENTER_MM);

    // Side-danger guard:
    // left: L0,L1,L2 (front/forward/side)
    // right: R1,R2,R3 (side/forward/front)
    const uint16_t lDangerMm = min_valid_idx(zL, 0, 1, 2);
    const uint16_t rDangerMm = min_valid_idx(zR, 1, 2, 3);
    const bool lDanger = is_valid_mm(lDangerMm) && lDangerMm <= GTG_SIDE_DANGER_MM;
    const bool rDanger = is_valid_mm(rDangerMm) && rDangerMm <= GTG_SIDE_DANGER_MM;

    if (frontTrig || prepTrig || sideNearTrig || sideOpenTrig || lDanger || rDanger) {
      FollowSide newSide = gSide;
      if (lDanger && !rDanger) {
        newSide = FOLLOW_LEFT;
      } else if (rDanger && !lDanger) {
        newSide = FOLLOW_RIGHT;
      } else if (sideOpenTrig) {
        // Side-open handoff usually keeps the current follow side, but do not let
        // that override clear opposite-side evidence when the current side is the
        // one that just opened/lost.
        FollowSide candSide = choose_side_sticky(zM, dFront, zL, zR, nowMs);
        const uint16_t curSideMin = dSideMin;
        const uint16_t oppSideMin = min4_valid(zAway);
        const bool curVeryOpen = !is_valid_mm(curSideMin) ||
                                 (curSideMin >= (uint16_t)(gWfTargetMm + 520.0f));
        const bool oppSeesWall = is_valid_mm(oppSideMin) && (oppSideMin < SIDE_CHOOSE_MM);
        const bool oppClearlyBetter = is_valid_mm(oppSideMin) &&
                                      (!is_valid_mm(curSideMin) ||
                                       ((int)oppSideMin + (int)SIDE_SWITCH_MARGIN_MM < (int)curSideMin));
        if ((candSide != gSide) && curVeryOpen && (oppSeesWall || oppClearlyBetter)) {
          newSide = candSide;
        } else {
          newSide = gSide;
        }
      } else {
        FollowSide candSide = choose_side_sticky(zM, dFront, zL, zR, nowMs);
#if BUG2_NO_POSE
        // In no-pose testing, avoid easy side flips at GTG->WF handoff.
        // Keep chosen side unless current side is truly dangerous or lost.
        if (candSide != gSide) {
          const uint16_t curSideMin = dSideMin;
          const uint16_t oppSideMin = min4_valid(zAway);
          const bool curDanger = is_valid_mm(curSideMin) && (curSideMin <= GTG_SIDE_DANGER_MM);
          const bool curLost = !is_valid_mm(curSideMin) || (curSideMin > SIDE_CHOOSE_MM);
          const bool oppClearlyBetter = is_valid_mm(oppSideMin) &&
                                        (!is_valid_mm(curSideMin) ||
                                         ((int)oppSideMin + (int)SIDE_SWITCH_MARGIN_MM < (int)curSideMin));
          if (!(curDanger || (curLost && oppClearlyBetter))) {
            candSide = gSide;
          }
        }
#endif
        newSide = candSide;
      }
      if (newSide != gSide) {
        gSide = newSide;
        gSideLastSetMs = nowMs;
      }
#if !BUG2_NO_POSE
      if (gBug2Mode == BUG2_MODE_FULL) {
        note_hit_point(robot, dGoal);
      }
#endif
      if (lDanger || rDanger) {
        log_decision(gSide == FOLLOW_LEFT ? "SIDE>WF_L" : "SIDE>WF_R");
      } else if (sideOpenTrig) {
        log_decision(gSide == FOLLOW_LEFT ? "OPEN>WF_L" : "OPEN>WF_R");
      } else if (prepTrig || sideNearTrig) {
        log_decision(gSide == FOLLOW_LEFT ? "PREP>WF_L" : "PREP>WF_R");
      } else {
        log_decision(gSide == FOLLOW_LEFT ? "OBS>WF_L" : "OBS>WF_R");
      }
      set_state(FS_WF);
      // First WF tick will pick a combo immediately.
      break;
    }

    // --- Normal GTG drive (decelerate near obstacle) ---
    float sp = SPD_MED;
    if (is_valid_mm(dFront)) {
      if (dFront < 1200) sp = 0.50f;
      if (dFront <  900) sp = 0.35f;
      if (dFront <  700) sp = 0.20f;
      if (dFront <  620) sp = 0.16f;
      if (dFront <  520) sp = 0.12f;
      if (gFrontClosingMms > 260.0f && dFront < 700) sp = fminf(sp, 0.12f);
    }
    snprintf(gBug2Dbg, sizeof(gBug2Dbg),
             "sv=%d L=[%u,%u,%u,%u] M=[%u,%u,%u,%u] R=[%u,%u,%u,%u] GTG side=%c dF=%u dS=%u wz=%u lost=%u r(%.2f,%.2f)",
             gtgAngle,
             (unsigned)zL[0], (unsigned)zL[1], (unsigned)zL[2], (unsigned)zL[3],
             (unsigned)zM[0], (unsigned)zM[1], (unsigned)zM[2], (unsigned)zM[3],
             (unsigned)zR[0], (unsigned)zR[1], (unsigned)zR[2], (unsigned)zR[3],
             isLeft ? 'L' : 'R',
             (unsigned)dFront, (unsigned)dSideMin,
             (unsigned)nFollow, (unsigned)wallLostCnt,
             robot.x, robot.y);
    movement_apply_angle(clampi(gtgAngle, SERVO_LEFT, SERVO_RIGHT), "bug2_gtg", sp);
  } break;

  // ---------------------------------------------------------
  // Wall Follow: feature/mode controller (inner/outer/deadend aware)
  // ---------------------------------------------------------
  case FS_WF: {
    // --- M-line leave-point detection (requires pose) ---
#if !BUG2_NO_POSE
    if (gBug2Mode == BUG2_MODE_FULL) {
      if (should_leave_wall(robot, dFront)) {
        note_leave_point(robot);
        set_state(FS_GTG);
        log_decision("WF>GTG_LEAVE");
        break;
      }
    }
#endif

    // --- Track wall visibility ---
    if (nFollow >= 1) {
      wallLostCnt = 0;
    } else {
      if (wallLostCnt < 255) wallLostCnt++;
    }

    const bool openSpaceNoWall =
        (wallLostCnt > 5) &&
        !is_valid_mm(dSideMin) &&
        !is_valid_mm(min4_valid(zAway)) &&
        !is_valid_mm(dFront);

    // Wall lost too long  STUCK
    if ((wallLostCnt > WF_LOST_LIMIT * 2) && !openSpaceNoWall) {
      log_decision("WF_LOST>STUCK");
      set_state(FS_STUCK);
      break;
    }

    // Testing mode: when the wall opens completely, keep reacquiring instead of
    // leaving to GTG. If this also happens to satisfy leave-point geometry,
    // the leave candidate is already noted above and we keep following.
    if (openSpaceNoWall) {
      if (wallLostCnt == 6) {
        log_decision("WF_OPEN_KEEP_FOLLOW");
      }
      if (gWfMode != WF_OUTER_SEEK) {
        wf_note_loss_anchor(robot, nowMs, isLeft, dFront, dSideMin, nFollow);
        gWfMode = WF_OUTER_SEEK;
        gWfModeSinceMs = nowMs;
        log_decision("WF_MODE_OUT");
      }
    }

    const uint16_t F0 = zFollow[0], F1 = zFollow[1], F2 = zFollow[2], F3 = zFollow[3];
    const uint16_t A0 = zAway[0],   A1 = zAway[1],   A2 = zAway[2],   A3 = zAway[3];
    const uint16_t M0 = zM[0],      M1 = zM[1],      M2 = zM[2],      M3 = zM[3];

    // Emergency always wins.
    // Important: reverse is fallback only for truly critical front/side.
    // For non-critical emergency, keep a forward pivot-away to avoid FBLK->EMERG
    // immediate direction flip that can scrape the followed wall.
    const bool eM0 = zv(M0, gZoneEmergency);
    const bool eM1 = zv(M1, gZoneEmergency);
    const bool eM2 = zv(M2, gZoneEmergency);
    const bool eM3 = zv(M3, gZoneEmergency);
    const uint8_t eCnt = (eM0 ? 1 : 0) + (eM1 ? 1 : 0) + (eM2 ? 1 : 0) + (eM3 ? 1 : 0);
    // Emergency should not trigger from a single edge-only hit (M0/M3) which is often
    // a side wall grazing the FoV. Require a core hit or multi-zone emergency evidence.
    if (eM1 || eM2 || (eCnt >= 2)) {
      const int awaySign = away_sign_from_clearance(zL, zR, gSide);
      const int openSign = front_open_turn_sign(zM);
      const bool cM0 = zv(M0, WF_EMERG_REV_FRONT_MM);
      const bool cM1 = zv(M1, WF_EMERG_REV_FRONT_MM);
      const bool cM2 = zv(M2, WF_EMERG_REV_FRONT_MM);
      const bool cM3 = zv(M3, WF_EMERG_REV_FRONT_MM);
      const uint8_t cCnt = (cM0 ? 1 : 0) + (cM1 ? 1 : 0) + (cM2 ? 1 : 0) + (cM3 ? 1 : 0);
      const bool frontCrit = cM1 || cM2 || (cCnt >= 2);
      const bool frontNearRev = is_valid_mm(dFront) && (dFront <= WF_EMERG_REV_NEAR_MM);
      const bool frontForceRev = is_valid_mm(dFront) && (dFront <= WF_EMERG_REV_FORCE_MM);
      const bool sideCrit = is_valid_mm(dSideMin) && (dSideMin <= WF_SIDE_CRIT_MM);
      const bool recentFblk = (gWfMode == WF_FRONT_BLOCK) &&
                              ((nowMs - gWfModeSinceMs) < WF_EMERG_FBLK_GUARD_MS);
      // Reverse only when truly boxed-in: side-critical OR critical front with
      // no clear turn to one side. This keeps reverse as rare fallback.
      // Also reverse when front is imminently near to avoid grazing impact.
      const bool nearNoTurnRoom = frontNearRev &&
                                  ((openSign == 0) ||
                                   (is_valid_mm(dSideMin) && (dSideMin <= (uint16_t)(gWfTargetMm + 60.0f))));
      const bool needReverse = frontForceRev || sideCrit || (frontCrit && (openSign == 0)) || nearNoTurnRoom;

      if (needReverse) {
        // Reverse toward the followed side, then carry the next forward escape
        // away from it. Front asymmetry is noisy at contact distance and caused
        // left/right reverse flips in new87.
        int revServoSign = (gSide == FOLLOW_RIGHT) ? +1 : -1;
        // Keep the reverse direction latched while the front is still critical.
        // Alternating reverse servo signs on consecutive ticks cancels the escape
        // and was visible as bump/re-bump oscillation.
        const bool recentRev =
            (gWfEmergLastRevServoSign != 0) &&
            ((nowMs - gWfEmergLastRevMs) <= WF_EMERG_LAST_REV_MS);
        if (recentRev && (frontNearRev || frontCrit)) {
          revServoSign = (int)gWfEmergLastRevServoSign;
        }
        int revSign = -revServoSign;
        gWfEmergCarrySign = (int8_t)revSign;
        gWfEmergCarryUntilMs = nowMs + 500;
        gWfRecoverTowardUntilMs = nowMs + WF_RECOVER_TOWARD_MS;
        int revServo = clampi(SERVO_STRAIGHT + revServoSign * 20, SERVO_LEFT, SERVO_RIGHT);
        gWfEmergLastRevServoSign = (int8_t)revServoSign;
        gWfEmergLastRevMs = nowMs;
        movement_apply_reverse(revServo, "bug2_wf_emerg_rev", SPD_CRAWL);
        set_wf_recovery_dbg(robot, isLeft, "EMERG_REV", revServo, dFront, dSideMin, zL, zM, zR);
      } else {
        // Non-critical emergency: forward pivot-away, stronger if we just came from FBLK.
        const uint16_t lMin = min4_valid(zL);
        const uint16_t rMin = min4_valid(zR);
        int asymBoost = 0;
        if (is_valid_mm(lMin) && is_valid_mm(rMin)) {
          int diff = abs((int)lMin - (int)rMin);
          asymBoost = clampi(diff / 80, 0, 8);
        }
        const int baseMag = recentFblk ? 26 : 22;
        const int turnMag = clampi(baseMag + asymBoost, 20, 30);
        int escSign = 0;
        const bool carryFresh = (gWfEmergCarrySign != 0) && (nowMs <= gWfEmergCarryUntilMs);
        if (carryFresh) escSign = (int)gWfEmergCarrySign;
        else escSign = (openSign != 0) ? openSign : awaySign;
        int escServo = clampi(SERVO_STRAIGHT + escSign * turnMag, SERVO_LEFT, SERVO_RIGHT);
        float escSpd = (is_valid_mm(dFront) && dFront <= 130) ? 0.09f : SPD_CRAWL;
        movement_apply_angle(escServo, "bug2_wf_emerg_fwd", escSpd);
        set_wf_recovery_dbg(robot, isLeft, "EMERG_FWD", escServo, dFront, dSideMin, zL, zM, zR);
      }
      break;
    }

    if (gWfSideHardCooldownTicks > 0) gWfSideHardCooldownTicks--;

    if ((gWfSideSettleTicks > 0) && !is_valid_mm(dSideMin)) {
      // Do not spend a neutral settle tick while blind; let the normal WF mode
      // logic immediately enter OUTER_SEEK/reacquire on this same cycle.
      gWfSideSettleTicks = 0;
    }

    if ((gWfSideSettleTicks > 0) && (dSideMin > WF_SIDE_CRIT_MM)) {
      gWfSideSettleTicks--;
      int settleServo = SERVO_STRAIGHT;
      if (is_valid_mm(dSideMin) && dSideMin <= WF_SIDE_HARD_MM) {
        const int settleSideSign = isLeft ? +1 : -1;
        int settleAwayMag = 8;
        if (dSideMin <= 100) settleAwayMag = 12;
        else if (dSideMin <= 120) settleAwayMag = 10;
        if (is_valid_mm(dFront) && dFront <= 220) settleAwayMag = min(settleAwayMag, 8);
        settleServo = clampi(SERVO_STRAIGHT + settleSideSign * settleAwayMag,
                             SERVO_LEFT, SERVO_RIGHT);
      }
      movement_apply_angle(settleServo, "bug2_wf_side_settle", SPD_CRAWL);
      set_wf_recovery_dbg(robot, isLeft, "SIDE_SETTLE", settleServo, dFront, dSideMin, zL, zM, zR);
      break;
    }

    // Side hard-close override:
    // react immediately when followed side is too close, before mode logic.
    if (is_valid_mm(dSideMin) && dSideMin <= WF_SIDE_HARD_MM) {
      bool sideHardHandled = false;
      int sideSign = isLeft ? +1 : -1; // +1:right command, -1:left command
      if (dSideMin <= WF_SIDE_CRIT_MM) {
        gWfSideSettleTicks = 0;
        gWfSideHardCooldownTicks = 0;
        gWfRecoverTowardUntilMs = nowMs + WF_RECOVER_TOWARD_MS;
        int revServo = clampi(SERVO_STRAIGHT - sideSign * 22, SERVO_LEFT, SERVO_RIGHT);
        movement_apply_reverse(revServo, "bug2_wf_sidecrit", SPD_ESCAPE);
        set_wf_recovery_dbg(robot, isLeft, "SIDE_CRIT", revServo, dFront, dSideMin, zL, zM, zR);
        sideHardHandled = true;
      } else if (gWfSideHardCooldownTicks == 0) {
        int awayMag = 12;
        if (dSideMin <= 120) awayMag = 16;
        else if (dSideMin <= 140) awayMag = 14;
        if (is_valid_mm(dFront) && dFront <= 260) awayMag = min(awayMag, 12);
        if (is_valid_mm(dFront) && dFront <= 200) awayMag = min(awayMag, 10);
        int awayServo = clampi(SERVO_STRAIGHT + sideSign * awayMag, SERVO_LEFT, SERVO_RIGHT);
        gWfSideSettleTicks = WF_SIDE_SETTLE_TICKS;
        gWfSideHardCooldownTicks = WF_SIDE_HARD_COOLDOWN_TICKS;
        gWfRecoverTowardUntilMs = nowMs + 450;
        movement_apply_angle(awayServo, "bug2_wf_sidehard", SPD_CRAWL);
        set_wf_recovery_dbg(robot, isLeft, "SIDE_HARD", awayServo, dFront, dSideMin, zL, zM, zR);
        sideHardHandled = true;
      } else {
        // Non-critical hard-side correction was already pulsed recently.
        // Fall through to the softer TRACK/FRONT_BLOCK logic for one re-assess cycle.
      }
      if (sideHardHandled) break;
    }

    // Features from zone combinations.
    const uint8_t idxAll[4] = {0,1,2,3};
    const uint8_t idxCore[2] = {1,2}; // M1,M2
    const uint8_t idxSideL[2] = {2,3}; // M2,M3
    const uint8_t idxSideR[2] = {0,1}; // M0,M1
    const uint8_t* idxSide = isLeft ? idxSideL : idxSideR;

    float followConf = mean_conf_idx(zFollow, idxAll, 4, 220, 900);
    float oppConf    = mean_conf_idx(zAway,   idxAll, 4, 200, 700);
    float frontCoreConf = mean_conf_idx(zM, idxCore, 2, 220, 800);
    float frontSideConf = mean_conf_idx(zM, idxSide, 2, 220, 900);
    float frontRightConf = mean_conf_idx(zM, idxSideR, 2, 220, 900);
    float frontLeftConf  = mean_conf_idx(zM, idxSideL, 2, 220, 900);
    float frontAsymConf  = fabsf(frontLeftConf - frontRightConf);

    SideWallFit wallFit = estimate_side_wall_fit(zFollow, zM, isLeft);

    // Follow-side front/rear distance split (legacy fallback + debug visibility).
    uint16_t dSf = isLeft ? min_valid_idx(zFollow, 0, 1) : min_valid_idx(zFollow, 3, 2);
    uint16_t dSr = isLeft ? min_valid_idx(zFollow, 2, 3) : min_valid_idx(zFollow, 1, 0);
    const bool dSfValid = is_valid_mm(dSf);
    const bool dSrValid = is_valid_mm(dSr);
    const bool dSideValid = is_valid_mm(dSideMin);
    float dSfMm = dSfValid ? (float)dSf : (float)ZONE_WALL;
    float dSrMm = dSrValid ? (float)dSr : (float)ZONE_WALL;
    float dSideMm = dSideValid ? (float)dSideMin : (float)ZONE_WALL;

    // Angle inference from zone combinations.
    static const float kLFollowDeg[6] = {53.0f, 64.0f, 76.0f, 87.0f, 8.0f, 23.0f};
    static const float kRFollowDeg[6] = {-87.0f, -76.0f, -64.0f, -53.0f, -23.0f, -8.0f};
    static const float kMidDeg[4] = {-23.0f, -8.0f, 8.0f, 23.0f};
    uint16_t followVals[6];
    if (isLeft) {
      followVals[0]=zL[0]; followVals[1]=zL[1]; followVals[2]=zL[2]; followVals[3]=zL[3];
      followVals[4]=zM[2]; followVals[5]=zM[3];
    } else {
      followVals[0]=zR[0]; followVals[1]=zR[1]; followVals[2]=zR[2]; followVals[3]=zR[3];
      followVals[4]=zM[0]; followVals[5]=zM[1];
    }
    float followBear = weighted_bearing_deg(followVals, isLeft ? kLFollowDeg : kRFollowDeg, 6, 220, 1100);
    float frontBear  = weighted_bearing_deg(zM, kMidDeg, 4, 220, 1000);

    float targetBear = isLeft ? 72.0f : -72.0f;
    float eBear = 0.0f;
    if (!isnan(followBear)) {
      eBear = targetBear - followBear;
      while (eBear > 180.0f) eBear -= 360.0f;
      while (eBear < -180.0f) eBear += 360.0f;
    }

    // Distance + orientation control:
    // - estimate side distance/orientation from a weighted side-wall line fit
    // - blend with legacy front/rear difference so behavior degrades gracefully
    //   when side geometry is sparse/noisy.
    const bool wallFitValid = wallFit.valid;
    const float fitBlend = wallFitValid
      ? clampf(0.30f + 0.70f * wallFit.quality, 0.30f, 1.00f)
      : 0.0f;

    float sideCtrlMm = dSideMm;
    if (wallFitValid && dSideValid) {
      sideCtrlMm = (1.0f - fitBlend) * dSideMm + fitBlend * wallFit.distMm;
    } else if (wallFitValid) {
      sideCtrlMm = wallFit.distMm;
    }
    sideCtrlMm = clampf(sideCtrlMm, 60.0f, (float)ZONE_WALL);

    const bool sideCtrlValid = dSideValid || wallFitValid;
    float eDist = sideCtrlValid ? (gWfTargetMm - sideCtrlMm) : 0.0f; // + when too close
    float eYawLegacy = (dSrValid && dSfValid) ? (dSrMm - dSfMm) : 0.0f;
    float eTheta = wallFitValid ? (-wallFit.tangentDeg) : 0.0f; // + when side-front is closing in
    float eYawFit = clampf(eTheta * 10.0f, -250.0f, 250.0f); // map deg -> legacy mm-like scale
    float eYaw = wallFitValid
      ? ((1.0f - fitBlend) * eYawLegacy + fitBlend * eYawFit)
      : eYawLegacy;

    eDist = clampf(eDist, -350.0f, 350.0f);
    eYaw  = clampf(eYaw,  -250.0f, 250.0f);
    int sideSign = isLeft ? +1 : -1;      // + means right-turn command

    const uint16_t dFrontCoreMm = min_valid_idx(zM, 1, 2);
    const bool frontCoreRightValid = is_valid_mm(zM[1]);
    const bool frontCoreLeftValid  = is_valid_mm(zM[2]);
    const bool frontCoreAnyValid   = frontCoreRightValid || frontCoreLeftValid;
    const bool frontCoreBothValid  = frontCoreRightValid && frontCoreLeftValid;
    const bool frontNear = is_valid_mm(dFrontCoreMm) && dFrontCoreMm <= gZoneMid;
    const int frontOpenSign = front_open_turn_sign(zM);
    const uint16_t dFrontRightMm = front_half_effective_mm(zM[1], zM[0]);
    const uint16_t dFrontLeftMm  = front_half_effective_mm(zM[2], zM[3]);
    const uint16_t dFrontEdge = isLeft ? zM[3] : zM[0];
    const uint16_t dFrontCoreAdj = isLeft ? zM[2] : zM[1];
    const uint16_t dFollowMidMm = isLeft ? min_valid_idx(zM, 2, 3)
                                         : min_valid_idx(zM, 0, 1);
    const uint16_t dOppMidMm = isLeft ? min_valid_idx(zM, 0, 1)
                                      : min_valid_idx(zM, 2, 3);
    const uint16_t dFollowMidNearA = isLeft ? zM[2] : zM[0];
    const uint16_t dFollowMidNearB = isLeft ? zM[3] : zM[1];
    uint16_t dFollowMidMaxMm = 0;
    if (is_valid_mm(dFollowMidNearA)) dFollowMidMaxMm = dFollowMidNearA;
    if (is_valid_mm(dFollowMidNearB) && dFollowMidNearB > dFollowMidMaxMm) {
      dFollowMidMaxMm = dFollowMidNearB;
    }
    const uint8_t nFollowMid =
        (isLeft ? (is_valid_mm(zM[2]) ? 1 : 0) + (is_valid_mm(zM[3]) ? 1 : 0)
                : (is_valid_mm(zM[0]) ? 1 : 0) + (is_valid_mm(zM[1]) ? 1 : 0));
    const uint8_t nOppMid =
        (isLeft ? (is_valid_mm(zM[0]) ? 1 : 0) + (is_valid_mm(zM[1]) ? 1 : 0)
                : (is_valid_mm(zM[2]) ? 1 : 0) + (is_valid_mm(zM[3]) ? 1 : 0));
    const float frontEdgeClosingMms = isLeft ? gM3ClosingMms : gM0ClosingMms;
    const uint16_t dFollowFrontMm = isLeft ? min_valid_idx(zFollow, 0, 1)
                                           : min_valid_idx(zFollow, 2, 3);
    const uint16_t dFollowRearMm  = isLeft ? min_valid_idx(zFollow, 2, 3)
                                           : min_valid_idx(zFollow, 0, 1);
    const uint16_t dFollowNoseMm  = isLeft ? dFrontLeftMm : dFrontRightMm;
    // Local follow-pair equilibrium:
    // - RIGHT: R1/R2
    // - LEFT:  L1/L2
    // Positive error means the side-front of the robot is opening away from the
    // followed wall; negative means it is pointing into the wall.
    const uint16_t dFollowEqFrontMm = isLeft ? zFollow[1] : zFollow[2];
    const uint16_t dFollowEqRearMm  = isLeft ? zFollow[2] : zFollow[1];
    const bool followEqValid = is_valid_mm(dFollowEqFrontMm) && is_valid_mm(dFollowEqRearMm);
    const float followEqErrMm = followEqValid
      ? ((float)dFollowEqFrontMm - (float)dFollowEqRearMm)
      : 0.0f;
    const float followEqAbsMm = fabsf(followEqErrMm);
    const bool followEqAligned = followEqValid && (followEqAbsMm <= WF_EQ_ALIGN_MM);
    const bool followEqCapture = followEqValid && (followEqAbsMm <= WF_EQ_CAPTURE_MM);
    // The useful "already aligned" evidence can be the three zones nearest the
    // front edge: RIGHT follow R1/R2/R3, LEFT follow L0/L1/L2. This catches the
    // case where the rear-most zone drops out even though the side wall is clean.
    const uint16_t fTripA = isLeft ? zFollow[0] : zFollow[1];
    const uint16_t fTripB = isLeft ? zFollow[1] : zFollow[2];
    const uint16_t fTripC = isLeft ? zFollow[2] : zFollow[3];
    const bool followTripletValid =
        is_valid_mm(fTripA) && is_valid_mm(fTripB) && is_valid_mm(fTripC);
    const uint16_t followTripletMin = followTripletValid
        ? min(fTripA, min(fTripB, fTripC))
        : 0;
    const uint16_t followTripletMax = followTripletValid
        ? max(fTripA, max(fTripB, fTripC))
        : 0;
    const bool followTripletAligned =
        followTripletValid &&
        ((uint16_t)(followTripletMax - followTripletMin) <= WF_SIDE_TRIPLET_ALIGN_SPAN_MM) &&
        (followTripletMin >= WF_SIDE_TRIPLET_MIN_MM) &&
        (followTripletMax <= (uint16_t)(gWfTargetMm + 180.0f));
    // Front-most followed-side zone (symmetrical by side):
    // LEFT follow -> L0 is most front/inner, RIGHT follow -> R3 is most front/inner.
    const uint16_t dFollowLeadMm = isLeft ? zFollow[0] : zFollow[3];
    // "Blocked" = tighter side from the asymmetry sign, "Clear" = opposite side.
    const uint16_t dFrontBlockedMm = (frontOpenSign > 0) ? dFrontLeftMm : dFrontRightMm;
    const uint16_t dFrontClearMm   = (frontOpenSign > 0) ? dFrontRightMm : dFrontLeftMm;
    const bool frontAsymStrong = frontAsymConf >= 0.24f;
    const uint8_t frontNearCnt240 = count_wall_zones(zM, 240);
    const bool frontShapeWide = frontNearCnt240 >= 2;
    const bool frontAsymEnter = frontCoreBothValid && (frontOpenSign != 0) && frontAsymStrong &&
                                ((is_valid_mm(dFrontCoreMm) && (dFrontCoreMm <= gWfFblkExitFrontMm)) ||
                                 (is_valid_mm(dFrontBlockedMm) && (dFrontBlockedMm <= (uint16_t)(gWfFblkExitFrontMm + 140))));
    const bool frontAsymKeep = frontCoreBothValid && (frontOpenSign != 0) && (frontAsymConf >= 0.14f) &&
                               ((is_valid_mm(dFrontCoreMm) && (dFrontCoreMm <= (uint16_t)(gWfFblkExitFrontMm + 90))) ||
                                (is_valid_mm(dFrontBlockedMm) && (dFrontBlockedMm <= (uint16_t)(gWfFblkExitFrontMm + 180))));
    bool frontSoon = false;
    if (frontCoreAnyValid) {
      if (frontCoreBothValid) {
        frontSoon = is_valid_mm(dFrontCoreMm) &&
                    (dFrontCoreMm <= (uint16_t)(gWfFblkEnterFrontMm + 80));
      } else {
        // With only one core zone visible, be conservative.
        frontSoon = is_valid_mm(dFrontCoreMm) &&
                    (dFrontCoreMm <= (uint16_t)(gWfFblkEnterFrontMm + 40));
      }
    } else if (is_valid_mm(dFront)) {
      // Without core-front evidence, require both very near and spatially wide
      // evidence (>=2 front zones) to avoid side-wall edge false positives.
      frontSoon = (dFront <= 220) && frontShapeWide;
    }
    if (!frontSoon && frontCoreBothValid && (frontOpenSign != 0) && frontAsymStrong &&
        is_valid_mm(dFrontBlockedMm) &&
        (dFrontBlockedMm <= (uint16_t)(gWfFblkEnterFrontMm + 120))) {
      frontSoon = true;
    }
    const bool frontEdgeSoon =
        is_valid_mm(dFrontEdge) &&
        (dFrontEdge <= (uint16_t)(gWfFblkEnterFrontMm + 70)) &&
        ((!is_valid_mm(dFrontCoreAdj)) ||
         (dFrontCoreAdj > (uint16_t)(dFrontEdge + 120)) ||
         (frontSideConf > 0.24f) ||
         (frontEdgeClosingMms > 110.0f));
    if (!frontSoon && frontEdgeSoon) {
      frontSoon = true;
    }
    const bool frontFastKeep = frontCoreAnyValid
      ? (is_valid_mm(dFrontCoreMm) &&
         (dFrontCoreMm <= (uint16_t)(gWfFblkExitFrontMm + 80)) &&
         (gFrontClosingMms > 180.0f))
      : (is_valid_mm(dFront) &&
         (dFront <= 220) && frontShapeWide &&
         (gFrontClosingMms > 180.0f));
    const bool frontBlockedEnter = wf_front_block_enter(frontCoreConf, dFrontCoreMm) || frontAsymEnter || frontSoon;
    const bool frontBlockedKeep  = wf_front_block_keep(frontCoreConf, dFrontCoreMm) || frontFastKeep || frontAsymKeep;
    const bool innerCorner  = (!frontBlockedEnter) &&
                              is_valid_mm(dFrontCoreMm) &&
                              (dFrontCoreMm <= 340) &&
                              (followConf > 0.30f) &&
                              (frontSideConf > 0.65f);
    const bool outerCorner  = (wallLostCnt > 3) && (followConf < 0.15f) && (frontCoreConf < 0.25f);
    const uint16_t dOppMin = min4_valid(zAway);
    const bool oppValid = is_valid_mm(dOppMin);
    const bool followWeak = (nFollow <= 2) || (followConf < 0.12f);
    const bool followLost = !dSideValid && (followConf < 0.12f);
    const bool followLostStrong = !dSideValid || (nFollow == 0);
    const bool followNotCaptured = !dSideValid ||
                                   (dSideMin > (uint16_t)(gWfTargetMm + WF_REACQ_CAPTURE_BAND_MM));
    const bool oppSeen = oppValid && (dOppMin < SIDE_CHOOSE_MM);
    const bool frontRecoverRoom = !is_valid_mm(dFrontCoreMm) ||
                                  (dFrontCoreMm > (uint16_t)(gWfFblkEnterFrontMm + 20));
    const bool followLostRecover = gWfCapturedWall &&
                                   followWeak &&
                                   (oppSeen || (oppConf > 0.35f)) &&
                                   frontRecoverRoom;
    const bool frontCritical = (is_valid_mm(dFrontCoreMm) && (dFrontCoreMm <= 200)) ||
                               (is_valid_mm(dFront) && (dFront <= 185));
    const bool followTooNear = dSideValid && (dSideMin <= (uint16_t)(gWfTargetMm - 40.0f));
    const bool followFrontOpen = !is_valid_mm(dFollowFrontMm) ||
                                 (dFollowFrontMm > (uint16_t)(gWfTargetMm + 210.0f));
    const bool followFrontSeen = is_valid_mm(dFollowFrontMm) &&
                                 (dFollowFrontMm < (uint16_t)(gWfTargetMm + 320.0f));
    const bool followLeadOpen = !is_valid_mm(dFollowLeadMm) ||
                                (dFollowLeadMm > (uint16_t)(gWfTargetMm + 170.0f));
    const bool followSlopeOpen = is_valid_mm(dFollowFrontMm) && is_valid_mm(dFollowRearMm) &&
                                 (((int)dFollowFrontMm - (int)dFollowRearMm) >= 120);
    const bool followRearSeen = is_valid_mm(dFollowRearMm) &&
                                (dFollowRearMm < (uint16_t)(gWfTargetMm + 300.0f));
    const bool followGeomAligned = is_valid_mm(dFollowFrontMm) && is_valid_mm(dFollowRearMm) &&
                                   (abs((int)dFollowFrontMm - (int)dFollowRearMm) <= 110);
    const bool wallFitAligned = wallFitValid &&
                                (wallFit.quality >= 0.28f) &&
                                (fabsf(wallFit.tangentDeg) <= 14.0f);
    const bool followCapturedAligned = sideCtrlValid &&
                                       (sideCtrlMm <= (gWfTargetMm + 260.0f)) &&
                                       (followGeomAligned || wallFitAligned || followEqCapture ||
                                        followTripletAligned ||
                                        (followFrontSeen && followRearSeen && (nFollow >= 3)));
    const bool outerFrontRoom = (is_valid_mm(dFrontCoreMm) && (dFrontCoreMm > (uint16_t)(gWfFblkEnterFrontMm + 120))) ||
                                (!is_valid_mm(dFrontCoreMm) && (!is_valid_mm(dFront) || dFront > 360));
    if (!gWfCapturedWall && followCapturedAligned && dSideValid && (nFollow >= 3)) {
      gWfCapturedWall = true;
    }
    const bool freshEntryNoCapture = !gWfCapturedWall;
    const bool reacquireLostStrong = gWfCapturedWall && followLostStrong;
    const bool reacquireNotCaptured = gWfCapturedWall && followNotCaptured;
    const bool outerCapture = !frontCritical &&
                              (followFrontOpen || followSlopeOpen) &&
                              (followRearSeen || followWeak) &&
                              reacquireLostStrong &&
                              (sideCtrlMm > (gWfTargetMm + 40.0f)) &&
                              outerFrontRoom &&
                              (!is_valid_mm(dFrontCoreMm) ||
                               (dFrontCoreMm > (uint16_t)(gWfFblkEnterFrontMm - 20)));
    // Early outer-seek cue:
    // if follow-side front opens (e.g. R3/L0 clears) while rear still sees wall,
    // switch to seek immediately instead of waiting for deeper drift/front-block.
    const uint16_t frontOpenSeekMarginMm =
        (uint16_t)(gWfTargetMm + (followCapturedAligned
                                  ? (float)WF_FRONT_OPEN_SEEK_CAPTURE_MARGIN_MM
                                  : (float)WF_FRONT_OPEN_SEEK_MARGIN_MM));
    const bool frontOpeningPartialLoss =
        (nFollow <= 2) ||
        !is_valid_mm(dFollowFrontMm) ||
        !is_valid_mm(dFollowLeadMm);
    const bool frontOpeningSeekCue = !frontCritical &&
                                     (followFrontOpen || followLeadOpen) &&
                                     followRearSeen &&
                                     frontOpeningPartialLoss &&
                                     dSideValid &&
                                     (dSideMin > frontOpenSeekMarginMm) &&
                                     outerFrontRoom &&
                                     gWfCapturedWall;
    const bool frontOpeningHardCue = frontOpeningSeekCue &&
                                     (followLeadOpen || !is_valid_mm(dFollowFrontMm) ||
                                      (is_valid_mm(dFollowFrontMm) &&
                                       (dFollowFrontMm > (uint16_t)(gWfTargetMm + 260.0f))));
    const bool sidePeelRoom =
        (!is_valid_mm(dFrontCoreMm) || (dFrontCoreMm >= WF_PEELOFF_FRONT_CLEAR_MM)) &&
        (!is_valid_mm(dFront) || (dFront >= WF_PEELOFF_FRONT_CLEAR_MM));
    const bool sidePeelSeekCue =
        (gWfMode == WF_TRACK) &&
        gWfCapturedWall &&
        !frontCritical &&
        !followTooNear &&
        outerFrontRoom &&
        sidePeelRoom &&
        dSideValid &&
        (dSideMin >= (uint16_t)(gWfTargetMm + 25.0f)) &&
        followRearSeen &&
        (followFrontOpen || followLeadOpen || (nFollow <= 2));
    const bool frontMostlyOpen =
        (frontCoreConf < 0.22f) &&
        (frontSideConf < 0.22f) &&
        (!is_valid_mm(dFrontCoreMm) || (dFrontCoreMm > 650)) &&
        (!is_valid_mm(dFront) || (dFront > 600));
    const bool outerEdgeGripCue =
        gWfCapturedWall &&
        followCapturedAligned &&
        dSideValid &&
        (nFollow >= 3) &&
        followRearSeen &&
        followLeadOpen &&
        (dSideMin >= (uint16_t)fmaxf(0.0f, gWfTargetMm - 35.0f)) &&
        (dSideMin <= (uint16_t)(gWfTargetMm + 140.0f)) &&
        frontMostlyOpen &&
        !frontCritical;
    if (outerEdgeGripCue) {
      gWfOuterEdgeGripUntilMs = nowMs + WF_OUTER_EDGE_GRIP_MS;
      if (gWfSeekTowardUntilMs < gWfOuterEdgeGripUntilMs) {
        gWfSeekTowardUntilMs = gWfOuterEdgeGripUntilMs;
      }
    }
    const bool outerEdgeGripActive =
        (gWfOuterEdgeGripUntilMs != 0) &&
        (nowMs <= gWfOuterEdgeGripUntilMs) &&
        !frontCritical;
    const bool outerEdgeGripAllowed =
        outerEdgeGripActive &&
        (!dSideValid ||
         (dSideMin >= (uint16_t)(gWfTargetMm + WF_GRIP_SIDE_CLOSE_PAD_MM)));
    const bool outerSeekHold = (gWfMode == WF_OUTER_SEEK) &&
                               !frontCritical &&
                               !followTooNear &&
                               (reacquireLostStrong || reacquireNotCaptured || !followCapturedAligned);
    const bool recoverTowardActive = (gWfRecoverTowardUntilMs != 0) &&
                                     (nowMs <= gWfRecoverTowardUntilMs);
    const uint16_t recoverTrackSideMin =
        (uint16_t)fmaxf(0.0f, gWfTargetMm - 10.0f);
    const uint16_t recoverTrackSideMax =
        (uint16_t)(gWfTargetMm + (float)WF_RECOVER_TRACK_SIDE_BAND_MM);
    const bool recoverTrackReady =
        dSideValid &&
        (dSideMin >= recoverTrackSideMin) &&
        (dSideMin <= recoverTrackSideMax) &&
        (nFollow >= 3) &&
        (is_valid_mm(dFollowFrontMm) || followGeomAligned || wallFitAligned) &&
        (!is_valid_mm(dFront) || (dFront > WF_RECOVER_TRACK_FRONT_CLEAR_MM));
    const bool recoverTrackGuard = recoverTowardActive && !recoverTrackReady;
    const bool deadEndFrontGate =
        (is_valid_mm(dFrontCoreMm) && (dFrontCoreMm <= 170)) ||
        (is_valid_mm(dFront) && (dFront <= 180));
    const bool deadEndNarrowGate = (followConf > 0.45f) && (oppConf > 0.52f);
    const bool deadEndEnterRaw = wf_deadend_enter(frontCoreConf, oppConf, followConf) &&
                                 deadEndFrontGate && deadEndNarrowGate;
    const bool deadEndKeep  = wf_deadend_keep(frontCoreConf, oppConf) &&
                              (((is_valid_mm(dFrontCoreMm) && (dFrontCoreMm <= 230)) ||
                                (frontCoreConf > 0.55f)));
    if (gWfMode != WF_DEADEND) {
      if (deadEndEnterRaw) {
        if (gWfDendCandCnt < 255) gWfDendCandCnt++;
      } else {
        gWfDendCandCnt = 0;
      }
    } else {
      gWfDendCandCnt = 0;
    }
    const bool deadEndEnter = (gWfDendCandCnt >= WF_DEND_ENTER_COUNT);

    WfMode mode = gWfMode;
    WfMode desired = WF_TRACK;
    bool acqFrontSuppressed = false;
    bool acquireGuardActive = false;

    // Dead-end hysteresis has highest priority.
    if (gWfMode == WF_DEADEND) {
      if (deadEndKeep) desired = WF_DEADEND;
    } else if (deadEndEnter) {
      desired = WF_DEADEND;
    }

    // Front-block hysteresis is next.
    if (desired != WF_DEADEND) {
      const bool keepOuterRecover = (gWfMode == WF_OUTER_SEEK) &&
                                    (reacquireLostStrong || reacquireNotCaptured) &&
                                    (oppSeen || (oppConf > 0.28f)) &&
                                    (!is_valid_mm(dFrontCoreMm) ||
                                     (dFrontCoreMm > (uint16_t)(gWfFblkExitFrontMm - 40)));
      const bool preferSeekOverFblk = (reacquireLostStrong || reacquireNotCaptured) &&
                                      (oppSeen || (oppConf > 0.30f)) &&
                                      !frontCritical;
      const bool acquireContextNow =
          (gWfMode == WF_OUTER_SEEK) ||
          reacquireLostStrong ||
          reacquireNotCaptured ||
          followLostRecover ||
          outerSeekHold ||
          outerCapture ||
          outerEdgeGripAllowed ||
          frontOpeningSeekCue ||
          sidePeelSeekCue ||
          (gWfCapturedWall && (nFollow <= 2)) ||
          (gWfCapturedWall && !followCapturedAligned);
      if (acquireContextNow) {
        gWfAcquireGuardUntilMs = nowMs + WF_ACQ_FRONT_GUARD_MS;
      }
      acquireGuardActive = (gWfAcquireGuardUntilMs != 0) &&
                           (nowMs <= gWfAcquireGuardUntilMs);

      const bool acqM0 = is_valid_mm(M0) && (M0 <= WF_ACQ_FRONT_EDGE_MM);
      const bool acqM3 = is_valid_mm(M3) && (M3 <= WF_ACQ_FRONT_EDGE_MM);
      const bool acqM1 = is_valid_mm(M1) && (M1 <= WF_ACQ_FRONT_CORE_MM);
      const bool acqM2 = is_valid_mm(M2) && (M2 <= WF_ACQ_FRONT_CORE_MM);
      const uint8_t acqMidCloseCnt =
          (uint8_t)((acqM0 ? 1 : 0) + (acqM1 ? 1 : 0) +
                    (acqM2 ? 1 : 0) + (acqM3 ? 1 : 0));
      const bool acqCoreClose =
          acqM1 || acqM2 ||
          (is_valid_mm(dFrontCoreMm) && (dFrontCoreMm <= WF_ACQ_FRONT_CORE_MM));
      const bool acqFrontBroad =
          acqCoreClose ||
          frontCoreBothValid ||
          (frontCoreConf >= WF_ACQ_FRONT_BROAD_CONF) ||
          (acqMidCloseCnt >= 2);
      const bool acqFrontCritical =
          frontCritical ||
          (is_valid_mm(dFront) && (dFront <= WF_ACQ_FRONT_CRIT_MM)) ||
          (is_valid_mm(dFrontCoreMm) && (dFrontCoreMm <= WF_ACQ_FRONT_CRIT_MM));
      const bool acqFrontNeedsConfirm =
          acquireGuardActive &&
          frontBlockedEnter &&
          !acqFrontCritical &&
          !acqFrontBroad;
      if (!acqFrontNeedsConfirm) {
        gWfAcqFrontConfirmTicks = 0;
      } else if (freshSensFrame) {
          if (gWfAcqFrontConfirmTicks < 255) gWfAcqFrontConfirmTicks++;
      }
      const bool acqFrontConfirmed =
          !acqFrontNeedsConfirm ||
          (gWfAcqFrontConfirmTicks >= WF_ACQ_FRONT_CONFIRM_TICKS);
      acqFrontSuppressed = acqFrontNeedsConfirm && !acqFrontConfirmed;

      bool wantFrontBlock = (gWfMode == WF_FRONT_BLOCK) ? frontBlockedKeep : frontBlockedEnter;
      if ((gWfMode != WF_FRONT_BLOCK) && acqFrontSuppressed) wantFrontBlock = false;
      if ((outerSeekHold || keepOuterRecover || followLostRecover || preferSeekOverFblk ||
           outerCapture || frontOpeningSeekCue || sidePeelSeekCue ||
           (outerEdgeGripAllowed && ((nFollow <= 2) || !dSideValid))) && !followTooNear) {
        desired = WF_OUTER_SEEK;
      }
      else if (wantFrontBlock) desired = WF_FRONT_BLOCK;
      else if (recoverTrackGuard) {
        const bool recoverNeedFrontBlock =
            (is_valid_mm(dFront) && (dFront <= (uint16_t)(WF_RECOVER_TRACK_FRONT_CLEAR_MM + 20))) ||
            (dSideValid && (dSideMin < recoverTrackSideMin));
        desired = recoverNeedFrontBlock ? WF_FRONT_BLOCK : WF_OUTER_SEEK;
      }
      else if ((gWfMode == WF_OUTER_SEEK) && followTooNear) desired = WF_TRACK;
      else if (innerCorner) desired = WF_INNER_CORNER;
      else if (outerCorner && reacquireLostStrong) desired = WF_OUTER_SEEK;
      else desired = WF_TRACK;
    }

    // Minimum mode hold time to prevent rapid ping-pong.
    if (desired != gWfMode) {
      uint32_t modeAge = nowMs - gWfModeSinceMs;
      const bool forceModeChange = frontCritical || followTooNear ||
                                   ((desired == WF_OUTER_SEEK) &&
                                    (frontOpeningSeekCue || sidePeelSeekCue));
      if (forceModeChange || (modeAge >= (uint32_t)gWfModeDwellMs)) {
        const WfMode prevMode = gWfMode;
        if ((desired == WF_OUTER_SEEK) && (desired != gWfMode)) {
          wf_note_loss_anchor(robot, nowMs, isLeft, dFront, dSideMin, nFollow);
        }
        if ((desired == WF_OUTER_SEEK) && (desired != gWfMode) &&
            gWfCapturedWall && !frontCritical) {
          gWfSeekTowardUntilMs = nowMs + WF_SEEK_TOWARD_LATCH_MS;
        } else if (desired != WF_OUTER_SEEK) {
          gWfSeekTowardUntilMs = 0;
        }
        gWfMode = desired;
        gWfModeSinceMs = nowMs;
        if ((prevMode == WF_FRONT_BLOCK) && (desired == WF_TRACK)) {
          gWfPostFblkDampUntilMs = nowMs + WF_POST_FBLK_DAMP_MS;
        } else if (desired == WF_FRONT_BLOCK) {
          gWfPostFblkDampUntilMs = 0;
        }
        if (desired == WF_TRACK) log_decision("WF_MODE_TRK");
        else if (desired == WF_INNER_CORNER) log_decision("WF_MODE_IN");
        else if (desired == WF_OUTER_SEEK) log_decision("WF_MODE_OUT");
        else if (desired == WF_FRONT_BLOCK) log_decision("WF_MODE_FBLK");
        else if (desired == WF_DEADEND) log_decision("WF_MODE_DEND");
      }
    }
    mode = gWfMode;
    if (mode != WF_FRONT_BLOCK) gWfFblkSign = 0;
    if ((mode == WF_TRACK) && followCapturedAligned && (nFollow >= 3)) {
      wf_reset_loss_anchor();
    }
    float lossDistM = 0.0f;
    float lossYawDeg = 0.0f;
    float lossCumYawDeg = 0.0f;
    uint32_t lossAgeMs = 0;
    char lossBucket = '-';
    wf_update_loss_metrics(robot, nowMs, lossDistM, lossYawDeg,
                           lossCumYawDeg, lossAgeMs, lossBucket);

    int servo = SERVO_STRAIGHT;
    float spd = SPD_SLOW;
    const char* reason = "bug2_wf";
    float u = 0.0f;

    if (mode == WF_DEADEND) {
      if (!gWfDeadendActive) {
        gWfDeadendActive = true;
        gWfDeadendMs = nowMs;
      }
      uint32_t dt = nowMs - gWfDeadendMs;
      if (dt < gWfDeadendRevMs) {
        int revServo = clampi(SERVO_STRAIGHT - sideSign * 20, SERVO_LEFT, SERVO_RIGHT);
        movement_apply_reverse(revServo, "bug2_wf_dead_reverse", SPD_CRAWL);
        snprintf(gBug2Dbg, sizeof(gBug2Dbg),
                 "r(%.2f,%.2f) WF side=%c mode=%s servo=%d dFront=%u dSide=%u frontC=%.2f frontSideC=%.2f oppC=%.2f",
                 robot.x, robot.y, isLeft ? 'L' : 'R', wf_mode_name(mode), revServo,
                 (unsigned)dFront, (unsigned)dSideMin, frontCoreConf, frontSideConf, oppConf);
        break;
      } else if (dt < ((uint32_t)gWfDeadendRevMs + (uint32_t)gWfDeadendTurnMs)) {
        servo = clampi((int)roundf((float)SERVO_STRAIGHT + sideSign * 24.0f), SERVO_LEFT, SERVO_RIGHT);
        movement_apply_angle(servo, "bug2_wf_dead_turn", SPD_CRAWL);
        snprintf(gBug2Dbg, sizeof(gBug2Dbg),
                 "r(%.2f,%.2f) WF side=%c mode=%s servo=%d dFront=%u dSide=%u frontC=%.2f frontSideC=%.2f oppC=%.2f",
                 robot.x, robot.y, isLeft ? 'L' : 'R', wf_mode_name(mode), servo,
                 (unsigned)dFront, (unsigned)dSideMin, frontCoreConf, frontSideConf, oppConf);
        break;
      } else {
        gWfDeadendActive = false;
      }
    } else {
      gWfDeadendActive = false;
    }

    float uDist = sideSign * gWfDistK * clampf(eDist / 120.0f, -3.0f, 3.0f);
    float uYaw  = sideSign * gWfYawK  * clampf(eYaw  / 120.0f, -3.0f, 3.0f);
    float uBear = gWfBearK * clampf(eBear, -45.0f, 45.0f);
    float uOpp  = -sideSign * gWfOppK * clamp01((oppConf - 0.35f) / 0.65f);
    int frontTurnSign = (frontOpenSign != 0) ? frontOpenSign : sideSign;
    // During full-loss reacquire with no clear front asymmetry, bias front term
    // toward the chosen follow side (not away) to avoid drifting past the capture corridor.
    if ((frontOpenSign == 0) && reacquireLostStrong && !dSideValid) {
      frontTurnSign = -sideSign;
    }
    float uFront= frontTurnSign * gWfFrontK * clamp01((frontCoreConf - 0.20f) / 0.80f);
    float uOpenBias = 0.0f;
    float uBal = 0.0f;
    float uEq = 0.0f;
    float uSideFar = 0.0f;
    float uLost = 0.0f;
    float uSideOpen = 0.0f;
    float uHold = 0.0f;
    float uSideNear = 0.0f;
    float uFrontEdge = 0.0f;
    float uFrontTrend = 0.0f;
    float uHeadGeom = 0.0f;
    float uRecover = 0.0f;
    float uEntryAlign = 0.0f;
    float uFrontConv = 0.0f;
    float uInnerPrecaution = 0.0f;
    bool frontConvInner = false;
    const bool midSideStable =
        ((mode == WF_TRACK) || (mode == WF_FRONT_BLOCK)) &&
        dSideValid &&
        (nFollow >= 3) &&
        (dSideMin <= (uint16_t)(gWfTargetMm + 130.0f)) &&
        !frontCritical &&
        !reacquireLostStrong;
    const bool trackSideStable = midSideStable && (mode == WF_TRACK);
    const bool followMidOnly = (nFollowMid > 0) && (nOppMid == 0);
    const bool followMidSoftProjection =
        followMidOnly &&
        (nFollowMid >= 2) &&
        is_valid_mm(dFollowMidMm) &&
        is_valid_mm(dFollowMidMaxMm) &&
        ((float)dFollowMidMm >= (sideCtrlMm - (float)WF_FRONT_MID_PROJ_NEAR_MARGIN_MM)) &&
        ((float)dFollowMidMaxMm >= (sideCtrlMm + (float)WF_FRONT_MID_PROJ_SOFT_GAP_MM));
    const bool followMidProjectionOk =
        midSideStable &&
        followMidOnly &&
        is_valid_mm(dFollowMidMm) &&
        ((((float)dFollowMidMm >= (sideCtrlMm + (float)WF_FRONT_MID_PROJ_GAP_MM)) &&
          ((!is_valid_mm(dOppMidMm)) || (dOppMidMm > 850))) ||
         followMidSoftProjection);
    const uint8_t nMid900 = count_wall_zones(zM, 900);
    const bool broadMidWithSide =
        midSideStable &&
        ((nOppMid > 0 && is_valid_mm(dOppMidMm) && (dOppMidMm <= 900)) ||
         frontCoreBothValid ||
         (nMid900 >= 3));
    const bool innerPrecautionSideStable =
        ((mode == WF_TRACK) || (mode == WF_OUTER_SEEK) || (mode == WF_FRONT_BLOCK)) &&
        dSideValid &&
        (nFollow >= 3) &&
        (dSideMin <= (uint16_t)(gWfTargetMm + 170.0f)) &&
        !frontCritical &&
        !reacquireLostStrong;
    const bool broadMidPrecautionWithSide =
        innerPrecautionSideStable &&
        ((nOppMid > 0 && is_valid_mm(dOppMidMm) && (dOppMidMm <= 900)) ||
         frontCoreBothValid ||
         (nMid900 >= 3));
    const uint16_t innerFrontMinMm = (mode == WF_FRONT_BLOCK) ? 240 : 360;
    const bool innerCornerPrecaution =
        broadMidPrecautionWithSide &&
        !followMidProjectionOk &&
        is_valid_mm(dFrontCoreMm) &&
        (dFrontCoreMm >= innerFrontMinMm) &&
        (dFrontCoreMm <= 820) &&
        (nFollow >= 3);
    const bool innerPreCue =
        followMidProjectionOk &&
        (mode == WF_TRACK) &&
        innerPrecautionSideStable &&
        is_valid_mm(dFrontCoreMm) &&
        is_valid_mm(dFollowMidMm) &&
        (dFrontCoreMm <= 980) &&
        (dFollowMidMm <= 980) &&
        (dSideMin <= (uint16_t)(gWfTargetMm + 135.0f)) &&
        ((nFollowMid >= 2) ||
         (nMid900 >= 2) ||
         (is_valid_mm(dFollowMidMaxMm) && (dFollowMidMaxMm <= 1150)));
    const char* wfPrecautionName =
        acqFrontSuppressed ? "ACQ" :
        (sidePeelSeekCue ? "PEEL" :
        (outerEdgeGripAllowed ? "GRIP" :
        (innerCornerPrecaution ? "INNER" : (innerPreCue ? "PREIN" :
        (followMidProjectionOk ? "MIDPROJ" : "-")))));
    if (freshEntryNoCapture) {
      float frontNeed = 0.0f;
      if (is_valid_mm(dFrontCoreMm)) {
        frontNeed = close_conf(dFrontCoreMm, 260, 900);
      } else if (is_valid_mm(dFront)) {
        frontNeed = close_conf(dFront, 260, 900);
      }
      if (frontNeed > 0.0f) {
        const float sideMissing = !dSideValid
          ? 1.0f
          : clamp01(((float)dSideMin - (gWfTargetMm + 40.0f)) / 420.0f);
        const float weakFollow = clamp01((3.0f - (float)nFollow) / 3.0f);
        const float entryNeed = fmaxf(sideMissing, weakFollow);
        uEntryAlign = (float)sideSign * (8.0f + 10.0f * frontNeed) * (0.35f + 0.65f * entryNeed);
      }
    }
    {
      uint16_t dFrontConvMm = dFrontCoreMm;
      if (!is_valid_mm(dFrontConvMm) ||
          (is_valid_mm(dFront) && dFront < dFrontConvMm)) {
        dFrontConvMm = dFront;
      }
      if (!is_valid_mm(dFrontConvMm) ||
          (is_valid_mm(dFrontEdge) && dFrontEdge < dFrontConvMm)) {
        dFrontConvMm = dFrontEdge;
      }
      if (!is_valid_mm(dFrontConvMm) ||
          (is_valid_mm(dFrontBlockedMm) && dFrontBlockedMm < dFrontConvMm)) {
        dFrontConvMm = dFrontBlockedMm;
      }
      const bool edgeConvEvidence =
          is_valid_mm(dFrontEdge) &&
          (dFrontEdge == dFrontConvMm) &&
          ((!is_valid_mm(dFrontCoreAdj)) ||
           (dFrontCoreAdj > (uint16_t)(dFrontEdge + 100)) ||
           (frontEdgeClosingMms > 90.0f));
      const bool frontConvValid =
          is_valid_mm(dFrontConvMm) &&
          (frontCoreAnyValid || edgeConvEvidence || (frontSideConf > 0.24f) ||
           ((gFrontClosingMms > 100.0f) && (dFrontConvMm <= WF_FRONT_CONV_START_MM)));
      const bool followContext =
          gWfCapturedWall || dSideValid || (nFollow >= 2) || (followConf > 0.22f);
      frontConvInner =
          frontConvValid &&
          followContext &&
          !reacquireLostStrong &&
          (dFrontConvMm <= 520) &&
          ((frontSideConf > 0.34f) ||
           (is_valid_mm(dFollowNoseMm) &&
            (dFollowNoseMm <= (uint16_t)(gWfTargetMm + 260.0f))));

      if (frontConvValid && followContext) {
        float awayDeg = 0.0f;
        if (dFrontConvMm <= WF_FRONT_CONV_START_MM) awayDeg = 2.0f;
        if (dFrontConvMm <= 560) awayDeg = 4.0f;
        if (dFrontConvMm <= 480) awayDeg = 6.0f;
        if (dFrontConvMm <= WF_FRONT_CONV_STRONG_MM) awayDeg = 9.0f;
        if (dFrontConvMm <= 340) awayDeg = 11.0f;
        if (dFrontConvMm <= 280) awayDeg = 13.0f;
        if (dFrontConvMm <= 230) awayDeg = 15.0f;
        if (gFrontClosingMms > 120.0f && awayDeg > 0.0f) awayDeg += 2.0f;
        if (gFrontClosingMms > 240.0f && awayDeg > 0.0f) awayDeg += 2.0f;
        if (frontSideConf > 0.44f && awayDeg > 0.0f) awayDeg += 1.5f;
        if (edgeConvEvidence && awayDeg > 0.0f) awayDeg += 2.0f;
        if (frontConvInner && awayDeg > 0.0f) awayDeg += 1.5f;

        const float evidence = fmaxf(frontCoreConf, frontSideConf);
        if ((dFrontConvMm > WF_FRONT_CONV_STRONG_MM) &&
            (evidence < 0.22f) &&
            (gFrontClosingMms < 120.0f)) {
          awayDeg = 0.0f;
        }
        if (reacquireLostStrong &&
            (dFrontConvMm > WF_FRONT_CONV_STRONG_MM) &&
            (gFrontClosingMms < 180.0f)) {
          awayDeg = 0.0f;
        }
        if (trackSideStable) {
          float trackCap = WF_TRACK_INWARD_FRONT_CAP_DEG;
          if (followMidProjectionOk) {
            trackCap = WF_TRACK_PROJ_FRONT_CAP_DEG;
          } else if (broadMidWithSide) {
            trackCap = WF_TRACK_BROAD_FRONT_CAP_DEG;
          } else if (dFrontConvMm <= 280) {
            trackCap = WF_FRONT_CONV_MAX_DEG;
          }
          awayDeg = fminf(awayDeg, trackCap);
        }
        awayDeg = fminf(awayDeg, WF_FRONT_CONV_MAX_DEG);
        uFrontConv = (float)sideSign * awayDeg;
      }
    }
    const float frontConvAwayDeg = uFrontConv * (float)sideSign;
    if ((frontConvAwayDeg >= WF_ANTI_SWING_TRIGGER_DEG) &&
        (mode != WF_OUTER_SEEK)) {
      gWfAntiSwingUntilMs = nowMs + WF_ANTI_SWING_MS;
    }
    const bool antiSwingActive =
        (gWfAntiSwingUntilMs != 0) && (nowMs <= gWfAntiSwingUntilMs);
    if (frontOpenSign != 0 && frontAsymConf > 0.0f) {
      float asymMmStrength = 0.0f;
      if (is_valid_mm(dFrontBlockedMm) && is_valid_mm(dFrontClearMm)) {
        const float asymMm = (float)abs((int)dFrontClearMm - (int)dFrontBlockedMm);
        asymMmStrength = clamp01(asymMm / 450.0f);
      } else if (is_valid_mm(dFrontBlockedMm) && !is_valid_mm(dFrontClearMm)) {
        asymMmStrength = 1.0f;
      }
      const float openNearStrength = close_conf(dFrontBlockedMm, 220, 900);
      const float sideTight = dSideValid
        ? close_conf(dSideMin, (uint16_t)(gWfTargetMm - 20.0f), (uint16_t)(gWfTargetMm + 120.0f))
        : 0.0f;
      const float roomFactor = clampf(1.0f - 0.65f * sideTight, 0.35f, 1.0f);
      uOpenBias = frontOpenSign * (4.0f + 10.0f * asymMmStrength) * openNearStrength * roomFactor;
      if (is_valid_mm(dFrontCoreMm) && dFrontCoreMm < 360) {
        float nearCap = 8.0f;
        if (dFrontCoreMm < 300) nearCap = 7.0f;
        if (dFrontCoreMm < 250) nearCap = 6.0f;
        uOpenBias = clampf(uOpenBias, -nearCap, nearCap);
      }
    }
    // Corridor imbalance correction:
    // if opposite side is very close while followed side is far, bias turn toward followed side.
    if (dSideValid && oppValid) {
      const float followFar = clamp01((sideCtrlMm - (gWfTargetMm + 70.0f)) / 260.0f);
      const float oppNear = clamp01(((gWfTargetMm + 170.0f) - (float)dOppMin) / 220.0f);
      const float imbalance = followFar * oppNear;
      if (imbalance > 0.01f) {
        uBal = (-sideSign) * 9.0f * imbalance;
      }
    }
    // Local pair equilibrium reacts earlier than the full wall fit once the
    // side-facing pair becomes valid. Positive error => nose is opening away
    // from the followed wall; negative => nose is pointing into it.
    if (followEqValid) {
      float sideGate = sideCtrlValid
        ? clamp01(((gWfTargetMm + 420.0f) - sideCtrlMm) / 420.0f)
        : 0.35f;
      sideGate = fmaxf(sideGate, 0.35f);
      float frontRoom = is_valid_mm(dFront) ? clamp01(((float)dFront - 210.0f) / 420.0f) : 1.0f;
      const float towardFollowSign = (float)(-sideSign);
      uEq = towardFollowSign * WF_EQ_K *
            clampf(followEqErrMm / WF_EQ_ERR_SPAN_MM, -2.2f, 2.2f) *
            sideGate * (0.45f + 0.55f * frontRoom);
    }
    // If followed side is still far, keep pulling toward it in TRACK (even when opposite side
    // is not extremely near) so we do not drift and hand over too late to FRONT_BLOCK.
    if (sideCtrlValid) {
      float followFarOnly = clamp01((sideCtrlMm - (gWfTargetMm + 120.0f)) / 280.0f);
      float frontRoom = is_valid_mm(dFront) ? clamp01(((float)dFront - 240.0f) / 520.0f) : 1.0f;
      uSideFar = (-sideSign) * 7.0f * followFarOnly * frontRoom;
    }
    // Proactive outer-corner capture:
    // when follow side starts opening, turn into that side before front-block.
    if (sideCtrlValid) {
      // nF alone can hide a real opening (rear zones still seeing wall while front-side opens).
      float nWeak = clamp01((4.0f - (float)nFollow) / 3.0f); // nF: 4->0, 3->0.33, 2->0.67, <=1->1
      float sideOpen = clamp01((sideCtrlMm - (gWfTargetMm + 50.0f)) / 220.0f);
      float frontOpenCue = 0.0f;
      if (is_valid_mm(dFollowFrontMm)) {
        frontOpenCue = clamp01(((float)dFollowFrontMm - (gWfTargetMm + 80.0f)) / 260.0f);
      } else {
        frontOpenCue = 1.0f;
      }
      float rearHoldCue = is_valid_mm(dFollowRearMm)
        ? clamp01(((gWfTargetMm + 300.0f) - (float)dFollowRearMm) / 260.0f)
        : 0.0f;
      float frontRoom = is_valid_mm(dFront) ? clamp01(((float)dFront - 230.0f) / 560.0f) : 1.0f;
      float openingGeom = frontOpenCue * (0.4f + 0.6f * rearHoldCue);
      float openStrength = clampf(0.35f * nWeak + 0.30f * sideOpen + 0.35f * openingGeom, 0.0f, 1.0f) * frontRoom;
      uSideOpen = (-sideSign) * 10.0f * openStrength;
    }
    // Stronger recovery when followed side is lost and opposite wall is still seen.
    if (gWfCapturedWall && followWeak && (oppSeen || oppConf > 0.25f)) {
      float lostStrength = oppSeen ? clamp01((float)(SIDE_CHOOSE_MM - dOppMin) / (float)SIDE_CHOOSE_MM)
                                   : clamp01((oppConf - 0.25f) / 0.75f);
      if (dSideValid) lostStrength *= 0.45f;
      uLost = (-sideSign) * (7.0f + 9.0f * lostStrength);
    }
    // Baseline stickiness toward selected side while not too near.
    if (dSideValid) {
      float hold = clamp01(((float)dSideMin - (gWfTargetMm - 20.0f)) / 260.0f);
      uHold = (-sideSign) * 2.8f * hold;
    }
    // Soft near-side repulsion before SIDE_HARD kicks in; prevents scrape->reverse loops.
    if (dSideValid) {
      float nearSoft = close_conf(dSideMin, 190, (uint16_t)(gWfTargetMm + 120.0f));
      float nearHard = close_conf(dSideMin, 120, 220);
      uSideNear = sideSign * (6.0f * nearSoft + 8.0f * nearHard);
    }
    // Edge-front avoidance for side-wall grazing cases (M0/M3 near while adjacent core is open/missing).
    if (is_valid_mm(dFrontEdge)) {
      float edgeNear = close_conf(dFrontEdge, 120, 420);
      float isolated = 0.0f;
      if (!is_valid_mm(dFrontCoreAdj)) {
        isolated = edgeNear;
      } else {
        int coreDiff = (int)dFrontCoreAdj - (int)dFrontEdge;
        if (coreDiff > 80) {
          isolated = edgeNear * clamp01(((float)coreDiff - 80.0f) / 220.0f);
        }
      }
      const float edgeGain = !is_valid_mm(dFrontCoreAdj) ? 12.0f : 9.0f;
      uFrontEdge = sideSign * edgeGain * isolated;
      const float edgeApproach = close_conf(dFrontEdge, 260, 760);
      const float edgeFast = clamp01((frontEdgeClosingMms - 70.0f) / 220.0f);
      float edgeTrendGeom = 1.0f;
      if (is_valid_mm(dFrontCoreAdj)) {
        const float coreTight = close_conf(dFrontCoreAdj, 220, 620);
        edgeTrendGeom = 0.45f + 0.55f * coreTight;
      }
      uFrontTrend = sideSign * 8.5f * edgeApproach * edgeFast * edgeTrendGeom;
      if (followMidProjectionOk) {
        uFrontEdge = 0.0f;
        uFrontTrend = 0.0f;
      } else if (trackSideStable && is_valid_mm(dFrontEdge) && (dFrontEdge > 300)) {
        const float cap = broadMidWithSide ? WF_TRACK_BROAD_FRONT_CAP_DEG
                                           : WF_TRACK_INWARD_FRONT_CAP_DEG;
        uFrontEdge = clampf(uFrontEdge, -cap, cap);
        uFrontTrend = clampf(uFrontTrend, -cap, cap);
      }
    }
    // Geometry-consistency cue:
    // if the front-side zone (M0/M3) is too short for the currently followed-side
    // wall distance, we are likely yawed into that wall/corner rather than parallel.
    // Bias away from the followed wall early, symmetrically for left/right follow.
    if (dSideValid && is_valid_mm(dFrontEdge) && (nFollow >= 3) &&
        !frontCritical && !reacquireLostStrong) {
      const float headGapMm = (float)dFrontEdge - sideCtrlMm;
      const float expectedGapMm =
          clampf(180.0f + 0.35f * (sideCtrlMm - gWfTargetMm), 180.0f, 360.0f);
      const float shortfallMm = expectedGapMm - headGapMm;
      if (shortfallMm > 0.0f) {
        const float geomNeed = clamp01(shortfallMm / 220.0f);
        const float sideGate = clamp01((sideCtrlMm - (gWfTargetMm + 20.0f)) / 260.0f);
        const float frontGate = is_valid_mm(dFront)
            ? clamp01(((float)dFront - 220.0f) / 320.0f)
            : 1.0f;
        const float captureGate = followCapturedAligned ? 1.0f : 0.75f;
        uHeadGeom = sideSign * 8.0f * geomNeed *
                    (0.35f + 0.65f * sideGate) *
                    (0.45f + 0.55f * frontGate) * captureGate;
        if (followMidProjectionOk) {
          uHeadGeom = 0.0f;
        } else if (trackSideStable && is_valid_mm(dFrontEdge) && (dFrontEdge > 300)) {
          const float cap = broadMidWithSide ? WF_TRACK_BROAD_FRONT_CAP_DEG
                                             : WF_TRACK_INWARD_FRONT_CAP_DEG;
          uHeadGeom = clampf(uHeadGeom, -cap, cap);
        }
      }
    }
    if (innerCornerPrecaution) {
      const float frontPrec =
          clamp01((820.0f - (float)dFrontCoreMm) / 460.0f);
      const float midSpan =
          clamp01(((float)nMid900 - 2.0f) / 2.0f);
      const float sidePresent =
          clamp01(((gWfTargetMm + 260.0f) - sideCtrlMm) / 260.0f);
      float precDeg = 3.0f + 5.0f * frontPrec + 2.0f * midSpan;
      precDeg *= (0.55f + 0.45f * sidePresent);
      uInnerPrecaution = (float)sideSign *
                         clampf(precDeg, WF_INNER_PRECAUTION_MIN_DEG,
                                WF_INNER_PRECAUTION_MAX_DEG);
    }
    // As front closes, attenuate side-seeking terms to avoid over-commit/edge bumps.
    float frontSeekRoom = 1.0f;
    if (is_valid_mm(dFrontCoreMm)) {
      frontSeekRoom = clampf(((float)dFrontCoreMm - 220.0f) / 360.0f, 0.0f, 1.0f);
    } else if (is_valid_mm(dFront)) {
      frontSeekRoom = clampf(((float)dFront - 200.0f) / 360.0f, 0.0f, 1.0f);
    }
    uBal *= frontSeekRoom;
    uSideFar *= frontSeekRoom;
    uSideOpen *= frontSeekRoom;
    uLost *= frontSeekRoom;
    uHold *= (0.4f + 0.6f * frontSeekRoom);
    uEq *= (0.35f + 0.65f * frontSeekRoom);
    if (freshEntryNoCapture && !followCapturedAligned) {
      // First acquisition should not yaw toward a side just because it was
      // selected for future following. Let front avoidance establish the pass
      // direction; enable normal side pull only after the wall is captured.
      uDist = 0.0f;
      uYaw = 0.0f;
      uBal = 0.0f;
      uEq = 0.0f;
      uSideFar = 0.0f;
      uSideOpen = 0.0f;
      uHold = 0.0f;
    }

    const bool recoverTowardFresh = (gWfRecoverTowardUntilMs != 0) &&
                                    (nowMs <= gWfRecoverTowardUntilMs);
    const bool seekTowardFresh = (gWfSeekTowardUntilMs != 0) &&
                                 (nowMs <= gWfSeekTowardUntilMs);
    if (recoverTowardFresh && !frontCritical) {
      float room = is_valid_mm(dFront) ? clamp01(((float)dFront - 140.0f) / 260.0f) : 1.0f;
      if (room > 0.0f) {
        uRecover = (float)(-sideSign) * (4.0f + 6.0f * room);
      }
    }

    switch (mode) {
      case WF_TRACK:
      {
        // Bearing can dominate incorrectly in side corridors; keep it secondary.
        float uBearTrack = 0.30f * uBear;
        u = uDist + uYaw + uBearTrack + uOpp + 0.35f * uFront +
            uOpenBias + uBal + uEq + uSideFar + uLost + uSideOpen + uHold +
            0.55f * uRecover + uEntryAlign +
            uFrontConv + uInnerPrecaution + uSideNear + uFrontEdge + uFrontTrend +
            0.85f * uHeadGeom;
        // Guardrail: when followed side is not near, cap strong turns away from it.
        if (dSideValid && (dSideMin > (uint16_t)(gWfTargetMm - 20.0f)) && !frontCritical) {
          const float frontConvAway = uFrontConv * (float)sideSign;
          float awayCap = (frontConvAway > 0.0f)
              ? fmaxf(1.5f, fminf(frontConvAway, WF_FRONT_CONV_MAX_DEG))
              : 1.5f;
          if (innerCornerPrecaution) {
            awayCap = fmaxf(awayCap,
                            clampf(fabsf(uInnerPrecaution) + 1.0f, 7.0f, 10.5f));
          }
          float awayCmd = u * (float)sideSign; // >0 means away from followed side
          if (awayCmd > awayCap) u -= (awayCmd - awayCap) * (float)sideSign;
        }
        {
          uint16_t trackFrontMm = dFrontCoreMm;
          if (!is_valid_mm(trackFrontMm) ||
              (is_valid_mm(dFront) && dFront < trackFrontMm)) {
            trackFrontMm = dFront;
          }
          if (!is_valid_mm(trackFrontMm) ||
              (is_valid_mm(dFrontEdge) && dFrontEdge < trackFrontMm)) {
            trackFrontMm = dFrontEdge;
          }
          const bool precautionFloor = innerCornerPrecaution || followMidProjectionOk;
          const float minAway =
              wf_track_precaution_away_floor(trackFrontMm, dSideMin,
                                             precautionFloor, uInnerPrecaution);
          float awayCmd = u * (float)sideSign; // >0 means away from followed side
          if (awayCmd < minAway) u += (minAway - awayCmd) * (float)sideSign;
        }
        if (innerPreCue) {
          const float towardSign = (float)(-sideSign);
          float towardCmd = u * towardSign; // >0 means toward followed side
          if (towardCmd > WF_INNER_PRE_CUE_MAX_TOWARD_DEG) {
            u -= (towardCmd - WF_INNER_PRE_CUE_MAX_TOWARD_DEG) * towardSign;
          }
          const float preCueNeed = clamp01((980.0f - (float)dFollowMidMm) / 460.0f);
          const float minAway = WF_INNER_PRE_CUE_MIN_AWAY_DEG +
                                (WF_INNER_PRE_CUE_MAX_AWAY_DEG -
                                 WF_INNER_PRE_CUE_MIN_AWAY_DEG) * preCueNeed;
          float awayCmd = u * (float)sideSign; // >0 means away from followed side
          if (awayCmd < minAway) u += (minAway - awayCmd) * (float)sideSign;
        }
        const bool postFblkDamp = (gWfPostFblkDampUntilMs != 0) &&
                                  (nowMs <= gWfPostFblkDampUntilMs);
        if (postFblkDamp && !innerCornerPrecaution && !innerPreCue &&
            dSideValid && (nFollow >= 3) && !frontCritical) {
          const float awayCap = (is_valid_mm(dFront) && (dFront <= 310)) ? 5.0f : 3.5f;
          float awayCmd = u * (float)sideSign; // >0 means away from followed side
          if (awayCmd > awayCap) u -= (awayCmd - awayCap) * (float)sideSign;
        }
        // If followed side is fully lost and front is approaching, force a minimum
        // turn toward the chosen follow side to reacquire before front contact.
        if (reacquireLostStrong && is_valid_mm(dFront) && dFront <= 430) {
          const float towardFollowSign = (float)(-sideSign);
          float minToward = 10.0f;
          const bool fullLost = (nFollow == 0) && !dSideValid;
          if (fullLost) {
            minToward = (dFront > WF_REACQ_HARD_FRONT_MM)
              ? WF_REACQ_HARD_TOWARD_DEG
              : WF_REACQ_HARD_TOWARD_NEAR_DEG;
            // During full-loss reacquire do not allow turning away from followed side.
            float awayCmd = u * (float)sideSign; // >0 => away from followed side
            if (awayCmd > 0.0f) u -= awayCmd * (float)sideSign;
          }
          if ((u * towardFollowSign) < minToward) u = towardFollowSign * minToward;
        }
        // When follow-side front opens or peels away, keep a minimum
        // toward-follow turn in TRACK so we do not relax to straight before SEEK.
        if ((frontOpeningSeekCue || sidePeelSeekCue) && !innerPreCue &&
            is_valid_mm(dFront) && (dFront > 420)) {
          const float towardFollowSign = (float)(-sideSign);
          float minToward = (dFront > 650) ? 18.0f : 16.0f;
          if (frontOpeningHardCue || sidePeelSeekCue) minToward += 2.0f;
          float awayCmd = u * (float)sideSign;
          if (awayCmd > 0.0f) u -= awayCmd * (float)sideSign;
          if ((u * towardFollowSign) < minToward) u = towardFollowSign * minToward;
        }
        if (outerEdgeGripAllowed && !frontBlockedEnter && !frontCritical) {
          const float towardFollowSign = (float)(-sideSign);
          float minToward = WF_OUTER_EDGE_TRACK_TOWARD_DEG;
          if ((nFollow <= 2) || !dSideValid) minToward = WF_OUTER_EDGE_PARTIAL_TOWARD_DEG;
          if ((nFollow == 0) && !dSideValid) minToward = WF_OUTER_EDGE_FULL_TOWARD_DEG;
          float awayCmd = u * (float)sideSign;
          if (awayCmd > 0.0f) u -= awayCmd * (float)sideSign;
          if ((u * towardFollowSign) < minToward) u = towardFollowSign * minToward;
        }
        spd = (frontCoreConf > 0.35f) ? SPD_CRAWL : SPD_SLOW;
        if (is_valid_mm(dFront) && dFront <= 380) spd = fminf(spd, 0.11f);
        if (is_valid_mm(dFront) && dFront <= 300) spd = fminf(spd, 0.09f);
        if (gFrontClosingMms > 260.0f && is_valid_mm(dFront) && dFront <= 420) spd = fminf(spd, 0.09f);
        if (gWfCapturedWall && followWeak && (oppSeen || oppConf > 0.25f)) spd = fminf(spd, 0.11f);
        if (dSideValid && dSideMin <= 280) spd = fminf(spd, 0.10f);
        if (dSideValid && dSideMin <= 180) spd = fminf(spd, 0.09f);
        if (is_valid_mm(dFrontEdge) && !is_valid_mm(dFrontCoreAdj) && dFrontEdge <= 180) spd = fminf(spd, 0.09f);
        if (reacquireLostStrong && is_valid_mm(dFront) && dFront <= 430) spd = fminf(spd, 0.09f);
        if ((uFrontConv * (float)sideSign) >= 8.0f) spd = fminf(spd, 0.10f);
        {
          uint16_t trackFrontMm = dFrontCoreMm;
          if (!is_valid_mm(trackFrontMm) ||
              (is_valid_mm(dFront) && dFront < trackFrontMm)) {
            trackFrontMm = dFront;
          }
          if (!is_valid_mm(trackFrontMm) ||
              (is_valid_mm(dFrontEdge) && dFrontEdge < trackFrontMm)) {
            trackFrontMm = dFrontEdge;
          }
          if (is_valid_mm(trackFrontMm) && trackFrontMm <= 560) spd = fminf(spd, 0.10f);
          if (is_valid_mm(trackFrontMm) && trackFrontMm <= 420) spd = fminf(spd, 0.09f);
        }
        reason = "bug2_wf_track";
        break;
      }
      case WF_INNER_CORNER:
        u = sideSign * (10.0f + 14.0f * frontSideConf + 6.0f * frontCoreConf) + 0.4f * uBear;
        spd = SPD_CRAWL;
        reason = "bug2_wf_inner_corner";
        break;
      case WF_OUTER_SEEK:
      {
        const float lostGain = clamp01((float)wallLostCnt / 10.0f);
        float outerRoom = 1.0f;
        if (is_valid_mm(dFrontCoreMm)) {
          outerRoom = clampf(((float)dFrontCoreMm - 250.0f) / 420.0f, 0.20f, 1.00f);
        } else if (is_valid_mm(dFront)) {
          outerRoom = clampf(((float)dFront - 230.0f) / 420.0f, 0.20f, 1.00f);
        }
        float captureTaper = 1.0f;
        if (dSideValid) {
          const float minCaptureTaper = followCapturedAligned ? 0.25f : 0.65f;
          captureTaper = clampf(((float)dSideMin - (gWfTargetMm + 40.0f)) /
                                WF_REACQ_CAPTURE_BAND_MM, minCaptureTaper, 1.0f);
        }
        const float seekU = sideSign * (-10.0f - 8.0f * lostGain) * outerRoom * captureTaper;
        const float nearPush = dSideValid
          ? close_conf(dSideMin, 120, (uint16_t)(gWfTargetMm - 10.0f))
          : 0.0f;
        // If followed side is already near, blend in opposite turn to avoid
        // hugging/colliding while still trying to reacquire opening geometry.
        const float nearAvoidU = sideSign * (14.0f * nearPush);
        const float farBoost = sideCtrlValid
          ? clamp01((sideCtrlMm - (gWfTargetMm + 160.0f)) / 280.0f)
          : 0.0f;
        const float towardFarU = (-sideSign) * 5.0f * farBoost * outerRoom;
        u = seekU + nearAvoidU + towardFarU +
            0.25f * uBear + 0.55f * uBal + 0.90f * uEq + 0.55f * uSideOpen +
            0.35f * uLost + 0.25f * uOpenBias + 0.40f * uHold +
            0.45f * uRecover +
            (1.0f - outerRoom) * 0.55f * uFront +
            uFrontConv + uInnerPrecaution + 0.65f * uSideNear + 0.80f * uFrontEdge + 0.70f * uFrontTrend +
            0.75f * uHeadGeom;
        const bool seekOvershot =
            gWfLossAnchorValid &&
            ((fabsf(lossCumYawDeg) >= WF_SEEK_OVERSHOOT_YAW_DEG) ||
             (lossDistM >= WF_SEEK_OVERSHOOT_DIST_M));
        const bool frontBeforeFollowSide =
            is_valid_mm(dFront) &&
            (dFront <= 560) &&
            ((!dSideValid) ||
             (dSideMin > (uint16_t)(gWfTargetMm + 180.0f)) ||
             (nFollow <= 1));
        const bool seekSwingDamp =
            antiSwingActive || (seekOvershot && frontBeforeFollowSide);
        if (!followCapturedAligned && is_valid_mm(dFront) && (dFront > 240)) {
          const float awayCap = (frontOpeningHardCue || sidePeelSeekCue) ? 0.0f : 2.0f;
          float awayCmd = u * (float)sideSign; // >0 => away from followed side
          if (awayCmd > awayCap) u -= (awayCmd - awayCap) * (float)sideSign;
        }
        if (reacquireLostStrong && is_valid_mm(dFront) && dFront <= 430) {
          const float towardFollowSign = (float)(-sideSign);
          float minToward = 11.0f;
          const bool fullLost = (nFollow == 0) && !dSideValid;
          if (fullLost) {
            minToward = (dFront > WF_REACQ_HARD_FRONT_MM)
              ? WF_REACQ_HARD_TOWARD_DEG
              : WF_REACQ_HARD_TOWARD_NEAR_DEG;
            // During full-loss reacquire do not allow turning away from followed side.
            float awayCmd = u * (float)sideSign; // >0 => away from followed side
            if (awayCmd > 0.0f) u -= awayCmd * (float)sideSign;
          }
          if ((u * towardFollowSign) < minToward) u = towardFollowSign * minToward;
        }
        // If recovery just handed us into a full-loss gap after a corner, do not
        // fall back to the generic open-space seek arc. Keep a committed turn
        // toward the chosen wall until either front blocks again or side evidence
        // comes back.
        const bool recoverFullLostSeek =
            recoverTowardFresh &&
            (nFollow == 0) &&
            !dSideValid &&
            !frontCritical &&
            !frontBlockedEnter;
        if (recoverFullLostSeek) {
          const float towardFollowSign = (float)(-sideSign);
          float minToward = WF_RECOVER_FULLLOSS_TOWARD_DEG;
          if (is_valid_mm(dFront) && (dFront <= WF_REACQ_HARD_FRONT_MM)) {
            minToward = WF_RECOVER_FULLLOSS_TOWARD_NEAR_DEG;
          }
          if ((u * towardFollowSign) < minToward) u = towardFollowSign * minToward;
        }
        // After releasing from TRACK/FRONT_BLOCK into seek, keep a short-lived
        // toward-follow latch so full-loss reacquire does not decay immediately
        // into the generic shallow open-space arc.
        const bool seekTowardLatchActive =
            seekTowardFresh &&
            !frontCritical &&
            !frontBlockedEnter &&
            ((nFollow <= 2) || !dSideValid);
        if (seekTowardLatchActive) {
          const float towardFollowSign = (float)(-sideSign);
          const bool fullLost = (nFollow == 0) && !dSideValid;
          float minToward = fullLost ? WF_SEEK_TOWARD_FULL_DEG
                                     : WF_SEEK_TOWARD_PARTIAL_DEG;
          if (is_valid_mm(dFront) && (dFront <= WF_REACQ_HARD_FRONT_MM)) {
            minToward -= 2.0f;
          }
          if ((u * towardFollowSign) < minToward) u = towardFollowSign * minToward;
        }
        const bool outerAllClearHold =
            ((nowMs - gWfModeSinceMs) <= WF_OUTER_ALL_CLEAR_HOLD_MS) &&
            !frontCritical &&
            !frontBlockedEnter &&
            !dSideValid &&
            (nFollow == 0) &&
            (!is_valid_mm(dFront) || (dFront >= WF_PEELOFF_FRONT_ROOM_MM)) &&
            (!is_valid_mm(dFrontCoreMm) || (dFrontCoreMm >= WF_PEELOFF_FRONT_CLEAR_MM));
        if (outerAllClearHold) {
          const float towardFollowSign = (float)(-sideSign);
          if ((u * towardFollowSign) < WF_OUTER_ALL_CLEAR_MIN_DEG) {
            u = towardFollowSign * WF_OUTER_ALL_CLEAR_MIN_DEG;
          }
        }
        if (outerEdgeGripAllowed && !frontBlockedEnter && !frontCritical) {
          const float towardFollowSign = (float)(-sideSign);
          const bool fullLost = (nFollow == 0) && !dSideValid;
          float minToward = fullLost ? WF_OUTER_EDGE_FULL_TOWARD_DEG
                                     : WF_OUTER_EDGE_PARTIAL_TOWARD_DEG;
          if (dSideValid && (nFollow >= 3)) {
            minToward = fmaxf(WF_OUTER_EDGE_TRACK_TOWARD_DEG, minToward - 6.0f);
          }
          if ((u * towardFollowSign) < minToward) u = towardFollowSign * minToward;
        }
        // When side capture is weak/lost but front is still roomy, keep a minimum
        // toward-follow command so OUTER_SEEK reacquires earlier instead of drifting
        // too deep before turning in.
        const bool sideFarForSeek = !dSideValid ||
                                    (dSideMin > (uint16_t)(gWfTargetMm + 90.0f));
        const bool rearOnlyCapture = followRearSeen && !is_valid_mm(dFollowFrontMm);
        if ((((nFollow <= 3) || !dSideValid) && sideFarForSeek) || !followCapturedAligned) {
          if (is_valid_mm(dFront) && !frontCritical && !frontBlockedEnter) {
            const float towardFollowSign = (float)(-sideSign);
            float minToward = 17.0f;
            if (!followCapturedAligned) minToward = 19.0f;
            if (frontOpeningSeekCue) minToward = 21.0f;
            if (frontOpeningHardCue) minToward = 24.0f;
            if (sidePeelSeekCue) minToward = fmaxf(minToward, 22.0f);
            if (rearOnlyCapture) minToward = 22.0f;
            if (!dSideValid || (nFollow <= 1)) {
              minToward = (dFront > 700) ? 22.0f : 20.0f;
              if (frontOpeningHardCue || sidePeelSeekCue) minToward += 2.0f;
            } else if (nFollow == 2) {
              minToward = (dFront > 650) ? 20.0f : 18.0f;
              if (frontOpeningHardCue || sidePeelSeekCue) minToward += 2.0f;
            } else if (dFront > 650) {
              minToward = 18.0f;
              if (frontOpeningHardCue || sidePeelSeekCue) minToward += 2.0f;
            }
            // Do not let the toward-follow floor vanish as soon as front
            // distance slips below 300 mm; taper it down until front-block
            // is actually active so OUTER_SEEK stays committed enough to
            // reacquire the intended wall instead of drifting into a shallow,
            // opposite-side approach.
            float frontKeep = 1.0f;
            if (dFront <= 650) frontKeep = 0.90f;
            if (dFront <= 500) frontKeep = 0.75f;
            if (dFront <= 380) frontKeep = 0.60f;
            if (dFront <= 300) frontKeep = 0.45f;
            if (dFront <= 240) frontKeep = 0.30f;
            minToward *= frontKeep;
            if ((u * towardFollowSign) < minToward) u = towardFollowSign * minToward;
          }
        }
        if (gWfCapturedWall &&
            ((nFollow <= 3) || !dSideValid) && sideFarForSeek &&
            is_valid_mm(dFront) && (dFront > 430)) {
          const float towardFollowSign = (float)(-sideSign);
          float minToward = 15.0f;
          if (!dSideValid || (nFollow <= 1)) {
            minToward = (dFront > 700) ? 20.0f : 18.0f;
          } else if (nFollow == 2) {
            minToward = (dFront > 650) ? 18.0f : 16.0f;
          } else if (dFront > 650) {
            minToward = 17.0f;
          }
          if ((u * towardFollowSign) < minToward) u = towardFollowSign * minToward;
        }
        spd = (nearPush > 0.18f) ? 0.10f : SPD_CRAWL;
        if (dSideValid && dSideMin <= 260) spd = fminf(spd, 0.09f);
        if (!followCapturedAligned && is_valid_mm(dFront) && dFront <= 420) spd = fminf(spd, 0.09f);
        if (is_valid_mm(dFrontEdge) && !is_valid_mm(dFrontCoreAdj) && dFrontEdge <= 170) spd = fminf(spd, 0.09f);
        if (reacquireLostStrong && is_valid_mm(dFront) && dFront <= 430) spd = fminf(spd, 0.09f);
        if ((nFollow == 0) && !dSideValid) {
          spd = fminf(spd, 0.09f);
          if (is_valid_mm(dFront) && dFront <= 260) spd = fminf(spd, 0.08f);
        }
        if ((uFrontConv * (float)sideSign) >= 8.0f) spd = fminf(spd, 0.10f);
        {
          uint16_t seekFrontMm = dFrontCoreMm;
          if (!is_valid_mm(seekFrontMm) ||
              (is_valid_mm(dFront) && dFront < seekFrontMm)) {
            seekFrontMm = dFront;
          }
          if (!is_valid_mm(seekFrontMm) ||
              (is_valid_mm(dFrontEdge) && dFrontEdge < seekFrontMm)) {
            seekFrontMm = dFrontEdge;
          }
          if (!is_valid_mm(seekFrontMm) ||
              (is_valid_mm(dFrontBlockedMm) && dFrontBlockedMm < seekFrontMm)) {
            seekFrontMm = dFrontBlockedMm;
          }
          const bool seekFrontReacquire =
              is_valid_mm(seekFrontMm) &&
              (seekFrontMm <= 650) &&
              (dSideValid || (nFollow >= 2) || frontCoreAnyValid);
          if (seekFrontReacquire) {
            float minAway = wf_front_away_floor(seekFrontMm);
            if (dSideValid && (nFollow >= 3) &&
                (dSideMin <= (uint16_t)(gWfTargetMm + 380.0f))) {
              minAway += 2.0f;
            }
            minAway = fminf(minAway, 22.0f);
            const float awayCmd = u * (float)sideSign;
            if (awayCmd < minAway) {
              u += (minAway - awayCmd) * (float)sideSign;
            }
            spd = fminf(spd, (seekFrontMm <= 420) ? 0.09f : 0.10f);
          }
        }
        if (dSideValid && dSideMin <= 220) {
          const float minAway = (dSideMin <= 180) ? 9.0f : 7.0f;
          float awayCmd = u * (float)sideSign;
          if (awayCmd < minAway) u += (minAway - awayCmd) * (float)sideSign;
        }
        if (innerCornerPrecaution) {
          const float minAway = fminf(WF_INNER_PRECAUTION_MAX_DEG,
                                      fmaxf(WF_INNER_PRECAUTION_MIN_DEG,
                                            fabsf(uInnerPrecaution) * WF_INNER_TRACK_FLOOR_GAIN));
          float awayCmd = u * (float)sideSign;
          if (awayCmd < minAway) u += (minAway - awayCmd) * (float)sideSign;
        }
        {
          const bool frontConvOverride =
              (frontConvAwayDeg >= 7.0f) &&
              (!reacquireLostStrong || frontConvInner || (frontConvAwayDeg >= 11.0f));
          if (frontConvOverride) {
            const float minAway = fminf(frontConvAwayDeg, WF_FRONT_CONV_MAX_DEG);
            float awayCmd = u * (float)sideSign; // >0 means away from followed side
            if (awayCmd < minAway) {
              u += (minAway - awayCmd) * (float)sideSign;
            }
          }
        }
        if (seekSwingDamp) {
          const float towardFollowSign = (float)(-sideSign);
          float towardCap = WF_ANTI_SWING_SEEK_CAP_DEG;
          if (frontBeforeFollowSide) towardCap = 9.0f;
          if (seekOvershot && frontBeforeFollowSide) towardCap = 7.0f;
          const float towardCmd = u * towardFollowSign;
          if (towardCmd > towardCap) {
            u -= (towardCmd - towardCap) * towardFollowSign;
          }
          spd = fminf(spd, 0.09f);
        }
        reason = "bug2_wf_outer_seek";
        break;
      }
      case WF_FRONT_BLOCK:
      {
        int fblkSign = sideSign;
        const int openSign = frontOpenSign;
        const int clearanceSign = away_sign_from_clearance(zL, zR, gSide);
        if (openSign != 0) {
          bool safeToUse = true;
          if (!isLeft && openSign > 0 && is_valid_mm(dSideMin) &&
              dSideMin < (uint16_t)(gWfTargetMm + 80.0f)) safeToUse = false;
          if ( isLeft && openSign < 0 && is_valid_mm(dSideMin) &&
              dSideMin < (uint16_t)(gWfTargetMm + 80.0f)) safeToUse = false;
          if (safeToUse) fblkSign = openSign;
        } else {
          // If front asymmetry is weak, prefer turning away from the tighter side,
          // but do not steer into the followed wall when it's already close.
          const bool followedSideTight =
              is_valid_mm(dSideMin) && (dSideMin < (uint16_t)(gWfTargetMm + 40.0f));
          if (!followedSideTight) fblkSign = clearanceSign;
        }
        const bool followWeakNow = followWeak;
        const bool frontCriticalNow = frontCritical;
        if (reacquireLostStrong) {
          // When followed side is fully lost, force turn toward that side to reacquire.
          fblkSign = -sideSign;
        } else if (innerCornerPrecaution && !frontCriticalNow) {
          // Inner-corner precursor means the followed wall is still present while
          // MID is filling in. Keep the turn away from that wall and ramp it.
          fblkSign = sideSign;
        } else if (gWfCapturedWall && followWeakNow && !frontCriticalNow) {
          // Reacquire chosen side before committing to a hard front-block turn.
          fblkSign = -sideSign;
        }
        // Keep sign sticky unless a strong opposite-side opening is observed.
        // During follow-loss, trust reacquire sign immediately.
        if (reacquireLostStrong) {
          gWfFblkSign = (int8_t)fblkSign;
        } else if (gWfFblkSign == 0) {
          gWfFblkSign = (int8_t)fblkSign;
        } else if (fblkSign != (int)gWfFblkSign) {
          bool allowFlip = false;
          if (frontCoreBothValid && is_valid_mm(dFrontBlockedMm) && is_valid_mm(dFrontClearMm)) {
            allowFlip = ((int)dFrontBlockedMm + 220 < (int)dFrontClearMm);
          }
          if (!allowFlip) fblkSign = (int)gWfFblkSign;
          else gWfFblkSign = (int8_t)fblkSign;
        } else {
          gWfFblkSign = (int8_t)fblkSign;
        }

        // Geometry-scaled gain:
        // moderate at ~350-450mm, aggressive only when truly close.
        float blockedNear = 0.0f;
        if (is_valid_mm(dFrontBlockedMm)) blockedNear = close_conf(dFrontBlockedMm, 220, 650);
        else blockedNear = close_conf(dFront, 220, 650);
        float fblkGain = 6.0f + 5.0f * blockedNear + 6.0f * frontCoreConf + 4.0f * frontAsymConf;
        if (!is_valid_mm(dFrontCoreMm) || dFrontCoreMm > 340) {
          fblkGain = fminf(fblkGain, 11.0f);
        }
        if (is_valid_mm(dFrontCoreMm) && dFrontCoreMm <= 260) fblkGain += 6.0f;
        if (is_valid_mm(dFrontCoreMm) && dFrontCoreMm <= 200) fblkGain += 6.0f;
        if (followWeakNow && !frontCriticalNow) fblkGain = fminf(fblkGain, 12.0f);
        if (reacquireLostStrong) fblkGain = fminf(fblkGain, 11.0f);
        if (reacquireNotCaptured) fblkGain = fminf(fblkGain, 10.5f);
        if (innerCornerPrecaution && !frontCriticalNow) {
          const float innerGainCap =
              wf_inner_fblk_cap(dFrontCoreMm, dSideMin, uInnerPrecaution);
          fblkGain = fminf(fblkGain, innerGainCap);
        }
        fblkGain = clampf(fblkGain, 7.0f, 30.0f);
        u = fblkSign * fblkGain + 0.2f * uOpp + 0.35f * uEq +
            0.35f * uRecover + 0.45f * uSideNear + 0.35f * uFrontEdge +
            0.35f * uFrontConv + 0.65f * uInnerPrecaution;
        if (followMidProjectionOk && !frontCriticalNow) {
          float projCap = wf_fblk_proj_cap(dFrontCoreMm, dSideMin);
          if (is_valid_mm(dFront)) {
            projCap = fmaxf(projCap, wf_fblk_proj_cap(dFront, dSideMin));
          }
          const float awayCmd = u * (float)sideSign; // >0 means away from followed side
          if (awayCmd > projCap) {
            u -= (awayCmd - projCap) * (float)sideSign;
          }
        }
        // While the followed wall is still confidently present, taper the hard
        // front-block turn from the live side-pair and follow-side nose errors.
        // This keeps the pre-corner alignment responsive without trying to
        // "finish alignment" once the wall evidence is actually gone.
        const bool taperWallPresent =
            gWfCapturedWall &&
            !reacquireLostStrong &&
            !followWeakNow &&
            dSideValid &&
            followEqValid &&
            is_valid_mm(dFollowNoseMm) &&
            (nFollow >= 3);
        if (taperWallPresent) {
          const float noseTargetMm = clampf(WF_FBLK_NOSE_TARGET_BASE_MM +
                                            WF_FBLK_NOSE_TARGET_SIDE_GAIN *
                                            (sideCtrlMm - gWfTargetMm),
                                            WF_FBLK_NOSE_TARGET_MIN_MM,
                                            WF_FBLK_NOSE_TARGET_MAX_MM);
          const float noseNeed = clamp01((noseTargetMm - (float)dFollowNoseMm) /
                                         WF_FBLK_NOSE_NEED_SPAN_MM);
          const float pairIntoNeed = clamp01(((-followEqErrMm) - (float)WF_ALIGN_PAIR_SETTLE_MM) /
                                             WF_FBLK_PAIR_NEED_SPAN_MM);
          const float pairOpenRelief = clamp01((followEqErrMm - (float)WF_ALIGN_PAIR_SETTLE_MM) /
                                               WF_FBLK_PAIR_NEED_SPAN_MM);
          const float frontNeed = is_valid_mm(dFrontCoreMm)
              ? close_conf(dFrontCoreMm, 220, 360)
              : (is_valid_mm(dFront) ? close_conf(dFront, 220, 360) : 0.0f);
          const float sideSafeRelief =
              clamp01((sideCtrlMm - (gWfTargetMm + 70.0f)) / 220.0f);
          const float alignRelief = clamp01(0.55f * pairOpenRelief +
                                            0.45f * sideSafeRelief);
          float desiredAway = WF_FBLK_AWAY_MIN_DEG +
                              8.0f * noseNeed +
                              7.0f * pairIntoNeed +
                              3.0f * frontNeed -
                              7.0f * pairOpenRelief -
                              6.0f * sideSafeRelief -
                              4.0f * frontNeed * sideSafeRelief;
          desiredAway = clampf(desiredAway, 2.0f, WF_FBLK_AWAY_MAX_DEG);
          if (alignRelief > 0.20f) {
            const float alignCap = clampf(12.0f - 5.0f * frontNeed -
                                          4.0f * alignRelief,
                                          5.0f, 12.0f);
            desiredAway = fminf(desiredAway, alignCap);
          }
          const float awayCmd = u * (float)sideSign; // >0 means away from followed side
          if (awayCmd > desiredAway) {
            u -= (awayCmd - desiredAway) * (float)sideSign;
          }
        }
        uint16_t frontAvoidMm = dFrontCoreMm;
        if (!is_valid_mm(frontAvoidMm) ||
            (is_valid_mm(dFront) && dFront < frontAvoidMm)) {
          frontAvoidMm = dFront;
        }
        if (!is_valid_mm(frontAvoidMm) ||
            (is_valid_mm(dFrontEdge) && dFrontEdge < frontAvoidMm)) {
          frontAvoidMm = dFrontEdge;
        }
        if (!is_valid_mm(frontAvoidMm) ||
            (is_valid_mm(dFrontBlockedMm) && dFrontBlockedMm < frontAvoidMm)) {
          frontAvoidMm = dFrontBlockedMm;
        }
        if (!reacquireLostStrong && is_valid_mm(frontAvoidMm)) {
          float minAway = wf_front_away_floor(frontAvoidMm);
          if (gFrontClosingMms > 180.0f && minAway > 0.0f) minAway += 2.0f;
          if (gFrontClosingMms > 280.0f && minAway > 0.0f) minAway += 2.0f;
          const float frontConvAway = uFrontConv * (float)sideSign;
          if (frontConvAway > minAway) minAway = frontConvAway;
          if (frontConvInner && minAway > 0.0f) minAway += 1.5f;
          const bool fblkBroadFront =
              frontCoreBothValid || (frontCoreConf >= 0.55f) || (nMid900 >= 3);
          if (fblkBroadFront) {
            if (frontAvoidMm <= WF_FBLK_BROAD_EARLY_MM) {
              minAway = fmaxf(minAway, WF_FBLK_BROAD_EARLY_AWAY_DEG);
            }
            if (frontAvoidMm <= WF_FBLK_BROAD_STRONG_MM) {
              minAway = fmaxf(minAway, WF_FBLK_BROAD_STRONG_AWAY_DEG);
            }
            if (frontAvoidMm <= WF_FBLK_BROAD_MAX_MM) {
              minAway = fmaxf(minAway, WF_FBLK_BROAD_MAX_AWAY_DEG);
            }
          }
          if (followMidProjectionOk && (frontAvoidMm > 230)) {
            minAway = fminf(minAway, wf_fblk_proj_cap(frontAvoidMm, dSideMin));
          }
          if (innerCornerPrecaution && !frontCriticalNow && (frontAvoidMm > 210)) {
            minAway = fminf(minAway,
                            wf_inner_fblk_cap(frontAvoidMm, dSideMin,
                                              uInnerPrecaution));
          }
          minAway = fminf(minAway, WF_FBLK_AWAY_MAX_DEG);

          const float awayCmd = u * (float)sideSign; // >0 means away from followed side
          if (awayCmd < minAway) {
            u += (minAway - awayCmd) * (float)sideSign;
          }
          if (innerCornerPrecaution && !frontCriticalNow && (frontAvoidMm > 210)) {
            const float innerCap =
                wf_inner_fblk_cap(frontAvoidMm, dSideMin, uInnerPrecaution);
            const float cappedAway = u * (float)sideSign;
            if (cappedAway > innerCap) {
              u -= (cappedAway - innerCap) * (float)sideSign;
            }
          }
        }
        const bool fblkSideAlreadyAligned =
            !reacquireLostStrong &&
            !frontCriticalNow &&
            dSideValid &&
            (((nFollow >= 4) && followEqValid &&
              (followEqAbsMm <= WF_EQ_CAPTURE_MM)) ||
             followTripletAligned) &&
            (dSideMin <= (uint16_t)(gWfTargetMm + 120.0f)) &&
            is_valid_mm(frontAvoidMm) &&
            (frontAvoidMm > 420) &&
            !innerCornerPrecaution &&
            !followMidProjectionOk;
        if (fblkSideAlreadyAligned) {
          float alignedAwayCap = followTripletAligned
              ? WF_FBLK_TRIPLET_AWAY_CAP_DEG
              : ((frontAvoidMm > 320) ? 6.0f : 7.0f);
          if (dSideMin <= (uint16_t)(gWfTargetMm + 10.0f)) {
            alignedAwayCap = fminf(alignedAwayCap, 6.0f);
          }
          const float awayCmd = u * (float)sideSign; // >0 means away from followed side
          if (awayCmd > alignedAwayCap) {
            u -= (awayCmd - alignedAwayCap) * (float)sideSign;
          }
        }
        spd = SPD_CRAWL;
        if (is_valid_mm(frontAvoidMm) && frontAvoidMm <= 520) spd = fminf(spd, 0.10f);
        if (is_valid_mm(frontAvoidMm) && frontAvoidMm <= 340) spd = fminf(spd, 0.09f);
        if (is_valid_mm(frontAvoidMm) && frontAvoidMm <= 210) spd = fminf(spd, 0.08f);
        if (dSideValid && dSideMin <= 220) spd = fminf(spd, 0.09f);
        reason = "bug2_wf_front_block";
        break;
      }
      case WF_DEADEND:
      default:
        // handled above
        break;
    }

    int servoRaw = clampi((int)roundf((float)SERVO_STRAIGHT + u), SERVO_LEFT, SERVO_RIGHT);
    if (acqFrontSuppressed) {
      // During reacquire, a weak edge-only mid hit is often a transient; keep
      // tight side acquire, but do not let that blip command a hard away turn.
      // If the front is already real and close, allow enough away authority
      // to prevent an avoidable emergency.
      uint16_t acqFrontMm = dFront;
      if (is_valid_mm(dFrontCoreMm) &&
          (!is_valid_mm(acqFrontMm) || dFrontCoreMm < acqFrontMm)) {
        acqFrontMm = dFrontCoreMm;
      }
      if (is_valid_mm(dFrontEdge) &&
          (!is_valid_mm(acqFrontMm) || dFrontEdge < acqFrontMm)) {
        acqFrontMm = dFrontEdge;
      }
      int acqAwayCap = WF_ACQ_SUPPRESS_AWAY_CAP_DEG;
      if (is_valid_mm(acqFrontMm) && acqFrontMm <= 560) {
        acqAwayCap = WF_ACQ_REAL_FRONT_AWAY_CAP_DEG;
        if (acqFrontMm <= 380) acqAwayCap = 20;
        else if (acqFrontMm <= 460) acqAwayCap = 18;
      }
      const int awayCmd = (servoRaw - SERVO_STRAIGHT) * sideSign;
      if (awayCmd > acqAwayCap) {
        servoRaw = clampi(SERVO_STRAIGHT + sideSign * acqAwayCap,
                          SERVO_LEFT, SERVO_RIGHT);
      }
    }
    if ((mode == WF_OUTER_SEEK) && followCapturedAligned && dSideValid && (nFollow >= 3)) {
      const int towardSign = -sideSign; // toward followed side
      const int awaySign = sideSign;    // away from followed side
      int towardCmd = (servoRaw - SERVO_STRAIGHT) * towardSign;
      int awayCmd = (servoRaw - SERVO_STRAIGHT) * awaySign;
      if (towardCmd > 0) {
        int towardCap = 22;
        if (dSideMin <= 520) towardCap = 20;
        if (dSideMin <= 460) towardCap = 18;
        if (dSideMin <= 400) towardCap = 16;
        if (is_valid_mm(dFrontEdge)) {
          if (dFrontEdge <= 700) towardCap = min(towardCap, 16);
          if (dFrontEdge <= 520) towardCap = min(towardCap, 12);
          if (dFrontEdge <= 380) towardCap = min(towardCap, 8);
        }
        if (is_valid_mm(dFrontCoreMm)) {
          if (dFrontCoreMm <= 650) towardCap = min(towardCap, 16);
          if (dFrontCoreMm <= 500) towardCap = min(towardCap, 12);
          if (dFrontCoreMm <= 360) towardCap = min(towardCap, 8);
        }
        if (towardCmd > towardCap) {
          servoRaw = clampi(SERVO_STRAIGHT + towardSign * towardCap, SERVO_LEFT, SERVO_RIGHT);
          towardCmd = towardCap;
          awayCmd = (servoRaw - SERVO_STRAIGHT) * awaySign;
        }
      }
      if (followEqValid && (dSideMin <= 320) && (followEqErrMm <= -(float)WF_EQ_ALIGN_MM)) {
        const int minAway = (dSideMin <= 240) ? 8 : 6;
        if (awayCmd < minAway) {
          servoRaw = clampi(SERVO_STRAIGHT + awaySign * minAway, SERVO_LEFT, SERVO_RIGHT);
        }
      }
    }
    if ((mode == WF_TRACK) &&
        !outerEdgeGripAllowed &&
        !sidePeelSeekCue &&
        followCapturedAligned &&
        (followGeomAligned || wallFitAligned || followEqAligned) &&
        dSideValid) {
      const int towardSign = -sideSign; // toward followed side
      int towardCmd = (servoRaw - SERVO_STRAIGHT) * towardSign;
      int towardCap = 99;
      if (dSideMin <= 560) towardCap = 14;
      if (dSideMin <= 500) towardCap = 12;
      if (dSideMin <= 440) towardCap = 10;
      if (dSideMin <= 360) towardCap = 8;
      if (dSideMin <= 300) towardCap = 6;
      if (is_valid_mm(dFrontEdge)) {
        if (dFrontEdge <= 700) towardCap = min(towardCap, 10);
        if (dFrontEdge <= 520) towardCap = min(towardCap, 6);
        if (dFrontEdge <= 380) towardCap = min(towardCap, 4);
      }
      if (is_valid_mm(dFrontCoreMm)) {
        if (dFrontCoreMm <= 650) towardCap = min(towardCap, 10);
        if (dFrontCoreMm <= 500) towardCap = min(towardCap, 6);
        if (dFrontCoreMm <= 360) towardCap = min(towardCap, 4);
      }
      if (towardCmd > towardCap) {
        servoRaw = clampi(SERVO_STRAIGHT + towardSign * towardCap, SERVO_LEFT, SERVO_RIGHT);
      }
    }
    const uint32_t modeAgeMs = nowMs - gWfModeSinceMs;
    if ((mode == WF_TRACK) &&
        !outerEdgeGripAllowed &&
        !sidePeelSeekCue &&
        (modeAgeMs < 220) &&
        followCapturedAligned &&
        dSideValid &&
        (dSideMin > (uint16_t)(gWfTargetMm + 40.0f)) &&
        (((is_valid_mm(dFrontCoreMm) && (dFrontCoreMm > 260)) ||
          (!is_valid_mm(dFrontCoreMm) && is_valid_mm(dFront) && (dFront > 300))))) {
      // Fresh TRACK after OUTER/FRONT transitions: briefly cap steer so Ackermann
      // can settle before committing to another strong correction.
      servoRaw = clampi(servoRaw, SERVO_STRAIGHT - 8, SERVO_STRAIGHT + 8);
    }
    const bool reacquirePhase = (mode == WF_OUTER_SEEK) ||
                                reacquireLostStrong ||
                                followLostRecover ||
                                (gWfCapturedWall && (nFollow <= 2)) ||
                                (followConf < 0.18f) ||
                                (gWfCapturedWall && !followCapturedAligned);
    const bool captureAlignMode = (mode == WF_OUTER_SEEK) &&
                                  followEqValid &&
                                  dSideValid &&
                                  (dSideMin <= 440) &&
                                  (followEqAbsMm <= 130.0f) &&
                                  !reacquireLostStrong;
    const bool alignMode = ((mode == WF_TRACK) && !reacquirePhase) || captureAlignMode;
    const bool alignFrontOk = !is_valid_mm(dFront) || (dFront > WF_ALIGN_FRONT_MIN_MM);
    const bool alignSideOk = !dSideValid || (dSideMin > WF_ALIGN_SIDE_NEAR_MM);
    const bool alignAllowed = alignMode && !innerCornerPrecaution && !innerPreCue &&
                              !frontCritical && !frontBlockedEnter &&
                              !frontBlockedKeep && alignFrontOk && alignSideOk;
    int servoPlan = servoRaw;
    bool alignTrimMode = false;
    if (alignAllowed && dSideValid) {
      const float eSideMm = (float)dSideMin - gWfTargetMm; // +: far from followed side
      if (fabsf(eSideMm) <= WF_ALIGN_NEAR_DB_MM) {
        const int towardSign = -sideSign; // toward followed side
        const bool pairOpeningAway =
            followEqValid && (followEqErrMm > (float)WF_ALIGN_PAIR_SETTLE_MM);
        if (pairOpeningAway) {
          // Near target distance but still yaw-open from the followed wall:
          // hold a small continuous toward-wall trim instead of pulsing to neutral.
          const float settle01 = clamp01((followEqErrMm - (float)WF_ALIGN_PAIR_SETTLE_MM) /
                                         fmaxf(1.0f, (float)WF_EQ_CAPTURE_MM - (float)WF_ALIGN_PAIR_SETTLE_MM));
          const int trimDeg = WF_ALIGN_TRIM_MIN_DEG +
                              (int)roundf((float)(WF_ALIGN_TRIM_MAX_DEG - WF_ALIGN_TRIM_MIN_DEG + 1) * settle01);
          const int towardCmd = (servoPlan - SERVO_STRAIGHT) * towardSign;
          if (towardCmd < trimDeg) {
            servoPlan = clampi(SERVO_STRAIGHT + towardSign * trimDeg, SERVO_LEFT, SERVO_RIGHT);
          }
          alignTrimMode = true;
        } else {
          // Near target side distance: soften command and apply a small deadband.
          const float k = clamp01(fabsf(eSideMm) / WF_ALIGN_NEAR_DB_MM);
          const float scale = WF_ALIGN_NEAR_SOFT_SCALE + (1.0f - WF_ALIGN_NEAR_SOFT_SCALE) * k;
          const int d = (int)roundf((float)(servoPlan - SERVO_STRAIGHT) * scale);
          servoPlan = clampi(SERVO_STRAIGHT + d, SERVO_LEFT, SERVO_RIGHT);
          if (abs(servoPlan - SERVO_STRAIGHT) <= WF_ALIGN_NEAR_NEUTRAL_DEG) {
            servoPlan = SERVO_STRAIGHT;
          }
          // Small persistent corrections should behave like trim, not like
          // pulse-neutral-pulse cycles. Use a gentle continuous trim while the
          // wall is already captured/aligned and geometry is not strongly deviant.
          const bool trimEligible =
              (mode == WF_TRACK) &&
              followCapturedAligned &&
              (fabsf(eSideMm) >= WF_ALIGN_TRIM_MM) &&
              (!followEqValid || (followEqAbsMm <= 120.0f));
          if (trimEligible) {
            const int trimSign = (eSideMm > 0.0f) ? (-sideSign) : sideSign;
            const float trim01 = clamp01((fabsf(eSideMm) - WF_ALIGN_TRIM_MM) /
                                         fmaxf(1.0f, WF_ALIGN_NEAR_DB_MM - WF_ALIGN_TRIM_MM));
            const int trimDeg = WF_ALIGN_TRIM_MIN_DEG +
                                (int)roundf((float)(WF_ALIGN_TRIM_MAX_DEG - WF_ALIGN_TRIM_MIN_DEG) * trim01);
            const int trimCmd = (servoPlan - SERVO_STRAIGHT) * trimSign;
            if (trimCmd < trimDeg) {
              servoPlan = clampi(SERVO_STRAIGHT + trimSign * trimDeg, SERVO_LEFT, SERVO_RIGHT);
            }
            alignTrimMode = true;
          }
        }
      } else if (eSideMm > WF_ALIGN_NEAR_DB_MM) {
        // Far from followed side: make the align pulse harder toward that side.
        const float far = clamp01((eSideMm - WF_ALIGN_NEAR_DB_MM) / WF_ALIGN_FAR_SPAN_MM);
        const int towardSign = -sideSign; // toward followed side
        const int biasDeg = WF_ALIGN_FAR_BIAS_MIN_DEG +
                            (int)roundf((float)(WF_ALIGN_FAR_BIAS_MAX_DEG - WF_ALIGN_FAR_BIAS_MIN_DEG) * far);
        servoPlan = clampi(servoPlan + towardSign * biasDeg, SERVO_LEFT, SERVO_RIGHT);
      }
    }
    int servoCmd = servoPlan;
    bool alignHoldActive = false;
    bool alignNeutralActive = false;
    if (!alignAllowed) {
      gWfAlignHoldTicks = 0;
      gWfAlignNeutralTicks = 0;
      gWfAlignServo = SERVO_STRAIGHT;
    } else if (alignTrimMode) {
      gWfAlignHoldTicks = 0;
      gWfAlignNeutralTicks = 0;
      gWfAlignServo = SERVO_STRAIGHT;
      servoCmd = servoPlan;
    } else if (gWfAlignHoldTicks > 0) {
      alignHoldActive = true;
      servoCmd = gWfAlignServo;
      gWfAlignHoldTicks--;
      if (gWfAlignHoldTicks == 0) gWfAlignNeutralTicks = WF_ALIGN_NEUTRAL_TICKS;
    } else if (gWfAlignNeutralTicks > 0) {
      alignNeutralActive = true;
      servoCmd = SERVO_STRAIGHT;
      gWfAlignNeutralTicks--;
    } else {
      const int mag = abs(servoPlan - SERVO_STRAIGHT);
      const int delta = abs(servoPlan - gWfServoCmd);
      if (freshSensFrame && (mag >= WF_ALIGN_TRIGGER_DEG) && (delta >= WF_ALIGN_MIN_DELTA_DEG)) {
        const uint8_t holdTicks = (mag >= WF_ALIGN_STRONG_DEG)
          ? WF_ALIGN_HOLD_TICKS_STRONG
          : WF_ALIGN_HOLD_TICKS_MILD;
        gWfAlignServo = servoPlan;
        servoCmd = gWfAlignServo;
        alignHoldActive = true;
        gWfAlignHoldTicks = (holdTicks > 0) ? (uint8_t)(holdTicks - 1) : 0;
        if (gWfAlignHoldTicks == 0) gWfAlignNeutralTicks = WF_ALIGN_NEUTRAL_TICKS;
      }
    }
    uint16_t servoFrontMm = dFrontCoreMm;
    if (!is_valid_mm(servoFrontMm) ||
        (is_valid_mm(dFront) && dFront < servoFrontMm)) {
      servoFrontMm = dFront;
    }
    if (!is_valid_mm(servoFrontMm) ||
        (is_valid_mm(dFrontEdge) && dFrontEdge < servoFrontMm)) {
      servoFrontMm = dFrontEdge;
    }
    if (!is_valid_mm(servoFrontMm) ||
        (is_valid_mm(dFrontBlockedMm) && dFrontBlockedMm < servoFrontMm)) {
      servoFrontMm = dFrontBlockedMm;
    }
    int servoStep = 10;
    if (mode == WF_TRACK) {
      servoStep = 8;
      if (is_valid_mm(servoFrontMm) && servoFrontMm <= 560) servoStep = 12;
      if (is_valid_mm(servoFrontMm) && servoFrontMm <= 420) servoStep = 16;
      if (dSideValid && dSideMin <= 240 && servoStep < 12) servoStep = 12;
      if (dSideValid && dSideMin <= 180 && servoStep < 18) servoStep = 18;
    }
    if (mode == WF_OUTER_SEEK) {
      servoStep = 14;
      if ((nFollow <= 2) || !dSideValid) servoStep = 20;
      if ((nFollow == 0) && !dSideValid) servoStep = 24;
    }
    if (mode == WF_FRONT_BLOCK) {
      servoStep = 12;
      if (is_valid_mm(servoFrontMm) && servoFrontMm <= 520) servoStep = 14;
      if (is_valid_mm(servoFrontMm) && servoFrontMm <= 340) servoStep = 18;
      if (is_valid_mm(servoFrontMm) && servoFrontMm <= 260) servoStep = 22;
      if (is_valid_mm(servoFrontMm) && servoFrontMm <= 190) servoStep = 24;
    }
    const int servoCmdDelta = abs(servoCmd - gWfServoCmd);
    const bool urgentWfSteer =
        (servoCmdDelta >= 12) &&
        ((mode == WF_FRONT_BLOCK) ||
         (mode == WF_OUTER_SEEK) ||
         frontConvInner ||
         (wfPrecautionName && wfPrecautionName[0] == 'I') ||
         (is_valid_mm(servoFrontMm) && servoFrontMm <= 650) ||
         (dSideValid && dSideMin <= 220));
    if (urgentWfSteer) {
      int urgentStep = 24;
      if ((mode == WF_OUTER_SEEK) && (!dSideValid || nFollow <= 1)) urgentStep = 30;
      if (is_valid_mm(servoFrontMm) && servoFrontMm <= 420) urgentStep = 30;
      if (is_valid_mm(servoFrontMm) && servoFrontMm <= 280) urgentStep = 36;
      if (servoStep < urgentStep) servoStep = urgentStep;
    }
    if (alignHoldActive && (abs(servoCmd - SERVO_STRAIGHT) >= WF_ALIGN_STRONG_DEG) && servoStep < 14) {
      servoStep = 14;
    }
    if (alignNeutralActive && servoStep < 14) {
      servoStep = 14;
    }
    const int prevServoErr = gWfServoCmd - SERVO_STRAIGHT;
    const int cmdServoErr = servoCmd - SERVO_STRAIGHT;
    const bool wantCenter = (abs(cmdServoErr) <= WF_UNWIND_NEAR_CENTER_DEG);
    const bool farFromCenter = (abs(prevServoErr) >= WF_UNWIND_FROM_DEG);
    const bool unwindActive = (abs(cmdServoErr) + 2 < abs(prevServoErr));
    const bool crossCenterRecover = ((prevServoErr > 0) && (cmdServoErr <= 0)) ||
                                    ((prevServoErr < 0) && (cmdServoErr >= 0));
    if ((wantCenter || unwindActive || crossCenterRecover) && farFromCenter) {
      int unwindStep = wantCenter ? WF_UNWIND_STEP_DEG : WF_UNWIND_STEP_ACTIVE_DEG;
      const bool frontTightRecover =
          (is_valid_mm(dFrontCoreMm) && (dFrontCoreMm <= 340)) ||
          (is_valid_mm(dFrontEdge) && (dFrontEdge <= 260)) ||
          (is_valid_mm(dFront) && (dFront <= 300));
      if (frontTightRecover) {
        int tightStep = wantCenter ? WF_UNWIND_STEP_FRONT_DEG : WF_UNWIND_STEP_ACTIVE_FRONT_DEG;
        if (unwindStep < tightStep) unwindStep = tightStep;
      }
      if (crossCenterRecover && unwindStep < WF_UNWIND_STEP_CROSS_DEG) {
        unwindStep = WF_UNWIND_STEP_CROSS_DEG;
      }
      const bool urgentPrecautionCross =
          crossCenterRecover &&
          (abs(cmdServoErr) >= 6) &&
          ((mode == WF_FRONT_BLOCK) || frontConvInner ||
           (wfPrecautionName && wfPrecautionName[0] == 'I')) &&
          ((is_valid_mm(dFrontCoreMm) && dFrontCoreMm <= 560) ||
           (is_valid_mm(dFront) && dFront <= 560));
      if (urgentPrecautionCross && unwindStep < WF_UNWIND_STEP_ACTIVE_FRONT_DEG) {
        unwindStep = WF_UNWIND_STEP_ACTIVE_FRONT_DEG;
      }
      if (servoStep < unwindStep) servoStep = unwindStep;
    }
    servo = clampi(servoCmd, gWfServoCmd - servoStep, gWfServoCmd + servoStep);
    gWfServoCmd = servo;
    movement_apply_angle(servo, reason, spd);

    snprintf(gBug2Dbg, sizeof(gBug2Dbg),
             "sv=%d cmd=%d L=[%u,%u,%u,%u] M=[%u,%u,%u,%u] R=[%u,%u,%u,%u] WF s=%c m=%s dF=%u dS=%u nF=%u r(%.2f,%.2f) lD=%.2f yD=%.0f yC=%.0f lA=%lu lB=%c iC=%u prec=%s uO=%.1f uB=%.1f uF=%.1f uL=%.1f uS=%.1f uC=%.1f uI=%.1f uN=%.1f uE=%.1f uP=%.1f uG=%.1f uQ=%.1f uR=%.1f uT=%.1f",
             servo, servoCmd,
             (unsigned)zL[0], (unsigned)zL[1], (unsigned)zL[2], (unsigned)zL[3],
             (unsigned)zM[0], (unsigned)zM[1], (unsigned)zM[2], (unsigned)zM[3],
             (unsigned)zR[0], (unsigned)zR[1], (unsigned)zR[2], (unsigned)zR[3],
             isLeft ? 'L' : 'R', wf_mode_name(mode),
             (unsigned)dFront, (unsigned)dSideMin, (unsigned)nFollow,
             robot.x, robot.y,
             lossDistM, lossYawDeg, lossCumYawDeg, (unsigned long)lossAgeMs, lossBucket,
             frontConvInner ? 1U : 0U, wfPrecautionName,
             uOpenBias, uBal, uSideFar, uLost, uSideOpen, uFrontConv,
             uInnerPrecaution, uSideNear, uFrontEdge, uFrontTrend, uHeadGeom, uEq, uRecover, u);
  } break;

  // ---------------------------------------------------------
  case FS_REACHED: {
    movement_stop_reason("bug2_reached", true);
  } break;

  // ---------------------------------------------------------
  case FS_STUCK: {
    if (stuckPhaseMs == 0) {
      stuckPhaseMs = nowMs;
      stuckReversing = true;
    }

    uint32_t stuckElapsed = nowMs - stuckPhaseMs;

    if (stuckElapsed > STUCK_TIMEOUT_MS) {
      movement_stop_reason("bug2_stuck_timeout", true);
      noInterrupts(); gStarted = false; interrupts();
      gHasGoal = false;
      set_state(FS_IDLE);
      log_decision("STUCK_TIMEOUT>IDLE");
      break;
    }

    if (stuckReversing && stuckElapsed < STUCK_REV_MS) {
      FollowSide closerSide = (dL < dR) ? FOLLOW_LEFT : FOLLOW_RIGHT;
      int revServo = (closerSide == FOLLOW_LEFT)
                     ? clampi(SERVO_STRAIGHT + 20, SERVO_LEFT, SERVO_RIGHT)
                     : clampi(SERVO_STRAIGHT - 20, SERVO_LEFT, SERVO_RIGHT);
      movement_apply_reverse(revServo, "bug2_stuck_rev", SPD_ESCAPE);
      set_wf_recovery_dbg(robot, isLeft, "STUCK_REV", revServo, dFront, dSideMin, zL, zM, zR);
    } else if (stuckReversing) {
      stuckReversing = false;
      stuckPhaseMs = 0;
      log_decision(gSide == FOLLOW_LEFT ? "STUCK>WF_L" : "STUCK>WF_R");
      set_state(FS_WF);
    } else {
      movement_stop_reason("bug2_stuck", true);
    }
  } break;

  }  // switch
}
