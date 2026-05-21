#include "nav_core.h"
#include "movement_core.h"
#include "pure_pursuit.h"
#include "DbgLog.h"
#include "ServoControl.h"

static NavState gNav = NAV_IDLE;
static float gGoalX = 0.0f, gGoalY = 0.0f;

static const float NAV_STOP_DIST_M = 0.25f;
static const float NAV_SPEED_SCALE = 0.8f;
static const uint32_t NAV_TICK_MS  = 60;

static uint32_t gLastTick = 0;

static const PurePursuitParams PP = {
  0.14f, 30.0f, 0.15f, 0.60f, SERVO_CENTER, 30, SERVO_MIN, SERVO_MAX
};

void nav_setup(){
  gNav = NAV_IDLE;
  gLastTick = millis();
}

void nav_start(float goalX, float goalY){
  gGoalX = goalX;
  gGoalY = goalY;
  gNav = NAV_ACTIVE;
}

void nav_stop(){
  gNav = NAV_IDLE;
  movement_stop_reason("nav_stop", true);
  dbg_push(EV_MARK, 505, 0, 0);
  record_stop_cause(505, 0, dbg_hash16("nav_stop"), "nav_stop");
  dbg_push(EV_MARK, 520, 7, 0);
  noInterrupts(); gStarted = false; interrupts();
}

void nav_clear(){
  gNav = NAV_IDLE;
}


NavState nav_state(){ return gNav; }

bool nav_get_goal(float& x, float& y){
  if (gNav == NAV_IDLE) return false;
  x = gGoalX; y = gGoalY;
  return true;
}

void nav_tick(const Pose& robot, bool started){
  uint32_t now = millis();
  if (now - gLastTick < NAV_TICK_MS) return;
  gLastTick = now;

  if (!started){
    if (gNav != NAV_IDLE) gNav = NAV_IDLE;
    return;
  }
  if (gNav != NAV_ACTIVE) return;

  Pose t;
  t.x = gGoalX; t.y = gGoalY; t.yaw = 0; t.stamp_ms = 1;

  int angle; float dist;
  if (!pp_compute_angle(robot, t, PP, angle, dist)){
    movement_stop_reason("nav_no_pose", true);
    gNav = NAV_IDLE;
    return;
  }

if (dist < NAV_STOP_DIST_M){
  movement_stop_reason("nav_reached", true);

  dbg_push(EV_MARK, 506, 0, 0);
  record_stop_cause(506, 0, dbg_hash16("nav_reached"), "nav_reached");

  dbg_push(EV_MARK, 520, 8, 0);
  noInterrupts();
  gStarted = false;
  interrupts();

  gNav = NAV_REACHED;   // or NAV_IDLE if you prefer
  return;
}


  movement_set_pp_debug("nav_pp", robot, t, dist, angle,
                        servo_shape_autonomous_angle(angle), NAV_SPEED_SCALE);
  movement_apply_angle(angle, "nav_pp", NAV_SPEED_SCALE);
}
