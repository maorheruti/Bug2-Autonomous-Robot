// bug2_core.h — Zone-Combo Bug2 State Machine (v3)
//
// Minimal state machine: WF decisions are a priority-ordered list of
// zone-combo checks.  Each combo maps to a servo angle + speed.
// Test-stop lines halt the robot after one tick for observation.
//
// Public API unchanged from v1/v2 (no .ino changes needed).
//
// Backup: v2 VFH code → bug2_core_v2_vfh_01mar2026.cpp/h  (01 Mar 2026)
// ---------------------------------------------------------------------------
#pragma once
#include <Arduino.h>
#include "globals.h"

const char* bug2_dbg_line();
const char* bug2_decision_log();

struct Bug2WfTune {
  float target_mm;
  float dist_k;
  float yaw_k;
  float bear_k;
  float opp_k;
  float front_k;
  uint16_t zone_emergency_mm;
  uint16_t zone_close_mm;
  uint16_t zone_mid_mm;
  uint16_t deadend_rev_ms;
  uint16_t deadend_turn_ms;
  uint16_t mode_dwell_ms;
  float fblk_enter_conf;
  float fblk_exit_conf;
  uint16_t fblk_enter_mm;
  uint16_t fblk_exit_mm;
  float dend_enter_front_conf;
  float dend_enter_opp_conf;
  float dend_enter_follow_conf;
  float dend_exit_front_conf;
  float dend_exit_opp_conf;
};

enum Bug2State {
  BUG2_IDLE = 0,
  BUG2_GO_TO_GOAL,
  BUG2_FOLLOW_WALL,
  BUG2_WF_REACQUIRE,
  BUG2_REACHED,
  BUG2_STUCK
};

enum Bug2RunMode {
  BUG2_MODE_FULL = 0,
  BUG2_MODE_WF_ONLY
};

void bug2_setup();
void bug2_start(float goalX, float goalY);
void bug2_stop(const char* why = "bug2_stop");
void bug2_tick(const Pose& robot, bool started, uint16_t dL, uint16_t dM, uint16_t dR);
Bug2State bug2_state();
void bug2_set_mode(Bug2RunMode mode);
Bug2RunMode bug2_run_mode();
bool bug2_has_goal(float& x, float& y);
void bug2_wf_tune_get(Bug2WfTune& out);
void bug2_wf_tune_set(const Bug2WfTune& in);
