#pragma once
#include <Arduino.h>
#include "globals.h"

struct PurePursuitParams {
  float wheelbase_m;      // e.g. 0.14
  float max_steer_deg;    // e.g. 30
  float min_lookahead_m;  // e.g. 0.15
  float max_lookahead_m;  // e.g. 0.60
  int   servo_center_deg; // installed-servo center command
  int   servo_half_range; // legacy fallback when min/max are not overridden
  int servo_min_deg = 60;
  int servo_max_deg = 120;
};

// Compute desired servo angle (deg) using PP towards target point (t.x,t.y).
// Returns false if target is invalid (t.stamp_ms==0).
bool pp_compute_angle(const Pose& r, const Pose& t,
                      const PurePursuitParams& P,
                      int& outAngleDeg,
                      float& outDistM);

