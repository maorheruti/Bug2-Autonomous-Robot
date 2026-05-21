// movement_core.h
#pragma once
#include <Arduino.h>
#include "globals.h"

void movement_setup();
void movement_loop_tick();  // legacy pure pursuit follower

// Diagnostic helpers
const char* movement_reason();
int current_angle();
int movement_last_requested_angle();
int movement_last_physical_angle();
const char* movement_dbg_line();
void movement_set_pp_debug(const char* label,
                           const Pose& r,
                           const Pose& t,
                           float distM,
                           int requestedAngleDeg,
                           int physicalAngleDeg,
                           float speedScale);

// === pure-pursuit computation (NO actuation) ===
// Returns false if no valid target pose (t.stamp_ms == 0)
bool movement_pp_compute_angle(const Pose& r, const Pose& t,
                               int& outAngleDeg, float& outDistM);

// === actuation helpers (servo + motors) ===
// Apply servo + motors using the existing mapping in movement_core.
// speedScale: 0.1..1.0 (nav slow uses ~0.45)
void movement_apply_angle(int angleDeg, const char* reason, float speedScale = 1.0f);

// Reverse actuation helper (servo + reverse motors)
void movement_apply_reverse(int angleDeg, const char* reason, float speedScale = 1.0f);

// Stop motors, set reason, optionally center servo
void movement_stop_reason(const char* reason, bool centerServo = true);

// Diagnostic steering-only command: update the tracked angle/reason and move
// the servo without applying motor PWM.
void movement_servo_only(int angleDeg, const char* reason);

// Calibration-only steering command. Allows measured servo endpoints outside the
// conservative control range while keeping telemetry/reason state in sync.
void movement_servo_calibration(int angleDeg, const char* reason);

// PP Control enable/disable (for Bug2 to prevent fallback controller)
void movement_set_pp_control_enabled(bool enabled);
bool movement_is_pp_control_enabled();
