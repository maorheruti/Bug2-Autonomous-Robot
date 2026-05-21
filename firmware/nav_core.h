#pragma once
#include <Arduino.h>
#include "globals.h"

// simple nav state
enum NavState : uint8_t { NAV_IDLE, NAV_ACTIVE, NAV_REACHED };

void nav_setup();
void nav_start(float goalX, float goalY);
void nav_stop();
void nav_tick(const Pose& robot, bool started);  // call from loop

void nav_clear();
NavState nav_state();
const char* nav_reason();

// For UI/debug
bool nav_get_goal(float& x, float& y);




