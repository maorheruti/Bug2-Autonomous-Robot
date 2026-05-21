#pragma once
#include <Arduino.h>
#include <WebServer.h>
#include "globals.h"

// Initialise HTTP UI + WebSocket telemetry on top of an existing WebServer.
// - server: global WebServer(80) instance from NetESP.ino
// This will register "/" to serve an HTML dashboard page.
void ui_init(WebServer& server);

// To be called regularly from loop(); handles WebSocket events.
void ui_loop();

// Broadcast a telemetry frame over WebSocket to all connected clients.
// Call this at approximately 5-20 Hz from the main loop.
void ui_broadcast(const Pose& robot,
                  const Pose& target,
                  bool started,
                  const char* reason,
                  int   servoDeg,
                  int   pwmA,
                  int   pwmB,
                  char  dirA,
                  char  dirB,
                  uint16_t dL,
                  uint16_t dM,
                  uint16_t dR);



