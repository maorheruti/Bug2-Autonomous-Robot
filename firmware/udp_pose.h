#pragma once

// Initialize UDP listener (bind to port 22222).
void udp_setup();

// Pump UDP socket, parse any available OptiTrack packets, and
// update gRobotLatest / gTargetLatest / gLastPoseMs (from globals.h).
void udp_pump();


