// obstacle_map.h — Lightweight obstacle map built from ToF sensor detections
//
// Two buffers:
//   1. Isolated points — single detections not yet connected.
//   2. Segments — connected polylines built when nearby points are found.
//
// Usage:
//   obs_map_init()  — call once in setup()
//   obs_map_feed()  — call after each sensors_read() with current robot pose
//   obs_map_clear() — reset both buffers
//   obs_map_json()  — serialize for HTTP endpoint

#pragma once
#include <Arduino.h>
#include "globals.h"

#define OBS_MAX_ISOLATED  128
#define OBS_MAX_SEGMENTS   32
#define OBS_MAX_VERTICES  256

struct ObsPoint {
  float x, y;
};

void     obs_map_init();
void     obs_map_feed(const Pose& robot);   // project zone4 → world, add points
void     obs_map_clear();
size_t   obs_map_json(char* buf, size_t cap);
uint16_t obs_map_isolated_count();
uint16_t obs_map_segment_count();
uint16_t obs_map_vertex_count();
