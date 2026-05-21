#pragma once
#include <Arduino.h>

struct Pose {
  float x;
  float y;
  float yaw;
  uint32_t stamp_ms;
};

extern volatile bool     gStarted;
extern volatile Pose     gRobotLatest;
extern volatile Pose     gTargetLatest;
extern volatile uint32_t gLastPoseMs;

inline Pose snapshotPose(volatile Pose& v) {
  Pose p;
  noInterrupts();
  p.x        = v.x;
  p.y        = v.y;
  p.yaw      = v.yaw;
  p.stamp_ms = v.stamp_ms;
  interrupts();
  
  // Layer 3 debug removed from here - causes circular include dependency
  // Corruption will be caught by Layer 2b (globals update) and Layer 4 (telemetry)
  
  return p;
}

inline bool snapshotBool(volatile bool& v) {
  bool b;
  noInterrupts();
  b = v;
  interrupts();
  return b;
}

inline uint32_t snapshotU32(volatile uint32_t& v) {
  uint32_t w;
  noInterrupts();
  w = v;
  interrupts();
  return w;
}

// === NEW: global sensor snapshot values (mm) ===
extern volatile uint16_t gSensL_mm;
extern volatile uint16_t gSensM_mm;
extern volatile uint16_t gSensR_mm;

struct SensorMatrixData {
  int16_t distance[64];
  uint8_t status[64];
};

struct SensorZone4Data {
  uint16_t z[4];
  uint8_t  nValid[4];   // number of valid status-5/9 cells per zone
};

extern volatile SensorMatrixData gSensL_matrix;
extern volatile SensorMatrixData gSensM_matrix;
extern volatile SensorMatrixData gSensR_matrix;

extern volatile SensorZone4Data gSensL_z4;
extern volatile SensorZone4Data gSensM_z4;
extern volatile SensorZone4Data gSensR_z4;

// === Sensor read tracking (corruption detection) ===
extern volatile uint32_t gSensFrameId;
extern volatile uint32_t gSensCorruptCount;
extern volatile uint32_t gSensCorruptLastMs;

// === Sensor canary + CRC detection ===
extern volatile uint32_t gSensCanaryA;
extern volatile uint32_t gSensCanaryB;
extern volatile uint32_t gSensCanaryTripCount;
extern volatile uint32_t gSensCanaryLastMs;
extern volatile uint32_t gSensCrcTripCount;
extern volatile uint32_t gSensCrcLastMs;

// === Sensor corruption snapshot (raw values + source) ===
extern volatile uint32_t gSensCorruptSrc;       // 1=read path, 2=telemetry
extern volatile uint32_t gSensCorruptLastFrame;
extern volatile uint16_t gSensCorruptLastRawL;
extern volatile uint16_t gSensCorruptLastRawM;
extern volatile uint16_t gSensCorruptLastRawR;

// === Loop stall detection ===
extern volatile uint64_t gLoopBeatUs;
extern volatile uint32_t gLoopStallCount;
extern volatile uint32_t gLoopStallLastMs;

// === Loop canary guards (memory overwrite detection) ===
extern volatile uint32_t gLoopCanaryA;
extern volatile uint32_t gLoopCanaryB;
extern volatile uint32_t gLoopCanaryTripCount;
extern volatile uint32_t gLoopCanaryLastMs;

// === Last stop cause snapshot ===
extern volatile uint32_t gStopCauseTag;        // 501-508 or 509
extern volatile uint32_t gLastStopMs;
extern volatile uint32_t gLastStopBug2;
extern volatile uint32_t gLastStopReasonHash;
extern char gLastStopReason[24];
extern volatile bool gEverStarted;

// === Latched stop snapshot (first stop after reset) ===
extern volatile uint32_t gLatchedStopTag;
extern volatile uint32_t gLatchedStopMs;
extern volatile uint32_t gLatchedStopBug2;
extern volatile uint32_t gLatchedStopReasonHash;
extern char gLatchedStopReason[24];

void record_stop_cause(uint32_t tag, uint32_t b2, uint32_t reasonHash, const char* reason);
void reset_stop_cause();




