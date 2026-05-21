#include <Arduino.h>
#include <WiFiUdp.h>

#include "udp_pose.h"
#include "globals.h"
#include "DbgLog.h"

static WiFiUDP udp;
static const uint16_t UDP_PORT = 22222;

static const uint32_t POSE_MAGIC   = 0xB6020B42;
static const uint32_t POSE_VERSION = 1;

// Match Python PACK_FMT = "<IIfffffff"
#pragma pack(push, 1)
struct PosePacket {
  uint32_t magic;
  uint32_t version;
  float t;
  float rx, ry, ryaw;
  float tx, ty, tyaw;
};
#pragma pack(pop)

void udp_setup(){
  if (udp.begin(UDP_PORT)) {
    Serial.printf("[UDP] Listening on port %u\n", UDP_PORT);
  } else {
    Serial.printf("[UDP] FAILED to bind port %u\n", UDP_PORT);
  }
}

void udp_pump(){
  PosePacket lastValid;
  bool haveValid = false;

  // IMPORTANT:
  // If the PC sends pose at a high rate, draining ALL queued UDP datagrams can starve
  // WiFi/HTTP/WebSocket handling (UI freezes / "site unreachable").
  // So we time-budget + packet-budget the drain, and only apply the latest valid packet.
  const uint32_t t0_us = micros();
  const uint32_t BUDGET_US = 2000;   // ~2ms per loop iteration
  const int      MAX_PKTS  = 6;      // and no more than 6 datagrams per call
  int pkts = 0;

  while (pkts < MAX_PKTS && (uint32_t)(micros() - t0_us) < BUDGET_US) {
    int packetSize = udp.parsePacket();
    if (packetSize <= 0) break;

    // If it's not our size, discard quickly.
    if (packetSize < (int)sizeof(PosePacket)) {
      while (udp.available()) udp.read();
      pkts++;
      continue;
    }

    PosePacket pkt;
    int n = udp.read((uint8_t*)&pkt, sizeof(pkt));
    // Drain any remainder of this datagram (if sender sent bigger than expected)
    while (udp.available()) udp.read();
    pkts++;

    if (n == (int)sizeof(pkt) &&
        pkt.magic == POSE_MAGIC &&
        pkt.version == POSE_VERSION)
    {
      // Layer 1: raw UDP packet (OptiTrack -> UDP)
      dbg_push(EV_POSE_RX,
               (int32_t)(pkt.rx * 1000.0f),
               (int32_t)(pkt.ry * 1000.0f),
               1);  // marker=1
      lastValid = pkt;
      haveValid = true;
    }
  }

  if (!haveValid) return;

  uint32_t now_ms = millis();

  // VALIDATION: Check for OptiTrack corruption
  // Layer 2: after UDP parsing, before validation
  dbg_push(EV_POSE_RX,
           (int32_t)(lastValid.rx * 1000.0f),
           (int32_t)(lastValid.ry * 1000.0f),
           2);  // marker=2
  
  // Reject obviously invalid values but log them for debugging
  if (lastValid.rx > 100.0f || lastValid.rx < -100.0f ||
      lastValid.ry > 100.0f || lastValid.ry < -100.0f ||
      lastValid.ryaw > 6.3f || lastValid.ryaw < -6.3f ||
      lastValid.tx > 100.0f || lastValid.tx < -100.0f ||
      lastValid.ty > 100.0f || lastValid.ty < -100.0f) {
    // Corruption detected and rejected
    dbg_push(EV_POSE_RX,
             (int32_t)(lastValid.rx * 1000.0f),
             (int32_t)(lastValid.ry * 1000.0f),
             999);  // marker=999 (invalid)
    return;  // Don't update with corrupted data
  }

  // Update globals ONCE per pump (atomic block kept short)
  noInterrupts();
  gRobotLatest.x        = lastValid.rx;
  gRobotLatest.y        = lastValid.ry;
  gRobotLatest.yaw      = lastValid.ryaw;
  gRobotLatest.stamp_ms = now_ms;

  gTargetLatest.x        = lastValid.tx;
  gTargetLatest.y        = lastValid.ty;
  gTargetLatest.yaw      = lastValid.tyaw;
  gTargetLatest.stamp_ms = now_ms;

  gLastPoseMs = now_ms;
  interrupts();
  
  // Layer 2b: globals updated
  dbg_push(EV_POSE_RX,
           (int32_t)(gRobotLatest.x * 1000.0f),
           (int32_t)(gRobotLatest.y * 1000.0f),
           3);  // marker=3

  // DO NOT log pose every packet (can be very high-rate).
  static uint32_t lastPoseDbgMs = 0;
  if ((uint32_t)(now_ms - lastPoseDbgMs) >= 200) { // <= 5 Hz
    lastPoseDbgMs = now_ms;
    int32_t rx_mm    = (int32_t)(lastValid.rx * 1000.0f);
    int32_t ry_mm    = (int32_t)(lastValid.ry * 1000.0f);
    int32_t yaw_mrad = (int32_t)(lastValid.ryaw * 1000.0f);
    dbg_push(EV_POSE_RX, rx_mm, ry_mm, yaw_mrad);
  }
}


