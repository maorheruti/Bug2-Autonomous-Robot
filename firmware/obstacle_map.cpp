// obstacle_map.cpp — Lightweight obstacle map from ToF zone detections
//
// Maintains two buffers:
//   gIsolated[] — single world-coordinate points not yet part of a structure
//   gSegments[] + gVertices[] — connected polylines (structures)
//
// When a new point is added:
//   1. Deduplicate (skip if within DEDUP_DIST of existing point)
//   2. Try connecting to nearest isolated point within CONNECT_DIST → new segment
//   3. Try extending nearest segment endpoint within CONNECT_DIST
//   4. Otherwise add as new isolated point
//
// Feed is called after each sensor read with the current robot pose.
// World projection matches the UI (with same lateral-mirror correction).

#include "obstacle_map.h"
#include "globals.h"
#include "Sensors.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// ---------------------------------------------------------------------------
// Buffers
// ---------------------------------------------------------------------------
static ObsPoint   gIsolated[OBS_MAX_ISOLATED];
static uint16_t   gIsolatedN = 0;

struct ObsSegment {
  uint16_t start;   // first index in gVertices[]
  uint16_t count;   // number of vertices in this polyline
};

static ObsPoint   gVertices[OBS_MAX_VERTICES];
static uint16_t   gVertexN = 0;
static ObsSegment gSegments[OBS_MAX_SEGMENTS];
static uint16_t   gSegmentN = 0;

// ---------------------------------------------------------------------------
// Tunables
// ---------------------------------------------------------------------------
static constexpr float DEDUP_DIST    = 0.03f;   // 3 cm — skip if already mapped
static constexpr float CONNECT_DIST  = 0.12f;   // 12 cm — connect nearby points
static constexpr float SNAP_GRID     = 0.05f;   // 5 cm grid snap before dedup

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static inline float snap(float v) {
  return roundf(v / SNAP_GRID) * SNAP_GRID;
}

static inline float dist2d(float x1, float y1, float x2, float y2) {
  float dx = x2 - x1, dy = y2 - y1;
  return sqrtf(dx * dx + dy * dy);
}

// Check if a point already exists within DEDUP_DIST
static bool is_duplicate(float x, float y) {
  for (uint16_t i = 0; i < gIsolatedN; i++) {
    if (dist2d(x, y, gIsolated[i].x, gIsolated[i].y) < DEDUP_DIST) return true;
  }
  for (uint16_t i = 0; i < gVertexN; i++) {
    if (dist2d(x, y, gVertices[i].x, gVertices[i].y) < DEDUP_DIST) return true;
  }
  return false;
}

// Find nearest isolated point within threshold, return index or -1
static int find_nearest_isolated(float x, float y, float thresh) {
  int best = -1;
  float bestD = thresh;
  for (uint16_t i = 0; i < gIsolatedN; i++) {
    float d = dist2d(x, y, gIsolated[i].x, gIsolated[i].y);
    if (d < bestD) { bestD = d; best = (int)i; }
  }
  return best;
}

// Remove isolated point at index (swap with last)
static void remove_isolated(uint16_t idx) {
  if (idx < gIsolatedN - 1) gIsolated[idx] = gIsolated[gIsolatedN - 1];
  gIsolatedN--;
}

// Find segment whose start or end vertex is nearest to (x,y) within threshold.
// Returns segment index or -1.  whichEnd: 0 = start vertex, 1 = end vertex.
static int find_nearest_segment_end(float x, float y, float thresh, uint8_t& whichEnd) {
  int best = -1;
  float bestD = thresh;
  for (uint16_t s = 0; s < gSegmentN; s++) {
    uint16_t si = gSegments[s].start;
    uint16_t ei = si + gSegments[s].count - 1;
    float ds = dist2d(x, y, gVertices[si].x, gVertices[si].y);
    float de = dist2d(x, y, gVertices[ei].x, gVertices[ei].y);
    if (ds < bestD) { bestD = ds; best = (int)s; whichEnd = 0; }
    if (de < bestD) { bestD = de; best = (int)s; whichEnd = 1; }
  }
  return best;
}

// ---------------------------------------------------------------------------
// Core: add a single world-coordinate point
// ---------------------------------------------------------------------------
static void add_point(float x, float y) {
  // Snap to grid — prevents rounding-display duplicates
  x = snap(x);
  y = snap(y);

  if (is_duplicate(x, y)) return;

  // 1. Try to connect to nearest isolated point → create new segment
  int nearIso = find_nearest_isolated(x, y, CONNECT_DIST);
  if (nearIso >= 0 && gSegmentN < OBS_MAX_SEGMENTS && gVertexN + 2 <= OBS_MAX_VERTICES) {
    ObsPoint iso = gIsolated[nearIso];
    remove_isolated((uint16_t)nearIso);
    gSegments[gSegmentN].start = gVertexN;
    gSegments[gSegmentN].count = 2;
    gVertices[gVertexN++] = iso;
    gVertices[gVertexN++] = {x, y};
    gSegmentN++;
    return;
  }

  // 2. Try to extend nearest segment endpoint
  uint8_t whichEnd = 0;
  int nearSeg = find_nearest_segment_end(x, y, CONNECT_DIST, whichEnd);
  if (nearSeg >= 0) {
    ObsSegment& seg = gSegments[nearSeg];
    // Can only append to end if segment is last in vertex array
    if (whichEnd == 1 && seg.start + seg.count == gVertexN && gVertexN < OBS_MAX_VERTICES) {
      gVertices[gVertexN++] = {x, y};
      seg.count++;
      return;
    }
    // Start extension or non-contiguous: fall through to isolated
  }

  // 3. Add as isolated point
  if (gIsolatedN < OBS_MAX_ISOLATED) {
    gIsolated[gIsolatedN++] = {x, y};
  } else {
    // Buffer full: FIFO evict oldest
    memmove(gIsolated, gIsolated + 1, (OBS_MAX_ISOLATED - 1) * sizeof(ObsPoint));
    gIsolated[OBS_MAX_ISOLATED - 1] = {x, y};
  }
}

// ---------------------------------------------------------------------------
// Merge pass: connect isolated points that are close to each other
// Called periodically from obs_map_feed to build structures over time.
// ---------------------------------------------------------------------------
static void merge_isolated_pass() {
  // --- Phase 1: connect closest pair of isolated points into a new segment ---
  if (gIsolatedN >= 2 && gSegmentN < OBS_MAX_SEGMENTS && gVertexN + 2 <= OBS_MAX_VERTICES) {
    int bestA = -1, bestB = -1;
    float bestD = CONNECT_DIST;
    for (uint16_t i = 0; i < gIsolatedN; i++) {
      for (uint16_t j = i + 1; j < gIsolatedN; j++) {
        float d = dist2d(gIsolated[i].x, gIsolated[i].y,
                          gIsolated[j].x, gIsolated[j].y);
        if (d < bestD) { bestD = d; bestA = i; bestB = j; }
      }
    }
    if (bestA >= 0) {
      ObsPoint pa = gIsolated[bestA];
      ObsPoint pb = gIsolated[bestB];
      if (bestB > bestA) {
        remove_isolated((uint16_t)bestB);
        remove_isolated((uint16_t)bestA);
      } else {
        remove_isolated((uint16_t)bestA);
        remove_isolated((uint16_t)bestB);
      }
      gSegments[gSegmentN].start = gVertexN;
      gSegments[gSegmentN].count = 2;
      gVertices[gVertexN++] = pa;
      gVertices[gVertexN++] = pb;
      gSegmentN++;
    }
  }

  // --- Phase 2: absorb isolated points into existing segment endpoints ---
  for (int i = (int)gIsolatedN - 1; i >= 0; i--) {
    uint8_t whichEnd = 0;
    int nearSeg = find_nearest_segment_end(gIsolated[i].x, gIsolated[i].y, CONNECT_DIST, whichEnd);
    if (nearSeg >= 0) {
      ObsSegment& seg = gSegments[nearSeg];
      if (whichEnd == 1 && seg.start + seg.count == gVertexN && gVertexN < OBS_MAX_VERTICES) {
        gVertices[gVertexN++] = gIsolated[i];
        seg.count++;
        remove_isolated((uint16_t)i);
      }
    }
  }

  // --- Phase 3: merge segments that share nearby endpoints ---
  for (uint16_t a = 0; a < gSegmentN; a++) {
    for (uint16_t b = a + 1; b < gSegmentN; b++) {
      ObsSegment& sa = gSegments[a];
      ObsSegment& sb = gSegments[b];
      uint16_t aEnd   = sa.start + sa.count - 1;
      uint16_t bStart = sb.start;
      uint16_t bEnd   = sb.start + sb.count - 1;
      // Check all four endpoint combinations
      float d = dist2d(gVertices[aEnd].x, gVertices[aEnd].y,
                        gVertices[bStart].x, gVertices[bStart].y);
      float d2 = dist2d(gVertices[aEnd].x, gVertices[aEnd].y,
                         gVertices[bEnd].x, gVertices[bEnd].y);
      float d3 = dist2d(gVertices[sa.start].x, gVertices[sa.start].y,
                         gVertices[bStart].x, gVertices[bStart].y);
      float d4 = dist2d(gVertices[sa.start].x, gVertices[sa.start].y,
                         gVertices[bEnd].x, gVertices[bEnd].y);
      float minD = d;
      if (d2 < minD) minD = d2;
      if (d3 < minD) minD = d3;
      if (d4 < minD) minD = d4;
      if (minD < CONNECT_DIST) {
        // Merge if B is contiguous and immediately after A in vertex array
        if (sb.start + sb.count == gVertexN && sa.start + sa.count == sb.start) {
          sa.count += sb.count;
          if (b < gSegmentN - 1) gSegments[b] = gSegments[gSegmentN - 1];
          gSegmentN--;
          return; // restart on next merge_pass call
        }
      }
    }
  }
}

// ---------------------------------------------------------------------------
// Sensor projection to world coordinates
// Matches UI addObstacleFromSensor (with lateral-mirror correction).
// ---------------------------------------------------------------------------
struct SensMount {
  float baseAngleRad;  // sensor heading offset (physical, before UI correction)
  float offX, offY;    // mounting offset in robot frame
  float fovDeg;        // field of view
};

// Physical mounting parameters (same as UiDashboard.h addObstacleFromZones)
static const SensMount gMount[3] = {
  { +70.0f * (float)M_PI / 180.0f,  0.13f - 0.035355f,  +0.035355f, 45.0f },  // LEFT
  {  0.0f,                           0.13f,               0.0f,       60.0f },   // MID
  { -70.0f * (float)M_PI / 180.0f,  0.13f - 0.035355f,  -0.035355f, 45.0f },   // RIGHT
};

// ---------------------------------------------------------------------------
// MID sensor Z0/Z3 calibration.
// User reports Z0 reads ~5-8% closer and projects too far right;
//              Z3 reads ~5-8% closer and projects too far left.
// Apply per-zone distance scale and angle bias for MID (sensor index 1).
// ---------------------------------------------------------------------------
static constexpr float MID_Z0_DIST_SCALE = 1.06f;   // Z0 reads short → stretch
static constexpr float MID_Z3_DIST_SCALE = 1.06f;   // Z3 reads short → stretch
static constexpr float MID_Z0_ANGLE_BIAS = -2.0f * (float)M_PI / 180.0f;  // nudge inward (less right)
static constexpr float MID_Z3_ANGLE_BIAS = +2.0f * (float)M_PI / 180.0f;  // nudge inward (less left)

static void project_zones(const uint16_t z[4], const uint8_t nv[4],
                          uint8_t sensorIdx, const Pose& robot) {
  const SensMount& m = gMount[sensorIdx];
  const float fovRad = m.fovDeg * (float)M_PI / 180.0f;
  const float step   = fovRad / 4.0f;
  const float start  = m.baseAngleRad - fovRad / 2.0f + step / 2.0f;

  // UI lateral-mirror correction
  const float offYcorr = -m.offY;

  const float cosY = cosf(robot.yaw);
  const float sinY = sinf(robot.yaw);
  const float baseX = robot.x + m.offX * cosY - offYcorr * sinY;
  const float baseY = robot.y + m.offX * sinY + offYcorr * cosY;

  for (uint8_t i = 0; i < 4; i++) {
    if (z[i] == 0 || z[i] >= 8000) continue;
    if (nv[i] < 1) continue;     // require at least 1 valid cell

    float dist_m = (float)z[i] / 1000.0f;
    float zoneAngle = start + i * step;

    // MID per-zone calibration
    if (sensorIdx == 1) {
      if (i == 0) { dist_m *= MID_Z0_DIST_SCALE; zoneAngle += MID_Z0_ANGLE_BIAS; }
      if (i == 3) { dist_m *= MID_Z3_DIST_SCALE; zoneAngle += MID_Z3_ANGLE_BIAS; }
    }

    float angCorr = -zoneAngle;   // UI correction: negate sensor angle

    float ox = baseX + dist_m * cosf(robot.yaw + angCorr);
    float oy = baseY + dist_m * sinf(robot.yaw + angCorr);
    add_point(ox, oy);
  }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
void obs_map_init() {
  obs_map_clear();
}

void obs_map_clear() {
  gIsolatedN = 0;
  gVertexN   = 0;
  gSegmentN  = 0;
}

void obs_map_feed(const Pose& robot) {
  if (robot.stamp_ms == 0) return;

  // Snapshot zone4 data
  uint16_t zL[4], zM[4], zR[4];
  uint8_t  nL[4], nM[4], nR[4];
  noInterrupts();
  for (uint8_t i = 0; i < 4; i++) {
    zL[i] = gSensL_z4.z[i];    nL[i] = gSensL_z4.nValid[i];
    zM[i] = gSensM_z4.z[i];    nM[i] = gSensM_z4.nValid[i];
    zR[i] = gSensR_z4.z[i];    nR[i] = gSensR_z4.nValid[i];
  }
  interrupts();

  // Project each sensor's zones to world coordinates
  project_zones(zL, nL, 0, robot);
  project_zones(zM, nM, 1, robot);
  project_zones(zR, nR, 2, robot);

  // Periodically merge isolated points into segments
  static uint8_t mergeTick = 0;
  if (++mergeTick >= 5) {   // every 5 feeds (~750 ms)
    mergeTick = 0;
    merge_isolated_pass();
  }
}

uint16_t obs_map_isolated_count() { return gIsolatedN; }
uint16_t obs_map_segment_count()  { return gSegmentN; }
uint16_t obs_map_vertex_count()   { return gVertexN; }

// ---------------------------------------------------------------------------
// JSON serialisation: {"iso":[[x,y],...], "seg":[[[x,y],...], ...]}
// ---------------------------------------------------------------------------
size_t obs_map_json(char* buf, size_t cap) {
  size_t idx = 0;
  auto ap = [&](const char* fmt, ...) -> bool {
    if (idx >= cap) return false;
    va_list a;
    va_start(a, fmt);
    int w = vsnprintf(buf + idx, cap - idx, fmt, a);
    va_end(a);
    if (w < 0) return false;
    if ((size_t)w >= cap - idx) { idx = cap - 1; buf[idx] = 0; return false; }
    idx += (size_t)w;
    return true;
  };

  ap("{\"iso\":[");
  for (uint16_t i = 0; i < gIsolatedN; i++) {
    if (i > 0) ap(",");
    ap("[%.3f,%.3f]", (double)gIsolated[i].x, (double)gIsolated[i].y);
  }
  ap("],\"seg\":[");
  for (uint16_t s = 0; s < gSegmentN; s++) {
    if (s > 0) ap(",");
    ap("[");
    for (uint16_t v = 0; v < gSegments[s].count; v++) {
      if (v > 0) ap(",");
      uint16_t vi = gSegments[s].start + v;
      if (vi < gVertexN) {
        ap("[%.3f,%.3f]", (double)gVertices[vi].x, (double)gVertices[vi].y);
      }
    }
    ap("]");
  }
  ap("]}");
  return idx;
}
