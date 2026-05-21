#include "Sensors.h"

#include <Wire.h>
#include <vl53l5cx_api.h>

namespace vl53l7cx_ns {
extern "C" {
#include <vl53l7cx_api.h>
}
}

#define VL53L7CX_Configuration vl53l7cx_ns::VL53L7CX_Configuration
#define VL53L7CX_ResultsData vl53l7cx_ns::VL53L7CX_ResultsData

#include "globals.h"
#include "DbgLog.h"

static const int PIN_SDA = 22;
static const int PIN_SCL = 21;

static const int XSHUT_L = 16;
static const int XSHUT_M = 5;
static const int XSHUT_R = 17;

static const uint16_t ADDR_DEFAULT = 0x52;
static const uint16_t ADDR_L = 0x54;
static const uint16_t ADDR_M = 0x56;
static const uint16_t ADDR_R = 0x58;

struct SensorSlot {
  const char *name;
  int lpn_pin;
  uint16_t addr;
  bool is_7cx;
  bool active;

  VL53L5CX_Configuration dev5cx;
  VL53L5CX_ResultsData res5cx;

  VL53L7CX_Configuration dev7cx;
  VL53L7CX_ResultsData res7cx;
};

static SensorSlot sensors[] = {
  {"LEFT",  XSHUT_L, ADDR_L, false, false},
  {"MID",   XSHUT_M, ADDR_M, true,  false},
  {"RIGHT", XSHUT_R, ADDR_R, false, false}
};

static uint32_t gLastGoodMs = 0;
static uint32_t gLastTimeoutMs = 0;

static volatile bool gReqResetAll = false;
static volatile bool gReqResetOne = false;
static volatile int gReqResetIdx = -1;
static volatile bool gRecovering = false;

uint32_t sensors_last_good_ms(){ return gLastGoodMs; }
uint32_t sensors_last_timeout_ms(){ return gLastTimeoutMs; }
bool sensors_is_recovering(){ return gRecovering; }

static void clear_matrix(SensorMatrix &m){
  for (uint16_t i = 0; i < SENSOR_GRID_SIZE; i++) {
    m.distance[i] = SENSOR_INVALID;
    m.status[i] = 0;
  }
}

// Floor-rejection row masks: skip bottom N rows per sensor (row 0 = floor)
static constexpr uint8_t MASK_ROWS_LEFT  = 2;  // rows 0,1 detect floor
static constexpr uint8_t MASK_ROWS_MID   = 1;  // row 0 detects floor
static constexpr uint8_t MASK_ROWS_RIGHT = 1;  // row 0 detects floor

static void clear_zone4(SensorZone4 &z){
  for (uint8_t i = 0; i < 4; i++) { z.z[i] = SENSOR_INVALID; z.nValid[i] = 0; }
}

static uint16_t min4(const SensorZone4 &z){
  uint16_t best = SENSOR_INVALID;
  for (uint8_t i = 0; i < 4; i++) {
    if (z.z[i] > 0 && z.z[i] < best) best = z.z[i];
  }
  return best;
}

static void lpn_set(int pin, bool on) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, on ? HIGH : LOW);
}

static void lpn_all(bool on) {
  for (size_t i = 0; i < (sizeof(sensors) / sizeof(sensors[0])); i++) {
    lpn_set(sensors[i].lpn_pin, on);
  }
}

static bool init_one(SensorSlot &s) {
  lpn_set(s.lpn_pin, true);
  delay(100);

  if (s.is_7cx) {
    s.dev7cx.platform.address = ADDR_DEFAULT;

    uint8_t alive = 0;
    uint8_t st = vl53l7cx_ns::vl53l7cx_is_alive(&s.dev7cx, &alive);
    if (st != 0 || alive == 0) return false;

    st = vl53l7cx_ns::vl53l7cx_init(&s.dev7cx);
    if (st != 0) return false;

    st = vl53l7cx_ns::vl53l7cx_set_i2c_address(&s.dev7cx, s.addr);
    if (st != 0) return false;

    s.dev7cx.platform.address = s.addr;
    vl53l7cx_ns::vl53l7cx_set_resolution(&s.dev7cx, VL53L7CX_RESOLUTION_8X8);
    vl53l7cx_ns::vl53l7cx_set_ranging_frequency_hz(&s.dev7cx, 10);
    vl53l7cx_ns::vl53l7cx_set_target_order(&s.dev7cx, VL53L7CX_TARGET_ORDER_CLOSEST);
    s.active = true;
    return true;
  }

  s.dev5cx.platform.address = ADDR_DEFAULT;

  uint8_t alive = 0;
  uint8_t st = vl53l5cx_is_alive(&s.dev5cx, &alive);
  if (st != 0 || alive == 0) return false;

  st = vl53l5cx_init(&s.dev5cx);
  if (st != 0) return false;

  st = vl53l5cx_set_i2c_address(&s.dev5cx, s.addr);
  if (st != 0) return false;

  s.dev5cx.platform.address = s.addr;
  vl53l5cx_set_resolution(&s.dev5cx, VL53L5CX_RESOLUTION_8X8);
  vl53l5cx_set_ranging_frequency_hz(&s.dev5cx, 10);
  vl53l5cx_set_target_order(&s.dev5cx, VL53L5CX_TARGET_ORDER_CLOSEST);
  s.active = true;
  return true;
}

static bool start_one(SensorSlot &s) {
  if (s.is_7cx) {
    uint8_t st = vl53l7cx_ns::vl53l7cx_start_ranging(&s.dev7cx);
    if (st != 0) return false;
    return true;
  }

  uint8_t st = vl53l5cx_start_ranging(&s.dev5cx);
  if (st != 0) return false;
  return true;
}

static bool read_one(SensorSlot &s) {
  uint8_t ready = 0;
  if (s.is_7cx) {
    uint8_t st = vl53l7cx_ns::vl53l7cx_check_data_ready(&s.dev7cx, &ready);
    if (st != 0 || !ready) {
      return false;
    }
    st = vl53l7cx_ns::vl53l7cx_get_ranging_data(&s.dev7cx, &s.res7cx);
    return (st == 0);
  }

  uint8_t st = vl53l5cx_check_data_ready(&s.dev5cx, &ready);
  if (st != 0 || !ready) {
    return false;
  }
  st = vl53l5cx_get_ranging_data(&s.dev5cx, &s.res5cx);
  return (st == 0);
}

static void copy_5cx(SensorMatrix &out, const VL53L5CX_ResultsData &res) {
  for (uint16_t i = 0; i < SENSOR_GRID_SIZE; i++) {
    out.distance[i] = res.distance_mm[i];
    out.status[i] = res.target_status[i];
  }
}

static void copy_7cx(SensorMatrix &out, const VL53L7CX_ResultsData &res) {
  for (uint16_t i = 0; i < SENSOR_GRID_SIZE; i++) {
    out.distance[i] = res.distance_mm[i];
    out.status[i] = res.target_status[i];
  }
}

static void matrix_to_zone4(const SensorMatrix &m, SensorZone4 &z, uint8_t skip_rows = 0){
  clear_zone4(z);
  uint16_t second[4] = { SENSOR_INVALID, SENSOR_INVALID, SENSOR_INVALID, SENSOR_INVALID };
  for (int row = 0; row < 8; row++) {
    if (row < (int)skip_rows) continue;  // floor rejection
    for (int col = 0; col < 8; col++) {
      const int idx = row * 8 + col;
      const uint8_t st = m.status[idx];
      const int16_t d = m.distance[idx];
      // Only accept status 5 (valid) or 9 (valid, low confidence)
      if ((st != 5 && st != 9) || d <= 0 || d >= SENSOR_INVALID) continue;
      const int zone = col / 2;
      z.nValid[zone]++;
      if ((uint16_t)d < z.z[zone]) {
        second[zone] = z.z[zone];  // push previous min to 2nd
        z.z[zone] = (uint16_t)d;
      } else if ((uint16_t)d < second[zone]) {
        second[zone] = (uint16_t)d;
      }
    }
  }
  // Prefer 2nd-smallest when 2+ valid cells — rejects single-cell flicker
  for (int i = 0; i < 4; i++) {
    if (z.nValid[i] >= 2 && second[i] != SENSOR_INVALID) {
      z.z[i] = second[i];
    }
  }
}

size_t sensor_format_mm(uint16_t d, char* out, size_t outSize){
  if (!out || outSize == 0) return 0;
  const SensorKind k = sensor_kind(d);
  const char* label = "UNKNOWN";
  switch (k){
    case SENS_VALID: label = "VALID"; break;
    case SENS_CLEAR: label = "CLEAR"; break;
    case SENS_INVALID: label = "INVALID"; break;
    case SENS_CORRUPT: label = "CORRUPT"; break;
    default: break;
  }
  int n = snprintf(out, outSize, "%s(%u)", label, (unsigned)d);
  if (n < 0) n = 0;
  if ((size_t)n >= outSize) n = (int)outSize - 1;
  out[n] = 0;
  return (size_t)n;
}

void sensors_init() {
  gRecovering = true;

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);
#if defined(ARDUINO_ARCH_ESP32)
  Wire.setBufferSize(256);
#endif

  delay(250);

  auto init_start_all = [&]() -> uint8_t {
    lpn_all(false);
    delay(250);

    for (size_t i = 0; i < (sizeof(sensors) / sizeof(sensors[0])); i++) {
      sensors[i].active = false;
      if (!init_one(sensors[i])) {
        Serial.printf("[S] %s init FAIL\n", sensors[i].name);
      } else {
        Serial.printf("[S] %s init OK\n", sensors[i].name);
      }
    }

    delay(250);

    for (size_t i = 0; i < (sizeof(sensors) / sizeof(sensors[0])); i++) {
      if (!sensors[i].active) {
        Serial.printf("[S] %s skip (inactive)\n", sensors[i].name);
        continue;
      }
      if (!start_one(sensors[i])) {
        Serial.printf("[S] %s start FAIL\n", sensors[i].name);
        sensors[i].active = false;
      } else {
        Serial.printf("[S] %s ranging OK\n", sensors[i].name);
      }
    }

    uint8_t count = 0;
    for (size_t i = 0; i < 3; i++) if (sensors[i].active) count++;
    return count;
  };

  uint8_t nActive = init_start_all();
  if (nActive == 0) {
    Serial.println("[S] no active sensors, retry #2 (extra settle)");
    delay(500);
    nActive = init_start_all();
  }
  if (nActive == 0) {
    Serial.println("[S] no active sensors, retry #3 (I2C 50kHz)");
    Wire.setClock(50000);
    delay(500);
    nActive = init_start_all();
    if (nActive > 0) {
      Wire.setClock(100000);
    }
  }

  Serial.printf("[S] %u/3 sensors active\n", nActive);

  SensorMatrix m;
  clear_matrix(m);
  SensorZone4 z;
  clear_zone4(z);

  noInterrupts();
  for (uint16_t i = 0; i < SENSOR_GRID_SIZE; i++) {
    gSensL_matrix.distance[i] = m.distance[i];
    gSensM_matrix.distance[i] = m.distance[i];
    gSensR_matrix.distance[i] = m.distance[i];
    gSensL_matrix.status[i] = m.status[i];
    gSensM_matrix.status[i] = m.status[i];
    gSensR_matrix.status[i] = m.status[i];
  }
  for (uint8_t i = 0; i < 4; i++) {
    gSensL_z4.z[i] = z.z[i];
    gSensM_z4.z[i] = z.z[i];
    gSensR_z4.z[i] = z.z[i];
    gSensL_z4.nValid[i] = z.nValid[i];
    gSensM_z4.nValid[i] = z.nValid[i];
    gSensR_z4.nValid[i] = z.nValid[i];
  }
  interrupts();

  gLastGoodMs = millis();
  gLastTimeoutMs = 0;
  gRecovering = false;
}

void sensors_read_matrix(SensorMatrix &left, SensorMatrix &mid, SensorMatrix &right) {
  bool anyValid = false;

  for (size_t i = 0; i < (sizeof(sensors) / sizeof(sensors[0])); i++) {
    if (!sensors[i].active) continue;
    if (!read_one(sensors[i])) continue;

    if (i == 0) {
      copy_5cx(left, sensors[i].res5cx);
      anyValid = true;
    } else if (i == 1) {
      if (sensors[i].is_7cx) copy_7cx(mid, sensors[i].res7cx);
      else copy_5cx(mid, sensors[i].res5cx);
      anyValid = true;
    } else if (i == 2) {
      copy_5cx(right, sensors[i].res5cx);
      anyValid = true;
    }
  }

  if (anyValid) gLastGoodMs = millis();
  else gLastTimeoutMs = millis();
}

void sensors_copy_zone4(SensorZone4 &left, SensorZone4 &mid, SensorZone4 &right){
  noInterrupts();
  for (uint8_t i = 0; i < 4; i++) {
    left.z[i] = gSensL_z4.z[i];
    mid.z[i] = gSensM_z4.z[i];
    right.z[i] = gSensR_z4.z[i];
  }
  interrupts();
}

void sensors_read(uint16_t &dL, uint16_t &dM, uint16_t &dR) {
  static SensorMatrix left, mid, right;
  static bool sInited = false;
  if (!sInited) {
    clear_matrix(left);
    clear_matrix(mid);
    clear_matrix(right);
    sInited = true;
  }

  sensors_read_matrix(left, mid, right);

  SensorZone4 zL;
  SensorZone4 zM;
  SensorZone4 zR;
  matrix_to_zone4(left,  zL, MASK_ROWS_LEFT);
  matrix_to_zone4(mid,   zM, MASK_ROWS_MID);
  matrix_to_zone4(right, zR, MASK_ROWS_RIGHT);

  // --- C1: Temporal validity filter ("enter on 2nd frame / clear slow") ---
  // A zone value is output only after 2 consecutive valid frames (ENTER_TH).
  // When a zone goes invalid, the last valid value is held for up to HOLD_MAX
  // frames (~200 ms at 10 Hz), then cleared to SENSOR_INVALID.
  // prev_valid prevents emitting stale data on re-entry after a gap.
  {
    static const uint8_t ENTER_TH = 2;
    static const uint8_t HOLD_MAX = 2;

    static uint16_t prev_z4[3][4];         // last output value
    static uint8_t  prev_nv[3][4];          // last output nValid
    static uint8_t  streak[3][4];          // consecutive-valid counter
    static bool     prev_valid[3][4];      // true if prev_z4 holds a trustworthy value

    SensorZone4* zones[3] = { &zL, &zM, &zR };
    for (uint8_t s = 0; s < 3; s++) {
      for (uint8_t z = 0; z < 4; z++) {
        uint16_t cur = zones[s]->z[z];
        if (cur != SENSOR_INVALID) {
          // valid frame
          if (streak[s][z] < HOLD_MAX) streak[s][z]++;
          if (streak[s][z] >= ENTER_TH) {
            // output current value, update prev
            prev_z4[s][z]   = cur;
            prev_nv[s][z]   = zones[s]->nValid[z];
            prev_valid[s][z] = true;
          } else {
            // 1st valid frame after gap — do NOT emit
            zones[s]->z[z] = SENSOR_INVALID;
            zones[s]->nValid[z] = 0;
          }
        } else {
          // invalid frame — hold or clear
          if (streak[s][z] > 0) streak[s][z]--;
          if (streak[s][z] > 0 && prev_valid[s][z]) {
            zones[s]->z[z] = prev_z4[s][z]; // hold previous
            zones[s]->nValid[z] = prev_nv[s][z];
          } else {
            zones[s]->z[z] = SENSOR_INVALID;
            zones[s]->nValid[z] = 0;
            prev_valid[s][z] = false;
          }
        }
      }
    }
  }
  // --- end C1 filter ---

  dL = min4(zL);
  dM = min4(zM);
  dR = min4(zR);

  noInterrupts();
  for (uint16_t i = 0; i < SENSOR_GRID_SIZE; i++) {
    gSensL_matrix.distance[i] = left.distance[i];
    gSensM_matrix.distance[i] = mid.distance[i];
    gSensR_matrix.distance[i] = right.distance[i];
    gSensL_matrix.status[i] = left.status[i];
    gSensM_matrix.status[i] = mid.status[i];
    gSensR_matrix.status[i] = right.status[i];
  }
  gSensL_mm = dL;
  gSensM_mm = dM;
  gSensR_mm = dR;
  for (uint8_t i = 0; i < 4; i++) {
    gSensL_z4.z[i] = zL.z[i];
    gSensM_z4.z[i] = zM.z[i];
    gSensR_z4.z[i] = zR.z[i];
    gSensL_z4.nValid[i] = zL.nValid[i];
    gSensM_z4.nValid[i] = zM.nValid[i];
    gSensR_z4.nValid[i] = zR.nValid[i];
  }
  interrupts();
}

void sensors_request_reset_all(int whyCode){
  (void)whyCode;
  gReqResetAll = true;
}

void sensors_request_reset_one(int idx, int whyCode){
  (void)whyCode;
  gReqResetIdx = idx;
  gReqResetOne = true;
}

void sensors_force_reset_all(){
  sensors_request_reset_all(1);
}

void sensors_reset_one(int idx, int whyCode){
  sensors_request_reset_one(idx, whyCode);
}

void sensors_service(){
  if (gReqResetAll) {
    gReqResetAll = false;
    sensors_init();
    return;
  }
  if (gReqResetOne) {
    gReqResetOne = false;
    if (gReqResetIdx >= 0 && gReqResetIdx < 3) {
      sensors_init();
    }
    gReqResetIdx = -1;
  }
}

void sensors_test_window_start(uint32_t durationMs){
  (void)durationMs;
}

void sensors_test_slow_reads_start(uint32_t durationMs, uint32_t extraDelayUs){
  (void)durationMs;
  (void)extraDelayUs;
}
