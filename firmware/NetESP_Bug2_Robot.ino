#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <cstdio>
#include <math.h>
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nav_core.h"
#include "bug2_core.h"
#include "obstacle_map.h"

#include "globals.h"
#include "movement_core.h"
#include "udp_pose.h"
#include "Sensors.h"
#include "ServoControl.h"
#include "MotorControl.h"
#include "UiServer.h"
#include "DbgLog.h"
#include "wifi_manager.h"

#define ENABLE_LOOP_MONITOR 1
#define ENABLE_HEAP_STACK_CHECKS 1
#define ENABLE_TELEMETRY 1

// ===== WiFi config =====
// Public showcase: configure Wi-Fi before deployment.

const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";
const char* MDNS_NAME = "esp32";

WebServer server(80);
unsigned long lastBlink = 0;
bool ledOn = false;
static TaskHandle_t gLoopMonTask = nullptr;

// Curvature calibration test for steering-servo replacement.
//
// Use /curvetest?angle=95&pwm=125&ms=1200 to drive both motors at the same PWM
// while holding one direct MG90S steering command. The test is non-blocking so
// UDP pose continues to update; /curvetest_status reports measured yaw/distance.
static bool     gCurveTestActive = false;
static bool     gCurveTestHasResult = false;
static uint32_t gCurveTestStartMs = 0;
static uint32_t gCurveTestDurationMs = 0;
static int      gCurveTestAngle = SERVO_CENTER;
static int      gCurveTestPwm = 125;
static char     gCurveTestDir = 'F';
static Pose     gCurveTestStartPose = {0,0,0,0};
static Pose     gCurveTestEndPose = {0,0,0,0};

static float wrap_pi_local(float a){
  while (a >  M_PI) a -= 2.0f * M_PI;
  while (a < -M_PI) a += 2.0f * M_PI;
  return a;
}

static void curve_test_finish(const char* reason){
  if (gCurveTestActive){
    gCurveTestEndPose = snapshotPose(gRobotLatest);
    gCurveTestActive = false;
    gCurveTestHasResult = true;
  }
  motor_stop();
  movement_servo_only(SERVO_CENTER, reason ? reason : "curve_test_done");
}

static void handle_root(){
  server.send(200, "text/plain", "OK\nUse /, /start, /stop, /state\n");
}

static void handle_start(){
  reset_stop_cause();
  noInterrupts(); gStarted = true; interrupts();
  dbg_push(EV_HTTP_START, 0,0,0);
  server.send(200, "application/json", "{\"ok\":true,\"started\":true}");
}
static void handle_stop(){
  if (gCurveTestActive) curve_test_finish("curve_test_stop");
  dbg_push(EV_MARK, 501, 0, 0);
  record_stop_cause(501, (uint32_t)bug2_state(), dbg_hash16("http_stop"), "http_stop");
  dbg_push(EV_MARK, 520, 1, 0);
  noInterrupts(); gStarted = false; interrupts();
  dbg_push(EV_HTTP_STOP, 0,0,0);
  server.send(200, "application/json", "{\"ok\":true,\"started\":false}");
}

// Heap-safe JSON using snprintf
static void handle_state(){
  dbg_push(EV_MARK, 101, 0, 0);
  Pose r = snapshotPose(gRobotLatest);
  Pose t = snapshotPose(gTargetLatest);
  bool started = snapshotBool(gStarted);
  const char* reason = movement_reason();
  int angle = current_angle();

  volatile uint32_t canaryA = 0xA1B2C3D4;
  char buf[512];
  volatile uint32_t canaryB = 0xD4C3B2A1;
  int n = snprintf(buf, sizeof(buf),
    "{"
      "\"started\":%s,"
      "\"reason\":\"%s\","
      "\"angle\":%d,"
      "\"robot\":{\"x\":%.3f,\"y\":%.3f,\"yaw\":%.3f,\"ms\":%u},"
      "\"target\":{\"x\":%.3f,\"y\":%.3f,\"yaw\":%.3f,\"ms\":%u}"
    "}",
    started ? "true":"false",
    reason ? reason : "",
    angle,
    r.x, r.y, r.yaw, (unsigned)r.stamp_ms,
    t.x, t.y, t.yaw, (unsigned)t.stamp_ms
  );
  if (n < 0) n = 0;
  if (n >= (int)sizeof(buf)) n = sizeof(buf)-1;
  dbg_push(EV_MARK, 101, n, (int32_t)sizeof(buf));
  if (canaryA != 0xA1B2C3D4 || canaryB != 0xD4C3B2A1) {
    dbg_push(EV_MARK, 901, 101, 0);
  }
  server.send(200, "application/json", buf);
}

static void handle_sensreset(){
  sensors_force_reset_all();
  server.send(200, "application/json", "{\"ok\":true,\"sensreset\":true}");
}

static void handle_servo(){
  if (!server.hasArg("angle")){
    server.send(400, "application/json", "{\"ok\":false,\"err\":\"need angle\"}");
    return;
  }

  int angle = server.arg("angle").toInt();
  angle = constrain(angle, SERVO_CAL_MIN, SERVO_CAL_MAX);

  noInterrupts(); gStarted = false; interrupts();
  nav_clear();
  bug2_stop("servo_manual");
  movement_servo_calibration(angle, "servo_manual");

  char buf[200];
  int n = snprintf(buf, sizeof(buf),
                   "{\"ok\":true,\"servo_manual\":true,\"angle\":%d,\"min\":%d,\"center\":%d,\"max\":%d,\"cal_min\":%d,\"cal_max\":%d}",
                   angle, SERVO_MIN, SERVO_CENTER, SERVO_MAX, SERVO_CAL_MIN, SERVO_CAL_MAX);
  if (n < 0) n = 0;
  if (n >= (int)sizeof(buf)) n = sizeof(buf) - 1;
  server.send(200, "application/json", buf);
}

static void handle_motortest(){
  int pwm = 150;
  int ms = 700;
  char dir = 'F';

  if (server.hasArg("pwm")) pwm = server.arg("pwm").toInt();
  if (server.hasArg("ms")) ms = server.arg("ms").toInt();
  if (server.hasArg("dir")){
    String d = server.arg("dir");
    if (d.length() > 0 && (d[0] == 'B' || d[0] == 'b')) dir = 'B';
  }

  pwm = constrain(pwm, 0, 255);
  ms = constrain(ms, 50, 2000);

  noInterrupts(); gStarted = false; interrupts();
  nav_clear();
  bug2_stop("motor_test");
  movement_servo_only(SERVO_CENTER, "motor_test");

  motor_apply(dir, pwm, dir, pwm);
  delay(ms);
  motor_stop();

  char buf[160];
  int n = snprintf(buf, sizeof(buf),
                   "{\"ok\":true,\"motor_test\":true,\"dir\":\"%c\",\"pwm\":%d,\"ms\":%d}",
                   dir, pwm, ms);
  if (n < 0) n = 0;
  if (n >= (int)sizeof(buf)) n = sizeof(buf) - 1;
  server.send(200, "application/json", buf);
}

static void append_curve_result_json(char* buf, size_t len, bool startedNow){
  Pose s = gCurveTestStartPose;
  Pose e = gCurveTestEndPose;
  if (gCurveTestActive) e = snapshotPose(gRobotLatest);

  const float dx = e.x - s.x;
  const float dy = e.y - s.y;
  const float dist = sqrtf(dx*dx + dy*dy);
  const float dyaw = wrap_pi_local(e.yaw - s.yaw);
  const float durS = (gCurveTestDurationMs > 0) ? ((float)gCurveTestDurationMs * 0.001f) : 0.0f;
  const float yawRate = (durS > 0.0f) ? (dyaw / durS) : 0.0f;
  const float radius = (fabsf(dyaw) > 0.03f) ? (dist / fabsf(dyaw)) : 0.0f;

  int n = snprintf(buf, len,
    "{"
      "\"ok\":true,"
      "\"active\":%s,"
      "\"has_result\":%s,"
      "\"started\":%s,"
      "\"angle\":%d,"
      "\"center\":%d,"
      "\"min\":%d,"
      "\"max\":%d,"
      "\"cal_min\":%d,"
      "\"cal_max\":%d,"
      "\"pwm\":%d,"
      "\"dir\":\"%c\","
      "\"ms\":%u,"
      "\"start\":{\"x\":%.3f,\"y\":%.3f,\"yaw\":%.3f,\"stamp\":%u},"
      "\"end\":{\"x\":%.3f,\"y\":%.3f,\"yaw\":%.3f,\"stamp\":%u},"
      "\"dx\":%.3f,"
      "\"dy\":%.3f,"
      "\"dist\":%.3f,"
      "\"dyaw\":%.3f,"
      "\"yaw_rate\":%.3f,"
      "\"radius\":%.3f"
    "}",
    gCurveTestActive ? "true" : "false",
    gCurveTestHasResult ? "true" : "false",
    startedNow ? "true" : "false",
    gCurveTestAngle,
    SERVO_CENTER,
    SERVO_MIN,
    SERVO_MAX,
    SERVO_CAL_MIN,
    SERVO_CAL_MAX,
    gCurveTestPwm,
    gCurveTestDir,
    (unsigned)gCurveTestDurationMs,
    s.x, s.y, s.yaw, (unsigned)s.stamp_ms,
    e.x, e.y, e.yaw, (unsigned)e.stamp_ms,
    dx, dy, dist, dyaw, yawRate, radius
  );
  if (n < 0) n = 0;
  if (n >= (int)len) n = (int)len - 1;
  buf[n] = 0;
}

static void handle_curvetest_status(){
  char buf[640];
  append_curve_result_json(buf, sizeof(buf), false);
  server.send(200, "application/json", buf);
}

static void handle_curvetest_stop(){
  curve_test_finish("curve_test_stop");
  char buf[640];
  append_curve_result_json(buf, sizeof(buf), false);
  server.send(200, "application/json", buf);
}

static void handle_curvetest(){
  if (gCurveTestActive){
    char buf[640];
    append_curve_result_json(buf, sizeof(buf), false);
    server.send(409, "application/json", buf);
    return;
  }
  if (!server.hasArg("angle")){
    server.send(400, "application/json", "{\"ok\":false,\"err\":\"need angle\"}");
    return;
  }

  int angle = server.arg("angle").toInt();
  int pwm = 125;
  int ms = 1200;
  char dir = 'F';
  if (server.hasArg("pwm")) pwm = server.arg("pwm").toInt();
  if (server.hasArg("ms")) ms = server.arg("ms").toInt();
  if (server.hasArg("dir")){
    String d = server.arg("dir");
    if (d.length() > 0 && (d[0] == 'B' || d[0] == 'b')) dir = 'B';
  }

  angle = constrain(angle, SERVO_CAL_MIN, SERVO_CAL_MAX);
  pwm = constrain(pwm, 90, 180);
  ms = constrain(ms, 300, 3000);

  noInterrupts(); gStarted = false; interrupts();
  nav_clear();
  bug2_stop("curve_test");

  gCurveTestAngle = angle;
  gCurveTestPwm = pwm;
  gCurveTestDir = dir;
  gCurveTestDurationMs = (uint32_t)ms;
  gCurveTestStartMs = millis();
  gCurveTestStartPose = snapshotPose(gRobotLatest);
  gCurveTestEndPose = gCurveTestStartPose;
  gCurveTestHasResult = false;
  gCurveTestActive = true;

  movement_servo_calibration(angle, "curve_test");
  motor_apply(dir, pwm, dir, pwm);

  char buf[640];
  append_curve_result_json(buf, sizeof(buf), true);
  server.send(200, "application/json", buf);
}

// Heap-safe /dbg (no events for now to avoid String fragmentation)
static void handle_dbg(){
  dbg_push(EV_MARK, 102, 0, 0);
  bool wifi = (WiFi.status() == WL_CONNECTED);
  int rssi = wifi ? WiFi.RSSI() : 0;

  bool started = snapshotBool(gStarted);
  const char* reason = movement_reason();
  int servoDeg = current_angle();

  int pwmA, pwmB; char dirA, dirB;
  motor_get_last(dirA, pwmA, dirB, pwmB);

  // CRITICAL: Protect 16-bit reads from interrupt corruption
  uint16_t dL, dM, dR;
  noInterrupts();
  dL = gSensL_mm;
  dM = gSensM_mm;
  dR = gSensR_mm;
  interrupts();

  char dLTag[20], dMTag[20], dRTag[20];
  sensor_format_mm(dL, dLTag, sizeof(dLTag));
  sensor_format_mm(dM, dMTag, sizeof(dMTag));
  sensor_format_mm(dR, dRTag, sizeof(dRTag));

  uint32_t lastT = sensors_last_timeout_ms();
  uint32_t lastG = sensors_last_good_ms();

  uint32_t corrSrc, corrFrame;
  uint16_t corrL, corrM, corrR;
  uint32_t loopCanaryTrips, loopCanaryMs;
  noInterrupts();
  corrSrc = gSensCorruptSrc;
  corrFrame = gSensCorruptLastFrame;
  corrL = gSensCorruptLastRawL;
  corrM = gSensCorruptLastRawM;
  corrR = gSensCorruptLastRawR;
  loopCanaryTrips = gLoopCanaryTripCount;
  loopCanaryMs = gLoopCanaryLastMs;
  interrupts();

  uint32_t stackLoop = (uint32_t)uxTaskGetStackHighWaterMark(nullptr);
  uint32_t stackMon = gLoopMonTask ? (uint32_t)uxTaskGetStackHighWaterMark(gLoopMonTask) : 0;

  volatile uint32_t canaryA = 0xA1B2C3D4;
  char buf[1040];
  volatile uint32_t canaryB = 0xD4C3B2A1;
  int n = snprintf(buf, sizeof(buf),
    "{"
      "\"ms\":%u,"
      "\"wifi\":%s,"
      "\"rssi\":%d,"
      "\"started\":%s,"
      "\"reason\":\"%s\","
      "\"servo\":%d,"
      "\"dirA\":\"%c\",\"pwmA\":%d,"
      "\"dirB\":\"%c\",\"pwmB\":%d,"
      "\"dL\":%u,\"dM\":%u,\"dR\":%u,"
      "\"dL_tag\":\"%s\",\"dM_tag\":\"%s\",\"dR_tag\":\"%s\","
      "\"sens_last_timeout_ms\":%u,"
      "\"sens_last_good_ms\":%u,"
      "\"sens_corrupt_src\":%u,"
      "\"sens_corrupt_frame\":%u,"
      "\"sens_corrupt_rawL\":%u,"
      "\"sens_corrupt_rawM\":%u,"
      "\"sens_corrupt_rawR\":%u,"
      "\"loop_canary_trips\":%u,"
      "\"loop_canary_last_ms\":%u,"
      "\"stack_loop_hw\":%u,"
      "\"stack_mon_hw\":%u"
    "}",
    (unsigned)millis(),
    wifi ? "true":"false",
    rssi,
    started ? "true":"false",
    reason ? reason : "",
    servoDeg,
    dirA, pwmA,
    dirB, pwmB,
    (unsigned)dL, (unsigned)dM, (unsigned)dR,
    dLTag, dMTag, dRTag,
    (unsigned)lastT,
    (unsigned)lastG,
    (unsigned)corrSrc,
    (unsigned)corrFrame,
    (unsigned)corrL,
    (unsigned)corrM,
    (unsigned)corrR,
    (unsigned)loopCanaryTrips,
    (unsigned)loopCanaryMs,
    (unsigned)stackLoop,
    (unsigned)stackMon
  );

  if (n < 0) n = 0;
  if (n >= (int)sizeof(buf)) n = sizeof(buf)-1;
  dbg_push(EV_MARK, 102, n, (int32_t)sizeof(buf));
  if (canaryA != 0xA1B2C3D4 || canaryB != 0xD4C3B2A1) {
    dbg_push(EV_MARK, 901, 102, 0);
  }
  server.send(200, "application/json", buf);
}

// Return recent dbg events (includes EV_POSE_RX markers)
static void handle_dbg_events(){
  dbg_push(EV_MARK, 103, 0, 0);
  int maxEvents = 80;
  if (server.hasArg("n")){
    maxEvents = server.arg("n").toInt();
    if (maxEvents < 1) maxEvents = 1;
    if (maxEvents > 120) maxEvents = 120;
  }

  volatile uint32_t canaryA = 0xA1B2C3D4;
  char buf[4096];
  volatile uint32_t canaryB = 0xD4C3B2A1;
  size_t n = 0;
  n += snprintf(buf + n, sizeof(buf) - n, "{");
  n += dbg_append_events_json_buf(buf + n, sizeof(buf) - n, maxEvents);
  if (n < sizeof(buf) - 1) {
    n += snprintf(buf + n, sizeof(buf) - n, "}");
  }
  if (n >= sizeof(buf)) n = sizeof(buf) - 1;
  buf[n] = 0;
  dbg_push(EV_MARK, 103, maxEvents, (int32_t)n);
  if (canaryA != 0xA1B2C3D4 || canaryB != 0xD4C3B2A1) {
    dbg_push(EV_MARK, 901, 103, 0);
  }
  server.send(200, "application/json", buf);
}

static void handle_sens_test_start(){
  uint32_t ms = 3000;
  if(server.hasArg("ms")){
    ms = (uint32_t) server.arg("ms").toInt();
    if(ms < 200) ms = 200;
    if(ms > 8000) ms = 8000;
  }
  sensors_test_window_start(ms);
  dbg_push(EV_SENS_TEST_START, (int32_t)ms, 0, 0);
  server.send(200, "application/json", "{\"ok\":true,\"sens_test_start\":true}");
}

static void handle_nav(){
  if (!server.hasArg("x") || !server.hasArg("y")){
    server.send(400, "application/json", "{\"ok\":false,\"err\":\"need x,y\"}");
    return;
  }
  float x = server.arg("x").toFloat();
  float y = server.arg("y").toFloat();

  reset_stop_cause();
  bug2_stop("nav_override");
  nav_start(x, y);
  noInterrupts(); gStarted = true; interrupts();

  server.send(200, "application/json", "{\"ok\":true,\"nav\":true}");
}

static void handle_navstop(){
  nav_stop();
  server.send(200, "application/json", "{\"ok\":true,\"navstop\":true}");
}

static void handle_bug2(){
  if (!server.hasArg("x") || !server.hasArg("y")){
    server.send(400, "application/json", "{\"ok\":false,\"err\":\"need x,y\"}");
    return;
  }
  float x = server.arg("x").toFloat();
  float y = server.arg("y").toFloat();

  reset_stop_cause();
  nav_clear();
  bug2_set_mode(BUG2_MODE_FULL);
  bug2_start(x, y);
  noInterrupts(); gStarted = true; interrupts();

  server.send(200, "application/json", "{\"ok\":true,\"bug2\":true}");
}

static void handle_bug2wf(){
  if (!server.hasArg("x") || !server.hasArg("y")){
    server.send(400, "application/json", "{\"ok\":false,\"err\":\"need x,y\"}");
    return;
  }
  float x = server.arg("x").toFloat();
  float y = server.arg("y").toFloat();

  reset_stop_cause();
  nav_clear();
  bug2_set_mode(BUG2_MODE_WF_ONLY);
  bug2_start(x, y);
  noInterrupts(); gStarted = true; interrupts();

  server.send(200, "application/json", "{\"ok\":true,\"bug2wf\":true}");
}

static void handle_bug2stop(){
  dbg_push(EV_MARK, 503, 0, 0);
  record_stop_cause(503, (uint32_t)bug2_state(), dbg_hash16("bug2_stop_http"), "bug2_stop_http");
  bug2_stop("bug2_stop_http");
  dbg_push(EV_MARK, 520, 2, 0);
  noInterrupts(); gStarted = false; interrupts();
  server.send(200, "application/json", "{\"ok\":true,\"bug2stop\":true}");
}

static void handle_stop_cause(){
  uint32_t tag, ms, b2, rh;
  char reason[24];
  uint32_t ltag, lms, lb2, lrh;
  char lreason[24];
  noInterrupts();
  tag = gStopCauseTag;
  ms = gLastStopMs;
  b2 = gLastStopBug2;
  rh = gLastStopReasonHash;
  strncpy(reason, gLastStopReason, sizeof(reason) - 1);
  reason[sizeof(reason) - 1] = '\0';
  ltag = gLatchedStopTag;
  lms = gLatchedStopMs;
  lb2 = gLatchedStopBug2;
  lrh = gLatchedStopReasonHash;
  strncpy(lreason, gLatchedStopReason, sizeof(lreason) - 1);
  lreason[sizeof(lreason) - 1] = '\0';
  interrupts();

  if (tag == 0 && gEverStarted && !snapshotBool(gStarted)){
    const char* r = movement_reason();
    uint16_t h = dbg_hash16(r);
    record_stop_cause(509, (uint32_t)bug2_state(), (uint32_t)h, r);
    noInterrupts();
    tag = gStopCauseTag;
    ms = gLastStopMs;
    b2 = gLastStopBug2;
    rh = gLastStopReasonHash;
    strncpy(reason, gLastStopReason, sizeof(reason) - 1);
    reason[sizeof(reason) - 1] = '\0';
    interrupts();
  }

  if (tag == 0 && ltag != 0){
    tag = ltag;
    ms = lms;
    b2 = lb2;
    rh = lrh;
    strncpy(reason, lreason, sizeof(reason) - 1);
    reason[sizeof(reason) - 1] = '\0';
  }

  char buf[256];
  int n = snprintf(buf, sizeof(buf),
    "{"
      "\"tag\":%u,"
      "\"ms\":%u,"
      "\"b2\":%u,"
      "\"reason_hash\":%u,"
      "\"reason\":\"%s\""
    "}",
    (unsigned)tag,
    (unsigned)ms,
    (unsigned)b2,
    (unsigned)rh,
    reason
  );
  if (n < 0) n = 0;
  if (n >= (int)sizeof(buf)) n = sizeof(buf) - 1;
  server.send(200, "application/json", buf);
}

static void handle_bug2_decision_log(){
  // Return decision log in JSON format
  char buf[768];
  const char* log = bug2_decision_log();
  int n = snprintf(buf, sizeof(buf),
    "{"
      "\"ok\":true,"
      "\"log\":\"%s\""
    "}",
    log ? log : ""
  );
  if (n < 0) n = 0;
  if (n >= (int)sizeof(buf)) n = sizeof(buf)-1;
  server.send(200, "application/json", buf);
}

static void handle_wf_tune(){
  Bug2WfTune t;
  bug2_wf_tune_get(t);
  bool changed = false;

  if (server.hasArg("reset")){
    Bug2WfTune d{};
    d.target_mm = 260.0f;
    d.dist_k = 3.2f;
    d.yaw_k = 2.2f;
    d.bear_k = 0.12f;
    d.opp_k = 3.5f;
    d.front_k = 4.0f;
    d.zone_emergency_mm = 150;
    d.zone_close_mm = 300;
    d.zone_mid_mm = 520;
    d.deadend_rev_ms = 450;
    d.deadend_turn_ms = 300;
    d.mode_dwell_ms = 320;
    d.fblk_enter_conf = 0.65f;
    d.fblk_exit_conf = 0.45f;
    d.fblk_enter_mm = 300;
    d.fblk_exit_mm = 380;
    d.dend_enter_front_conf = 0.62f;
    d.dend_enter_opp_conf = 0.48f;
    d.dend_enter_follow_conf = 0.25f;
    d.dend_exit_front_conf = 0.46f;
    d.dend_exit_opp_conf = 0.34f;
    bug2_wf_tune_set(d);
    changed = true;
  } else {
    if (server.hasArg("target")) { t.target_mm = server.arg("target").toFloat(); changed = true; }
    if (server.hasArg("distk"))  { t.dist_k = server.arg("distk").toFloat(); changed = true; }
    if (server.hasArg("yawk"))   { t.yaw_k = server.arg("yawk").toFloat(); changed = true; }
    if (server.hasArg("beark"))  { t.bear_k = server.arg("beark").toFloat(); changed = true; }
    if (server.hasArg("oppk"))   { t.opp_k = server.arg("oppk").toFloat(); changed = true; }
    if (server.hasArg("frontk")) { t.front_k = server.arg("frontk").toFloat(); changed = true; }
    if (server.hasArg("zemerg")) { t.zone_emergency_mm = (uint16_t)server.arg("zemerg").toInt(); changed = true; }
    if (server.hasArg("zclose")) { t.zone_close_mm = (uint16_t)server.arg("zclose").toInt(); changed = true; }
    if (server.hasArg("zmid"))   { t.zone_mid_mm = (uint16_t)server.arg("zmid").toInt(); changed = true; }
    if (server.hasArg("drev"))   { t.deadend_rev_ms = (uint16_t)server.arg("drev").toInt(); changed = true; }
    if (server.hasArg("dturn"))  { t.deadend_turn_ms = (uint16_t)server.arg("dturn").toInt(); changed = true; }
    if (server.hasArg("dwell"))  { t.mode_dwell_ms = (uint16_t)server.arg("dwell").toInt(); changed = true; }
    if (server.hasArg("fblk_ec")) { t.fblk_enter_conf = server.arg("fblk_ec").toFloat(); changed = true; }
    if (server.hasArg("fblk_xc")) { t.fblk_exit_conf = server.arg("fblk_xc").toFloat(); changed = true; }
    if (server.hasArg("fblk_emm")) { t.fblk_enter_mm = (uint16_t)server.arg("fblk_emm").toInt(); changed = true; }
    if (server.hasArg("fblk_xmm")) { t.fblk_exit_mm = (uint16_t)server.arg("fblk_xmm").toInt(); changed = true; }
    if (server.hasArg("dend_efc")) { t.dend_enter_front_conf = server.arg("dend_efc").toFloat(); changed = true; }
    if (server.hasArg("dend_eoc")) { t.dend_enter_opp_conf = server.arg("dend_eoc").toFloat(); changed = true; }
    if (server.hasArg("dend_efol")) { t.dend_enter_follow_conf = server.arg("dend_efol").toFloat(); changed = true; }
    if (server.hasArg("dend_xfc")) { t.dend_exit_front_conf = server.arg("dend_xfc").toFloat(); changed = true; }
    if (server.hasArg("dend_xoc")) { t.dend_exit_opp_conf = server.arg("dend_xoc").toFloat(); changed = true; }
    if (changed) bug2_wf_tune_set(t);
  }

  bug2_wf_tune_get(t);
  char buf[860];
  int n = snprintf(buf, sizeof(buf),
    "{"
      "\"ok\":true,"
      "\"changed\":%s,"
      "\"target\":%.2f,"
      "\"distk\":%.3f,"
      "\"yawk\":%.3f,"
      "\"beark\":%.3f,"
      "\"oppk\":%.3f,"
      "\"frontk\":%.3f,"
      "\"zemerg\":%u,"
      "\"zclose\":%u,"
      "\"zmid\":%u,"
      "\"drev\":%u,"
      "\"dturn\":%u,"
      "\"dwell\":%u,"
      "\"fblk_ec\":%.3f,"
      "\"fblk_xc\":%.3f,"
      "\"fblk_emm\":%u,"
      "\"fblk_xmm\":%u,"
      "\"dend_efc\":%.3f,"
      "\"dend_eoc\":%.3f,"
      "\"dend_efol\":%.3f,"
      "\"dend_xfc\":%.3f,"
      "\"dend_xoc\":%.3f"
    "}",
    changed ? "true" : "false",
    t.target_mm, t.dist_k, t.yaw_k, t.bear_k, t.opp_k, t.front_k,
    (unsigned)t.zone_emergency_mm, (unsigned)t.zone_close_mm, (unsigned)t.zone_mid_mm,
    (unsigned)t.deadend_rev_ms, (unsigned)t.deadend_turn_ms,
    (unsigned)t.mode_dwell_ms,
    t.fblk_enter_conf, t.fblk_exit_conf,
    (unsigned)t.fblk_enter_mm, (unsigned)t.fblk_exit_mm,
    t.dend_enter_front_conf, t.dend_enter_opp_conf, t.dend_enter_follow_conf,
    t.dend_exit_front_conf, t.dend_exit_opp_conf
  );
  if (n < 0) n = 0;
  if (n >= (int)sizeof(buf)) n = sizeof(buf)-1;
  server.send(200, "application/json", buf);
}

static void start_server(){
  server.on("/", handle_root);
  server.on("/start", handle_start);
  server.on("/stop", handle_stop);
  server.on("/state", handle_state);
  server.on("/stop_cause", handle_stop_cause);

  server.on("/sensreset", handle_sensreset);
  server.on("/servo", handle_servo);
  server.on("/motortest", handle_motortest);
  server.on("/curvetest", handle_curvetest);
  server.on("/curvetest_status", handle_curvetest_status);
  server.on("/curvetest_stop", handle_curvetest_stop);
  server.on("/dbg", handle_dbg);
  server.on("/dbg_events", handle_dbg_events);
  server.on("/sens_test_start", handle_sens_test_start);

  server.on("/nav", handle_nav);
  server.on("/navstop", handle_navstop);

  server.on("/bug2", handle_bug2);
  server.on("/bug2wf", handle_bug2wf);
  server.on("/bug2stop", handle_bug2stop);
  server.on("/bug2_log", handle_bug2_decision_log);
  server.on("/wf_tune", handle_wf_tune);
  wifi_manager_register_routes(server);

  server.begin();
  Serial.println("[HTTP] server started on :80");
}

#if ENABLE_LOOP_MONITOR
static void loop_monitor_task(void*){
  while (true){
    if (gLoopCanaryA != 0xC0FFEEA5 || gLoopCanaryB != 0x5AA55AA5){
      noInterrupts();
      gLoopCanaryTripCount++;
      gLoopCanaryLastMs = millis();
      interrupts();
      dbg_push(EV_MEM_CANARY, (gLoopCanaryA != 0xC0FFEEA5) ? 1 : 2, (int32_t)gLoopCanaryTripCount, 0);
    }

    const uint64_t nowUs = esp_timer_get_time();
    const uint64_t lastUs = gLoopBeatUs;
    if (lastUs != 0 && (nowUs - lastUs) > 500000){ // >500ms without loop beat
      gLoopStallCount++;
      gLoopStallLastMs = (uint32_t)(nowUs / 1000ULL);
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}
#endif

static uint32_t fnv1a32(const uint8_t* data, size_t len){
  uint32_t h = 2166136261u;
  for (size_t i = 0; i < len; ++i){
    h ^= data[i];
    h *= 16777619u;
  }
  return h;
}

static inline bool sanitize_sensor_mm(uint16_t &d){
  if (d == SENSOR_OFFLINE_RAW) return false;
  if (d > 8191){
    d = SENSOR_INVALID;
    return true;
  }
  return false;
}

void setup(){
  Serial.begin(115200);
  pinMode(2, OUTPUT); // onboard LED

  servo_setup(SERVO_PIN);
  motor_init();
  movement_setup();
  nav_setup();
  bug2_setup();
  obs_map_init();

  sensors_init();
  dbg_init();

  wifi_manager_init(WIFI_SSID, WIFI_PASS);
  wifi_manager_begin_with_fallback();

  // Loop stall monitor task pinned to the other core
#if ENABLE_LOOP_MONITOR
  xTaskCreatePinnedToCore(loop_monitor_task, "loop_mon", 2048, nullptr, 1, &gLoopMonTask, 0);
#endif
  if (MDNS.begin(MDNS_NAME)){
    Serial.printf("[mDNS] http://%s.local/\n", MDNS_NAME);
  } else {
    Serial.println("[mDNS] failed");
  }

  start_server();
  udp_setup();
  ui_init(server);
  Serial.println("[BOOT] UI dashboard ready at /ui and WS on :81");
}

// Servo command throttle: prevents jitter from high-rate updates / noisy loop
static uint32_t gLastServoApplyMs = 0;
static int gLastServoAngle = -1;
static void apply_servo_throttled(int angle){
  uint32_t now = millis();
  if (gLastServoAngle < 0){
    gLastServoAngle = angle;
    gLastServoApplyMs = now;
    movement_apply_angle(angle, "servo_init", 0.0f);
    return;
  }
  if (angle == gLastServoAngle) return;
  if (now - gLastServoApplyMs < 40) return; // max 25Hz servo updates
  gLastServoApplyMs = now;
  gLastServoAngle = angle;
  movement_apply_angle(angle, "servo_throttle", 0.0f);
}

void loop(){
#if ENABLE_LOOP_MONITOR
  if (gLoopCanaryA != 0xC0FFEEA5 || gLoopCanaryB != 0x5AA55AA5){
    noInterrupts();
    gLoopCanaryTripCount++;
    gLoopCanaryLastMs = millis();
    interrupts();
    dbg_push(EV_MEM_CANARY, (gLoopCanaryA != 0xC0FFEEA5) ? 1 : 2, (int32_t)gLoopCanaryTripCount, 0);
  }

  gLoopBeatUs = esp_timer_get_time();
#endif
  server.handleClient();
  wifi_manager_loop();
  ui_loop();
  sensors_service();

  yield();

  // LED heartbeat
  unsigned long now = millis();
  if (now - lastBlink > 500){
    lastBlink = now;
    ledOn = !ledOn;
    digitalWrite(2, ledOn ? HIGH : LOW);
  }

  // Poll sensors at 100 ms to match the configured 10 Hz ranging rate.
  // 150 ms under-samples fresh frames and increases reaction latency.
  static constexpr uint32_t SENSOR_LOOP_PERIOD_MS = 100;
  static uint32_t lastSenseMs = 0;
  if (now - lastSenseMs >= SENSOR_LOOP_PERIOD_MS) {
    lastSenseMs = now;
    uint16_t dL, dM, dR;
    sensors_read(dL, dM, dR);
    const uint16_t rawL = dL;
    const uint16_t rawM = dM;
    const uint16_t rawR = dR;
    bool sensCorrupt = false;
    sensCorrupt |= sanitize_sensor_mm(dL);
    sensCorrupt |= sanitize_sensor_mm(dM);
    sensCorrupt |= sanitize_sensor_mm(dR);
    if (sensCorrupt){
      noInterrupts();
      gSensCorruptCount++;
      gSensCorruptLastMs = millis();
      gSensCorruptSrc = 1;
      gSensCorruptLastFrame = gSensFrameId;
      gSensCorruptLastRawL = rawL;
      gSensCorruptLastRawM = rawM;
      gSensCorruptLastRawR = rawR;
      interrupts();
      const int32_t packLM = (int32_t)((uint32_t)rawL | ((uint32_t)rawM << 16));
      dbg_push(EV_SENS_CORRUPT, 1, packLM, (int32_t)rawR);
      dbg_push(EV_MARK, 508, 1, 0);
      record_stop_cause(508, (uint32_t)bug2_state(), dbg_hash16("sens_corrupt"), "sens_corrupt");
      sensors_request_reset_all(300);
      movement_stop_reason("sens_corrupt", true);
      dbg_push(EV_MARK, 520, 3, 0);
      noInterrupts(); gStarted = false; interrupts();
    }
    // CRITICAL: Protect 16-bit writes from interrupt corruption
    noInterrupts();
    gSensL_mm = dL;
    gSensM_mm = dM;
    gSensR_mm = dR;
    gSensFrameId++;
    interrupts();

    // Feed obstacle map with current pose + fresh zone4 data
    {
      Pose rObs = snapshotPose(gRobotLatest);
      obs_map_feed(rObs);
    }
  }

  // Update pose
  udp_pump();

  Pose rNow = snapshotPose(gRobotLatest);
  bool startedNow = snapshotBool(gStarted);

  if (gCurveTestActive && (millis() - gCurveTestStartMs >= gCurveTestDurationMs)){
    curve_test_finish("curve_test_done");
  }

  if (startedNow) gEverStarted = true;
  static bool lastStarted = false;

  
  if (lastStarted && !startedNow){
    const uint16_t reasonHash = dbg_hash16(movement_reason());
    uint32_t tag = gStopCauseTag;
    if (tag == 0) {
      tag = 509;
      record_stop_cause(tag, (uint32_t)bug2_state(), (uint32_t)reasonHash, movement_reason());
    }
    dbg_push(EV_MARK, (int32_t)tag, (int32_t)bug2_state(), (int32_t)reasonHash);
  }
  lastStarted = startedNow;

  // Controller selection
  if (gCurveTestActive){
    // Curvature test owns servo + motors until its duration expires above.
  }
  else if (nav_state() == NAV_ACTIVE){
    nav_tick(rNow, startedNow);
  }
  else if (bug2_state() != BUG2_IDLE && bug2_state() != BUG2_REACHED){
    uint16_t dL, dM, dR;
    uint32_t fid;
    uint32_t canA, canB;
    static uint32_t lastFid = 0;
    static uint16_t lastL = 0, lastM = 0, lastR = 0;
    static uint32_t lastCrc = 0;
    noInterrupts();
    dL = gSensL_mm;
    dM = gSensM_mm;
    dR = gSensR_mm;
    fid = gSensFrameId;
    canA = gSensCanaryA;
    canB = gSensCanaryB;
    interrupts();

    if (canA != 0xA5A55A5A || canB != 0x5A5AA5A5){
      noInterrupts();
      gSensCanaryTripCount++;
      gSensCanaryLastMs = millis();
      interrupts();
    }

    uint8_t bytes[6];
    bytes[0] = (uint8_t)(dL & 0xFF);
    bytes[1] = (uint8_t)(dL >> 8);
    bytes[2] = (uint8_t)(dM & 0xFF);
    bytes[3] = (uint8_t)(dM >> 8);
    bytes[4] = (uint8_t)(dR & 0xFF);
    bytes[5] = (uint8_t)(dR >> 8);
    uint32_t crc = fnv1a32(bytes, sizeof(bytes));

    if (fid == lastFid){
      if (dL != lastL || dM != lastM || dR != lastR){
        noInterrupts();
        gSensCorruptCount++;
        gSensCorruptLastMs = millis();
        interrupts();
      }
      if (crc != lastCrc){
        noInterrupts();
        gSensCrcTripCount++;
        gSensCrcLastMs = millis();
        interrupts();
      }
    } else {
      lastFid = fid;
      lastCrc = crc;
    }
    lastL = dL;
    lastM = dM;
    lastR = dR;

    bug2_tick(rNow, startedNow, dL, dM, dR);
  }
  else{
    movement_loop_tick();
  }

  // Telemetry at ~4Hz (reduce load)

  static uint32_t lastTelemMs = 0;
  static uint32_t lastStackLogMs = 0;
  uint32_t nowMs = millis();
#if ENABLE_TELEMETRY
  if (nowMs - lastTelemMs > 250) {
    lastTelemMs = nowMs;

    Pose r = snapshotPose(gRobotLatest);
    Pose t = snapshotPose(gTargetLatest);
    bool started = snapshotBool(gStarted);
    const char* reason = movement_reason();
    int servoDeg = current_angle();

    // Layer 4: Detect corruption at point of telemetry broadcast (decision log via EV_POSE_RX)
    if (r.x > 100.0f || r.x < -100.0f || r.y > 100.0f || r.y < -100.0f) {
      dbg_push(EV_POSE_RX,
               (int32_t)(r.x * 1000.0f),
               (int32_t)(r.y * 1000.0f),
               5);  // marker=5
    }

    int pwmA, pwmB;
    char dirA, dirB;
    motor_get_last(dirA, pwmA, dirB, pwmB);

    // CRITICAL: Protect 16-bit reads from interrupt corruption
    uint16_t dL, dM, dR;
    noInterrupts();
    dL = gSensL_mm;
    dM = gSensM_mm;
    dR = gSensR_mm;
    interrupts();

    const uint16_t rawL = dL;
    const uint16_t rawM = dM;
    const uint16_t rawR = dR;

    bool sensCorrupt = false;
    sensCorrupt |= sanitize_sensor_mm(dL);
    sensCorrupt |= sanitize_sensor_mm(dM);
    sensCorrupt |= sanitize_sensor_mm(dR);
    if (sensCorrupt){
      noInterrupts();
      gSensCorruptCount++;
      gSensCorruptLastMs = millis();
      gSensCorruptSrc = 2;
      gSensCorruptLastFrame = gSensFrameId;
      gSensCorruptLastRawL = rawL;
      gSensCorruptLastRawM = rawM;
      gSensCorruptLastRawR = rawR;
      interrupts();
      const int32_t packLM = (int32_t)((uint32_t)rawL | ((uint32_t)rawM << 16));
      dbg_push(EV_SENS_CORRUPT, 2, packLM, (int32_t)rawR);
      dbg_push(EV_MARK, 508, 2, 0);
      record_stop_cause(508, (uint32_t)bug2_state(), dbg_hash16("sens_corrupt"), "sens_corrupt");
      sensors_request_reset_all(301);
      movement_stop_reason("sens_corrupt", true);
      dbg_push(EV_MARK, 520, 4, 0);
      noInterrupts(); gStarted = false; interrupts();
    }

    ui_broadcast(r, t, started, reason,
                 servoDeg,
                 pwmA, pwmB,
                 dirA, dirB,
                 dL, dM, dR);
  }
#endif
  if (ENABLE_HEAP_STACK_CHECKS && (nowMs - lastStackLogMs > 2000)) {
    lastStackLogMs = nowMs;
    uint32_t hwLoop = (uint32_t)uxTaskGetStackHighWaterMark(nullptr);
    dbg_push(EV_STACK_WM, 0, (int32_t)hwLoop, 0);
    if (gLoopMonTask){
      uint32_t hwMon = (uint32_t)uxTaskGetStackHighWaterMark(gLoopMonTask);
      dbg_push(EV_STACK_WM, 1, (int32_t)hwMon, 0);
    }

    if (!heap_caps_check_integrity_all(true)){
      dbg_push(EV_HEAP_FAIL, 0, 0, 0);
    }
  }

  yield();
}



