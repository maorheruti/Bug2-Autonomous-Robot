// UiServer.cpp
#include "UiServer.h"
#include "UiDashboard.h"
#include "UiObsMap.h"
#include "nav_core.h"
#include "bug2_core.h"
#include "movement_core.h"
#include "Sensors.h"
#include "obstacle_map.h"
#include <WebSocketsServer.h>
#include <ESPmDNS.h>
#include "DbgLog.h"

static WebSocketsServer ws(81);
static bool wsStarted = false; // track if WS server is running
static const bool kEnableWs = true;  // Set to false to eliminate WS from build
static const bool kEnableWsBroadcast = true; // Set to false to disable periodic WS broadcasts

static const char* safe_reason(const char* s){
  if (!s) return "";
  return s;
}

void ui_init(WebServer& server){
  server.on("/ui", [&server](){
    String html = FPSTR(UI_DASHBOARD_HTML);
    server.send(200, "text/html", html);
  });

  server.on("/matrix", [&server](){
    // Snapshot matrix data + statuses and encode as compact hex JSON
    int16_t  mL[64], mM[64], mR[64];
    uint8_t  sL[64], sM[64], sR[64];
    noInterrupts();
    for(int i=0; i<64; i++){
      mL[i] = gSensL_matrix.distance[i];
      mM[i] = gSensM_matrix.distance[i];
      mR[i] = gSensR_matrix.distance[i];
      sL[i] = gSensL_matrix.status[i];
      sM[i] = gSensM_matrix.status[i];
      sR[i] = gSensR_matrix.status[i];
    }
    interrupts();
    
    char buf[1800];
    size_t idx = 0;
    idx += snprintf(buf+idx, sizeof(buf)-idx, "{\"dL_matrix\":\"");
    for(int i=0; i<64 && idx<sizeof(buf)-10; i++){
      idx += snprintf(buf+idx, sizeof(buf)-idx, "%04x", (unsigned)(mL[i] & 0xffff));
    }
    idx += snprintf(buf+idx, sizeof(buf)-idx, "\",\"dM_matrix\":\"");
    for(int i=0; i<64 && idx<sizeof(buf)-10; i++){
      idx += snprintf(buf+idx, sizeof(buf)-idx, "%04x", (unsigned)(mM[i] & 0xffff));
    }
    idx += snprintf(buf+idx, sizeof(buf)-idx, "\",\"dR_matrix\":\"");
    for(int i=0; i<64 && idx<sizeof(buf)-10; i++){
      idx += snprintf(buf+idx, sizeof(buf)-idx, "%04x", (unsigned)(mR[i] & 0xffff));
    }
    idx += snprintf(buf+idx, sizeof(buf)-idx, "\",\"sL_matrix\":\"");
    for(int i=0; i<64 && idx<sizeof(buf)-10; i++){
      idx += snprintf(buf+idx, sizeof(buf)-idx, "%02x", (unsigned)sL[i]);
    }
    idx += snprintf(buf+idx, sizeof(buf)-idx, "\",\"sM_matrix\":\"");
    for(int i=0; i<64 && idx<sizeof(buf)-10; i++){
      idx += snprintf(buf+idx, sizeof(buf)-idx, "%02x", (unsigned)sM[i]);
    }
    idx += snprintf(buf+idx, sizeof(buf)-idx, "\",\"sR_matrix\":\"");
    for(int i=0; i<64 && idx<sizeof(buf)-10; i++){
      idx += snprintf(buf+idx, sizeof(buf)-idx, "%02x", (unsigned)sR[i]);
    }
    idx += snprintf(buf+idx, sizeof(buf)-idx, "\"}");
    
    server.send(200, "application/json", buf);
  });

  // Obstacle map endpoints
  server.on("/obsmap", [&server](){
    static char obsBuf[4096];
    size_t n = obs_map_json(obsBuf, sizeof(obsBuf));
    server.send(200, "application/json", obsBuf);
  });
  server.on("/obsmap_clear", [&server](){
    obs_map_clear();
    server.send(200, "application/json", "{\"ok\":true}");
  });
  server.on("/obsmap_ui", [&server](){
    String html = FPSTR(UI_OBSMAP_HTML);
    server.send(200, "text/html", html);
  });

  if (!kEnableWs){
    Serial.println("[UI] WS disabled (elimination test)");
    return;
  }

  ws.begin();
  ws.onEvent([](uint8_t num, WStype_t type, uint8_t * payload, size_t length){
    if (type == WStype_CONNECTED){
      IPAddress ip = ws.remoteIP(num);
      Serial.printf("[UI] WS client #%u connected from %s\n", num, ip.toString().c_str());
      return;
    }
    if (type == WStype_DISCONNECTED){
      Serial.printf("[UI] WS client #%u disconnected\n", num);
      return;
    }
    if (type != WStype_TEXT) return;

    // copy to null-terminated buffer
    char tmp[120];
    size_t n = (length < sizeof(tmp)-1) ? length : sizeof(tmp)-1;
    memcpy(tmp, payload, n);
    tmp[n] = 0;
    dbg_push(EV_MARK, 202, (int32_t)length, (int32_t)n);

    // start/stop
    if (!strncmp(tmp, "start", 5)){
      reset_stop_cause();
      noInterrupts(); gStarted = true;  interrupts(); dbg_push(EV_HTTP_START, 0,0,0);
      Serial.println("[UI] WS -> start");
      return;
    }
    if (!strncmp(tmp, "stop", 4)){
      dbg_push(EV_MARK, 502, 0, 0);
      record_stop_cause(502, (uint32_t)bug2_state(), dbg_hash16("ws_stop"), "ws_stop");
      dbg_push(EV_MARK, 520, 5, 0);
      noInterrupts(); gStarted = false; interrupts(); dbg_push(EV_HTTP_STOP, 0,0,0);
      Serial.println("[UI] WS -> stop");
      return;
    }

    // navstop
    if (!strncmp(tmp, "navstop", 7)){
      nav_stop();
      Serial.println("[UI] WS -> navstop");
      return;
    }

    // nav x y
    if (!strncmp(tmp, "nav ", 4)){
      float x=0, y=0;
      if (sscanf(tmp, "nav %f %f", &x, &y) == 2){
        reset_stop_cause();
        bug2_stop("nav_override_ws");   // avoid conflict
        nav_start(x, y);
        noInterrupts(); gStarted = true; interrupts();
        Serial.printf("[UI] WS -> nav %.3f %.3f\n", x, y);
      } else {
        Serial.println("[UI] WS nav parse fail");
      }
      return;
    }

    // bug2stop
    if (!strncmp(tmp, "bug2stop", 8)){
      dbg_push(EV_MARK, 504, 0, 0);
      record_stop_cause(504, (uint32_t)bug2_state(), dbg_hash16("bug2_stop_ws"), "bug2_stop_ws");
      bug2_stop("bug2_stop_ws");
      dbg_push(EV_MARK, 520, 6, 0);
      noInterrupts(); gStarted = false; interrupts();
      Serial.println("[UI] WS -> bug2stop");
      return;
    }

    // bug2 x y
    if (!strncmp(tmp, "bug2 ", 5)){
      float x=0, y=0;
      if (sscanf(tmp, "bug2 %f %f", &x, &y) == 2){
        Serial.printf("[UI] WS -> bug2 %.3f %.3f (raw payload: '%s')\n", x, y, tmp);
        reset_stop_cause();
        nav_clear();                   // avoid conflict
        bug2_set_mode(BUG2_MODE_FULL);
        bug2_start(x, y);
        noInterrupts(); gStarted = true; interrupts();
      } else {
        Serial.printf("[UI] WS bug2 parse fail (payload: '%s')\n", tmp);
      }
      return;
    }

    // bug2wf x y
    if (!strncmp(tmp, "bug2wf ", 7)){
      float x=0, y=0;
      if (sscanf(tmp, "bug2wf %f %f", &x, &y) == 2){
        Serial.printf("[UI] WS -> bug2wf %.3f %.3f (raw payload: '%s')\n", x, y, tmp);
        reset_stop_cause();
        nav_clear();                   // avoid conflict
        bug2_set_mode(BUG2_MODE_WF_ONLY);
        bug2_start(x, y);
        noInterrupts(); gStarted = true; interrupts();
      } else {
        Serial.printf("[UI] WS bug2wf parse fail (payload: '%s')\n", tmp);
      }
      return;
    }
  });

  wsStarted = true;
  Serial.println("[UI] Dashboard at /ui, WebSocket on :81");
}

void ui_loop(){
  if (wsStarted && kEnableWs){
    ws.loop();
  }
}

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
                  uint16_t dR)
{
  if (!wsStarted || !kEnableWs || !kEnableWsBroadcast) return;

  dbg_push(EV_MARK, 201, 0, 0);

  char b2dbgBuf[384];
  const char* b2dbgSrc = bug2_dbg_line();
  if (!b2dbgSrc) b2dbgSrc = "";
  if (b2dbgSrc[0] == '\0') {
    const char* moveDbg = movement_dbg_line();
    if (moveDbg) b2dbgSrc = moveDbg;
  }
  strncpy(b2dbgBuf, b2dbgSrc, sizeof(b2dbgBuf) - 1);
  b2dbgBuf[sizeof(b2dbgBuf) - 1] = '\0';

  // If a loop stall was detected, surface it directly to UI even if bug2_tick is not running
  const uint32_t nowMs = millis();
  if (gLoopStallLastMs != 0 && (nowMs - gLoopStallLastMs) < 10000) {
    snprintf(b2dbgBuf, sizeof(b2dbgBuf), "LOOP_STALL cnt=%lu",
             (unsigned long)gLoopStallCount);
  }

  char dLTag[20], dMTag[20], dRTag[20];
  sensor_format_mm(dL, dLTag, sizeof(dLTag));
  sensor_format_mm(dM, dMTag, sizeof(dMTag));
  sensor_format_mm(dR, dRTag, sizeof(dRTag));

  uint16_t zL[4], zM[4], zR[4];
  uint8_t  nL[4], nM[4], nR[4];
  noInterrupts();
  zL[0] = gSensL_z4.z[0]; zL[1] = gSensL_z4.z[1]; zL[2] = gSensL_z4.z[2]; zL[3] = gSensL_z4.z[3];
  zM[0] = gSensM_z4.z[0]; zM[1] = gSensM_z4.z[1]; zM[2] = gSensM_z4.z[2]; zM[3] = gSensM_z4.z[3];
  zR[0] = gSensR_z4.z[0]; zR[1] = gSensR_z4.z[1]; zR[2] = gSensR_z4.z[2]; zR[3] = gSensR_z4.z[3];
  nL[0] = gSensL_z4.nValid[0]; nL[1] = gSensL_z4.nValid[1]; nL[2] = gSensL_z4.nValid[2]; nL[3] = gSensL_z4.nValid[3];
  nM[0] = gSensM_z4.nValid[0]; nM[1] = gSensM_z4.nValid[1]; nM[2] = gSensM_z4.nValid[2]; nM[3] = gSensM_z4.nValid[3];
  nR[0] = gSensR_z4.nValid[0]; nR[1] = gSensR_z4.nValid[1]; nR[2] = gSensR_z4.nValid[2]; nR[3] = gSensR_z4.nValid[3];
  interrupts();

  volatile uint32_t canaryA = 0xA1B2C3D4;
  char buf[1400];
  volatile uint32_t canaryB = 0xD4C3B2A1;
  size_t idx = 0;
  auto appendf = [&](const char* fmt, ...) -> bool {
    if (idx >= sizeof(buf)) return false;
    va_list ap;
    va_start(ap, fmt);
    int w = vsnprintf(buf + idx, sizeof(buf) - idx, fmt, ap);
    va_end(ap);
    if (w < 0) return false;
    if ((size_t)w >= sizeof(buf) - idx) { idx = sizeof(buf) - 1; buf[idx] = 0; return false; }
    idx += (size_t)w;
    return true;
  };

  const char* reasonSafe = safe_reason(reason);
  const char* dbgSafe = safe_reason(b2dbgBuf);

  bool ok = true;
  ok &= appendf("{\"started\":%s,", started ? "true" : "false");
  ok &= appendf("\"reason\":\"%s\",", reasonSafe);
  ok &= appendf("\"b2\":%d,\"b2dbg\":\"%s\",", (int)bug2_state(), dbgSafe);
  ok &= appendf("\"servo\":%d,", servoDeg);
  ok &= appendf("\"pwmA\":%d,\"pwmB\":%d,", pwmA, pwmB);
  ok &= appendf("\"dirA\":\"%c\",\"dirB\":\"%c\",", dirA, dirB);
  ok &= appendf("\"dL\":[%u,%u,%u,%u],\"dM\":[%u,%u,%u,%u],\"dR\":[%u,%u,%u,%u],",
                (unsigned)zL[0], (unsigned)zL[1], (unsigned)zL[2], (unsigned)zL[3],
                (unsigned)zM[0], (unsigned)zM[1], (unsigned)zM[2], (unsigned)zM[3],
                (unsigned)zR[0], (unsigned)zR[1], (unsigned)zR[2], (unsigned)zR[3]);
  ok &= appendf("\"nL\":[%u,%u,%u,%u],\"nM\":[%u,%u,%u,%u],\"nR\":[%u,%u,%u,%u],",
                (unsigned)nL[0], (unsigned)nL[1], (unsigned)nL[2], (unsigned)nL[3],
                (unsigned)nM[0], (unsigned)nM[1], (unsigned)nM[2], (unsigned)nM[3],
                (unsigned)nR[0], (unsigned)nR[1], (unsigned)nR[2], (unsigned)nR[3]);
  ok &= appendf("\"dL_mm\":%u,\"dM_mm\":%u,\"dR_mm\":%u,", (unsigned)dL, (unsigned)dM, (unsigned)dR);
  ok &= appendf("\"dL_tag\":\"%s\",\"dM_tag\":\"%s\",\"dR_tag\":\"%s\",", dLTag, dMTag, dRTag);
  ok &= appendf("\"robot\":{\"x\":%.3f,\"y\":%.3f,\"yaw\":%.3f,\"ms\":%lu},",
               (double)robot.x, (double)robot.y, (double)robot.yaw, (unsigned long)robot.stamp_ms);
  ok &= appendf("\"target\":{\"x\":%.3f,\"y\":%.3f,\"yaw\":%.3f,\"ms\":%lu}}",
               (double)target.x, (double)target.y, (double)target.yaw, (unsigned long)target.stamp_ms);

  if (!ok){
    idx = 0;
    appendf("{\"started\":%s,\"reason\":\"%s\",\"b2\":%d}",
            started ? "true" : "false", reasonSafe, (int)bug2_state());
  }

  dbg_push(EV_MARK, 201, (int32_t)idx, (int32_t)sizeof(buf));
  if (canaryA != 0xA1B2C3D4 || canaryB != 0xD4C3B2A1) {
    dbg_push(EV_MARK, 901, 201, 0);
  }
  ws.broadcastTXT(buf);
}


