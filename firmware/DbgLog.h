#pragma once
#include <Arduino.h>

// Keep small + numeric (easy JSON, low RAM)
enum DbgEventType : uint8_t {
  EV_BOOT = 1,
  EV_HTTP_START,
  EV_HTTP_STOP,
  EV_REASON,          // a=servoDeg, b=hash(reason), c=0
  EV_MOTOR_STOP,      // a=line, b=fileHash16, c=0
  EV_SENS_FRAME,      // a=dL, b=dM, c=dR   (raw values you show in UI, i.e., 8191/65535/etc)
  EV_SENS_TIMEOUT,    // a=idx(0=L,1=M,2=R), b=mode(0=timeoutOccurred,1=65535 w/o flag), c=0
  EV_SENS_RESET_ALL,  // a=whyCode, b=0, c=0
  EV_SENS_TEST_START, // a=durationMs, b=0, c=0
  EV_SENS_TEST_END,   // a=pass(1)/fail(0), b=0, c=0
  EV_POSE_RX,         // a=rx_mm, b=ry_mm, c=ryaw_mrad

  // Added
  EV_I2C_RECOVER,         // a=idx(-1=global), b=whyCode, c=clockHz
  EV_SENS_RESET_ONE,      // a=idx, b=whyCode, c=0
  EV_SENS_RESET_ONE_OK,   // a=idx, b=whyCode, c=0
  EV_SENS_RESET_ONE_FAIL, // a=idx, b=whyCode, c=0
  EV_SERVO_STALL,         // a=angleDeg, b=elapsedMs, c=stallCount
  EV_SENS_CORRUPT,        // a=src(1=read,2=tele), b=pack(dL,dM), c=dR
  EV_MEM_CANARY,          // a=which(1=loopA,2=loopB), b=tripCount, c=0
  EV_STACK_WM,            // a=task(0=loop,1=mon), b=highWaterWords, c=0
  EV_HEAP_FAIL,           // a=0, b=0, c=0
  EV_MARK                 // a=tag, b=val1, c=val2
};

struct DbgEvent {
  uint32_t ms;
  uint8_t  type;
  int32_t  a, b, c;
};

// init once from setup()
void dbg_init();

// push events (fast)
void dbg_push(DbgEventType t, int32_t a=0, int32_t b=0, int32_t c=0);

// helper used by motor_stop_dbg
uint16_t dbg_hash16(const char* s);

// build JSON array of recent events (most recent last)
void dbg_append_events_json(String& out, int maxEvents);

// fixed-buffer version (returns bytes written, null-terminated)
size_t dbg_append_events_json_buf(char* out, size_t outSize, int maxEvents);


