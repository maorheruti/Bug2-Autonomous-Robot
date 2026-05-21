#include "DbgLog.h"

static const int DBG_CAP = 400;   // ~60s at 7Hz if you log sensors every frame
static DbgEvent  gBuf[DBG_CAP];
static volatile uint16_t gHead = 0;   // next write
static volatile uint16_t gCount = 0;  // how many valid

void dbg_init(){
  noInterrupts();
  gHead = 0;
  gCount = 0;
  interrupts();
  dbg_push(EV_BOOT, 0, 0, 0);
}

// Tiny stable hash (good enough for identifying strings like reason or filename)
uint16_t dbg_hash16(const char* s){
  uint16_t h = 0xA1B2;
  if(!s) return h;
  while(*s){
    h = (uint16_t)((h << 5) ^ (h >> 11) ^ (uint8_t)(*s));
    s++;
  }
  return h;
}

void dbg_push(DbgEventType t, int32_t a, int32_t b, int32_t c){
  uint32_t ms = millis();

  noInterrupts();
  uint16_t idx = gHead;
  gBuf[idx].ms   = ms;
  gBuf[idx].type = (uint8_t)t;
  gBuf[idx].a    = a;
  gBuf[idx].b    = b;
  gBuf[idx].c    = c;

  gHead = (uint16_t)((gHead + 1) % DBG_CAP);
  if(gCount < DBG_CAP) gCount++;
  interrupts();
}

void dbg_append_events_json(String& out, int maxEvents){
  if (maxEvents <= 0) maxEvents = 1;
  if (maxEvents > DBG_CAP) maxEvents = DBG_CAP;

  // snapshot head/count
  uint16_t head, count;
  noInterrupts();
  head = gHead;
  count = gCount;
  interrupts();

  int n = (count < (uint16_t)maxEvents) ? count : maxEvents;

  // oldest index among the last n
  int start = (int)head - n;
  while(start < 0) start += DBG_CAP;

  out += "\"events\":[";
  for(int i=0;i<n;i++){
    int idx = (start + i) % DBG_CAP;
    const DbgEvent& e = gBuf[idx];
    out += "{";
    out += "\"ms\":"; out += e.ms; out += ",";
    out += "\"t\":";  out += (int)e.type; out += ",";
    out += "\"a\":";  out += e.a; out += ",";
    out += "\"b\":";  out += e.b; out += ",";
    out += "\"c\":";  out += e.c;
    out += "}";
    if(i != n-1) out += ",";
  }
  out += "]";
}

size_t dbg_append_events_json_buf(char* out, size_t outSize, int maxEvents){
  if (!out || outSize == 0) return 0;
  if (maxEvents <= 0) maxEvents = 1;
  if (maxEvents > DBG_CAP) maxEvents = DBG_CAP;

  uint16_t head, count;
  noInterrupts();
  head = gHead;
  count = gCount;
  interrupts();

  int n = (count < (uint16_t)maxEvents) ? count : maxEvents;
  int start = (int)head - n;
  while (start < 0) start += DBG_CAP;

  size_t idx = 0;
  int w = snprintf(out + idx, outSize - idx, "\"events\":[");
  if (w < 0) w = 0;
  if ((size_t)w >= outSize - idx) { out[outSize - 1] = 0; return outSize - 1; }
  idx += (size_t)w;

  for (int i = 0; i < n; i++){
    int bi = (start + i) % DBG_CAP;
    const DbgEvent& e = gBuf[bi];
    w = snprintf(out + idx, outSize - idx,
                 "%s{\"ms\":%lu,\"t\":%u,\"a\":%ld,\"b\":%ld,\"c\":%ld}",
                 (i == 0) ? "" : ",",
                 (unsigned long)e.ms,
                 (unsigned)e.type,
                 (long)e.a,
                 (long)e.b,
                 (long)e.c);
    if (w < 0) w = 0;
    if ((size_t)w >= outSize - idx) { out[outSize - 1] = 0; return outSize - 1; }
    idx += (size_t)w;
  }

  w = snprintf(out + idx, outSize - idx, "]");
  if (w < 0) w = 0;
  if ((size_t)w >= outSize - idx) { out[outSize - 1] = 0; return outSize - 1; }
  idx += (size_t)w;
  if (idx < outSize) out[idx] = 0;
  return idx;
}


