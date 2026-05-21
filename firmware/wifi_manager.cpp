#include "wifi_manager.h"

#include <WiFi.h>
#include <Preferences.h>
#include <string.h>

static Preferences gWifiPrefs;

static constexpr uint8_t WIFI_MAX_PROFILES = 8;

struct WifiProfile {
  char ssid[33];
  char pass[65];
};

static WifiProfile gProfiles[WIFI_MAX_PROFILES];
static uint8_t gProfileCount = 0;
static uint8_t gPreferredIdx = 0;

static char gDefaultSsid[33] = {0};
static char gDefaultPass[65] = {0};

static bool gApplyPending = false;
static char gPendingSsid[33] = {0};
static char gPendingPass[65] = {0};

static void copy_str_trunc(char* dst, size_t cap, const char* src) {
  if (!dst || cap == 0) return;
  if (!src) {
    dst[0] = '\0';
    return;
  }
  strncpy(dst, src, cap - 1);
  dst[cap - 1] = '\0';
}

static bool is_valid_ssid(const char* ssid) {
  return ssid && ssid[0] && strlen(ssid) <= 32;
}

static void make_key(char* out, size_t cap, char prefix, uint8_t idx) {
  snprintf(out, cap, "%c%u", prefix, (unsigned)idx);
}

static int profile_find(const char* ssid) {
  if (!is_valid_ssid(ssid)) return -1;
  for (uint8_t i = 0; i < gProfileCount; i++) {
    if (strcmp(gProfiles[i].ssid, ssid) == 0) return (int)i;
  }
  return -1;
}

static void profile_compact() {
  uint8_t w = 0;
  for (uint8_t r = 0; r < gProfileCount; r++) {
    if (!is_valid_ssid(gProfiles[r].ssid)) continue;
    if (w != r) gProfiles[w] = gProfiles[r];
    w++;
  }
  gProfileCount = w;
  if (gProfileCount == 0) gPreferredIdx = 0;
  else if (gPreferredIdx >= gProfileCount) gPreferredIdx = 0;
}

static void profile_upsert(const char* ssid, const char* pass) {
  if (!is_valid_ssid(ssid)) return;
  int idx = profile_find(ssid);
  if (idx >= 0) {
    copy_str_trunc(gProfiles[idx].pass, sizeof(gProfiles[idx].pass), pass ? pass : "");
    return;
  }

  if (gProfileCount < WIFI_MAX_PROFILES) {
    idx = (int)gProfileCount++;
  } else {
    // Replace last slot when full.
    idx = WIFI_MAX_PROFILES - 1;
  }
  copy_str_trunc(gProfiles[idx].ssid, sizeof(gProfiles[idx].ssid), ssid);
  copy_str_trunc(gProfiles[idx].pass, sizeof(gProfiles[idx].pass), pass ? pass : "");
}

static void profile_remove(const char* ssid) {
  int idx = profile_find(ssid);
  if (idx < 0) return;
  for (uint8_t i = (uint8_t)idx; i + 1 < gProfileCount; i++) {
    gProfiles[i] = gProfiles[i + 1];
  }
  if (gProfileCount > 0) gProfileCount--;
  if (gProfileCount == 0) {
    gPreferredIdx = 0;
  } else if (gPreferredIdx >= gProfileCount) {
    gPreferredIdx = 0;
  } else if ((uint8_t)idx < gPreferredIdx) {
    gPreferredIdx--;
  }
}

static void profile_set_preferred(const char* ssid) {
  int idx = profile_find(ssid);
  if (idx >= 0) gPreferredIdx = (uint8_t)idx;
}

static bool prefs_begin(bool readOnly) {
  return gWifiPrefs.begin("wifi", readOnly);
}

static void prefs_end() {
  gWifiPrefs.end();
}

static void profiles_load() {
  gProfileCount = 0;
  gPreferredIdx = 0;
  memset(gProfiles, 0, sizeof(gProfiles));

  if (prefs_begin(true)) {
    uint8_t cnt = gWifiPrefs.getUChar("cnt", 0);
    uint8_t pref = gWifiPrefs.getUChar("pref", 0);
    if (cnt > WIFI_MAX_PROFILES) cnt = WIFI_MAX_PROFILES;

    for (uint8_t i = 0; i < cnt; i++) {
      char kS[6], kP[6];
      make_key(kS, sizeof(kS), 's', i);
      make_key(kP, sizeof(kP), 'p', i);
      String ssid = gWifiPrefs.getString(kS, "");
      String pass = gWifiPrefs.getString(kP, "");
      if (!ssid.length() || ssid.length() > 32) continue;
      copy_str_trunc(gProfiles[gProfileCount].ssid, sizeof(gProfiles[gProfileCount].ssid), ssid.c_str());
      copy_str_trunc(gProfiles[gProfileCount].pass, sizeof(gProfiles[gProfileCount].pass), pass.c_str());
      gProfileCount++;
    }

    // Backward compatibility: migrate legacy single-profile keys.
    if (gProfileCount == 0) {
      String legacySsid = gWifiPrefs.getString("ssid", "");
      String legacyPass = gWifiPrefs.getString("pass", "");
      if (legacySsid.length() > 0 && legacySsid.length() <= 32) {
        copy_str_trunc(gProfiles[0].ssid, sizeof(gProfiles[0].ssid), legacySsid.c_str());
        copy_str_trunc(gProfiles[0].pass, sizeof(gProfiles[0].pass), legacyPass.c_str());
        gProfileCount = 1;
        gPreferredIdx = 0;
      }
    }

    gPreferredIdx = (gProfileCount == 0) ? 0 : (pref < gProfileCount ? pref : 0);
    prefs_end();
  }

  profile_compact();
  if (gProfileCount == 0 && is_valid_ssid(gDefaultSsid)) {
    profile_upsert(gDefaultSsid, gDefaultPass);
    gPreferredIdx = 0;
  }
}

static void profiles_save() {
  if (!prefs_begin(false)) return;
  gWifiPrefs.clear();
  gWifiPrefs.putUChar("cnt", gProfileCount);
  gWifiPrefs.putUChar("pref", gPreferredIdx);
  for (uint8_t i = 0; i < gProfileCount; i++) {
    char kS[6], kP[6];
    make_key(kS, sizeof(kS), 's', i);
    make_key(kP, sizeof(kP), 'p', i);
    gWifiPrefs.putString(kS, gProfiles[i].ssid);
    gWifiPrefs.putString(kP, gProfiles[i].pass);
  }
  prefs_end();
}

static bool wifi_connect_sta(const char* ssid, const char* pass, uint32_t timeoutMs) {
  if (!is_valid_ssid(ssid)) return false;
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass ? pass : "");
  Serial.printf("[WiFi] Connecting to %s ...\n", ssid);
  const uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < timeoutMs) {
    delay(250);
    Serial.print(".");
    yield();
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WiFi] Connected");
    Serial.print("[WiFi] IP: "); Serial.println(WiFi.localIP());
    return true;
  }
  Serial.println("\n[WiFi] STA connect failed");
  return false;
}

static void wifi_start_ap_fallback() {
  Serial.println("[WiFi] Starting AP fallback");
  WiFi.mode(WIFI_AP);
  WiFi.softAP("NetESP-Setup");
  Serial.print("[WiFi] AP IP: "); Serial.println(WiFi.softAPIP());
}

struct ScanCandidate {
  uint8_t profIdx;
  int32_t rssi;
};

static uint8_t wifi_scan_saved_candidates(ScanCandidate* out, uint8_t cap) {
  if (!out || cap == 0 || gProfileCount == 0) return 0;

  int n = WiFi.scanNetworks(false, true);
  if (n < 0) return 0;

  bool matched[WIFI_MAX_PROFILES];
  int32_t bestRssi[WIFI_MAX_PROFILES];
  for (uint8_t i = 0; i < WIFI_MAX_PROFILES; i++) {
    matched[i] = false;
    bestRssi[i] = -9999;
  }

  for (int si = 0; si < n; si++) {
    String ssid = WiFi.SSID(si);
    if (!ssid.length()) continue;
    int32_t rssi = WiFi.RSSI(si);

    for (uint8_t pi = 0; pi < gProfileCount; pi++) {
      if (ssid == gProfiles[pi].ssid) {
        if (!matched[pi] || rssi > bestRssi[pi]) {
          matched[pi] = true;
          bestRssi[pi] = rssi;
        }
      }
    }
  }
  WiFi.scanDelete();

  uint8_t cnt = 0;
  for (uint8_t pi = 0; pi < gProfileCount && cnt < cap; pi++) {
    if (!matched[pi]) continue;
    out[cnt].profIdx = pi;
    out[cnt].rssi = bestRssi[pi];
    cnt++;
  }

  // Sort by RSSI (strongest first)
  for (uint8_t i = 0; i < cnt; i++) {
    for (uint8_t j = i + 1; j < cnt; j++) {
      if (out[j].rssi > out[i].rssi) {
        ScanCandidate t = out[i];
        out[i] = out[j];
        out[j] = t;
      }
    }
  }

  return cnt;
}

static bool wifi_connect_visible_saved() {
  if (gProfileCount == 0) return false;

  ScanCandidate cand[WIFI_MAX_PROFILES];
  uint8_t cnt = wifi_scan_saved_candidates(cand, WIFI_MAX_PROFILES);
  if (cnt == 0) {
    Serial.println("[WiFi] No saved SSID visible");
    return false;
  }

  for (uint8_t i = 0; i < cnt; i++) {
    uint8_t pi = cand[i].profIdx;
    const WifiProfile& p = gProfiles[pi];
    Serial.printf("[WiFi] Try saved '%s' RSSI=%ld dBm\n", p.ssid, (long)cand[i].rssi);
    if (wifi_connect_sta(p.ssid, p.pass, 8000)) {
      gPreferredIdx = pi;
      profiles_save();
      return true;
    }
  }
  return false;
}

static void json_escape(const char* src, char* dst, size_t cap) {
  if (!dst || cap == 0) return;
  size_t w = 0;
  if (!src) src = "";
  for (size_t i = 0; src[i] && w + 2 < cap; i++) {
    char c = src[i];
    if (c == '"' || c == '\\') {
      if (w + 2 >= cap) break;
      dst[w++] = '\\';
      dst[w++] = c;
    } else if ((unsigned char)c < 0x20) {
      continue;
    } else {
      dst[w++] = c;
    }
  }
  dst[w] = '\0';
}

static void handle_wifi_info(WebServer& server) {
  const bool connected = (WiFi.status() == WL_CONNECTED);
  const bool ap = (WiFi.getMode() & WIFI_AP) != 0;
  IPAddress ip = connected ? WiFi.localIP() : WiFi.softAPIP();
  const char* prefSsid = (gProfileCount > 0 && gPreferredIdx < gProfileCount) ? gProfiles[gPreferredIdx].ssid : "";

  char escSsid[80];
  json_escape(prefSsid, escSsid, sizeof(escSsid));
  char buf[320];
  int n = snprintf(buf, sizeof(buf),
                   "{"
                   "\"ok\":true,"
                   "\"ssid\":\"%s\","
                   "\"connected\":%s,"
                   "\"ap\":%s,"
                   "\"profiles\":%u,"
                   "\"ip\":\"%u.%u.%u.%u\""
                   "}",
                   escSsid,
                   connected ? "true" : "false",
                   ap ? "true" : "false",
                   (unsigned)gProfileCount,
                   (unsigned)ip[0], (unsigned)ip[1], (unsigned)ip[2], (unsigned)ip[3]);
  if (n < 0) n = 0;
  if (n >= (int)sizeof(buf)) n = sizeof(buf) - 1;
  server.send(200, "application/json", buf);
}

static void handle_wifi_profiles(WebServer& server) {
  char buf[1200];
  size_t idx = 0;
  auto appendf = [&](const char* fmt, ...) -> bool {
    if (idx >= sizeof(buf)) return false;
    va_list ap;
    va_start(ap, fmt);
    int w = vsnprintf(buf + idx, sizeof(buf) - idx, fmt, ap);
    va_end(ap);
    if (w < 0) return false;
    if ((size_t)w >= sizeof(buf) - idx) {
      idx = sizeof(buf) - 1;
      buf[idx] = '\0';
      return false;
    }
    idx += (size_t)w;
    return true;
  };

  appendf("{\"ok\":true,\"preferred\":\"");
  char escPref[80];
  const char* prefSsid = (gProfileCount > 0 && gPreferredIdx < gProfileCount) ? gProfiles[gPreferredIdx].ssid : "";
  json_escape(prefSsid, escPref, sizeof(escPref));
  appendf("%s\",\"items\":[", escPref);
  for (uint8_t i = 0; i < gProfileCount; i++) {
    if (i) appendf(",");
    char esc[80];
    json_escape(gProfiles[i].ssid, esc, sizeof(esc));
    appendf("{\"ssid\":\"%s\",\"preferred\":%s}", esc, (i == gPreferredIdx) ? "true" : "false");
  }
  appendf("]}");
  server.send(200, "application/json", buf);
}

static void schedule_apply(const char* ssid, const char* pass) {
  copy_str_trunc(gPendingSsid, sizeof(gPendingSsid), ssid);
  copy_str_trunc(gPendingPass, sizeof(gPendingPass), pass ? pass : "");
  gApplyPending = true;
}

static void handle_wifi_set(WebServer& server) {
  if (!server.hasArg("ssid")) {
    server.send(400, "application/json", "{\"ok\":false,\"err\":\"need ssid\"}");
    return;
  }
  String ssid = server.arg("ssid");
  String pass = server.hasArg("pass") ? server.arg("pass") : "";
  ssid.trim();
  if (ssid.length() < 1 || ssid.length() > 32) {
    server.send(400, "application/json", "{\"ok\":false,\"err\":\"ssid len 1..32\"}");
    return;
  }
  if (pass.length() > 64) {
    server.send(400, "application/json", "{\"ok\":false,\"err\":\"pass len <=64\"}");
    return;
  }

  profile_upsert(ssid.c_str(), pass.c_str());
  profile_set_preferred(ssid.c_str());
  profiles_save();
  schedule_apply(ssid.c_str(), pass.c_str());
  server.send(200, "application/json", "{\"ok\":true,\"scheduled\":true,\"msg\":\"reconnecting\"}");
}

static void handle_wifi_use(WebServer& server) {
  if (!server.hasArg("ssid")) {
    server.send(400, "application/json", "{\"ok\":false,\"err\":\"need ssid\"}");
    return;
  }
  String ssid = server.arg("ssid");
  ssid.trim();
  int idx = profile_find(ssid.c_str());
  if (idx < 0) {
    server.send(404, "application/json", "{\"ok\":false,\"err\":\"not found\"}");
    return;
  }
  gPreferredIdx = (uint8_t)idx;
  profiles_save();
  schedule_apply(gProfiles[gPreferredIdx].ssid, gProfiles[gPreferredIdx].pass);
  server.send(200, "application/json", "{\"ok\":true,\"scheduled\":true}");
}

static void handle_wifi_forget(WebServer& server) {
  if (!server.hasArg("ssid")) {
    server.send(400, "application/json", "{\"ok\":false,\"err\":\"need ssid\"}");
    return;
  }
  String ssid = server.arg("ssid");
  ssid.trim();
  profile_remove(ssid.c_str());
  profiles_save();
  server.send(200, "application/json", "{\"ok\":true}");
}

static void handle_wifi_scan(WebServer& server) {
  int n = WiFi.scanNetworks(false, true);
  if (n < 0) {
    server.send(500, "application/json", "{\"ok\":false,\"err\":\"scan failed\"}");
    return;
  }

  static const int MAX_SCAN = 40;
  String ssids[MAX_SCAN];
  int32_t rssis[MAX_SCAN];
  int32_t encs[MAX_SCAN];
  int cnt = 0;

  for (int i = 0; i < n; i++) {
    String s = WiFi.SSID(i);
    if (!s.length()) continue;
    int32_t r = WiFi.RSSI(i);
    int32_t e = (int32_t)WiFi.encryptionType(i);

    int existing = -1;
    for (int k = 0; k < cnt; k++) {
      if (ssids[k] == s) { existing = k; break; }
    }
    if (existing >= 0) {
      if (r > rssis[existing]) {
        rssis[existing] = r;
        encs[existing] = e;
      }
      continue;
    }
    if (cnt < MAX_SCAN) {
      ssids[cnt] = s;
      rssis[cnt] = r;
      encs[cnt] = e;
      cnt++;
    }
  }
  WiFi.scanDelete();

  char buf[2400];
  size_t idx = 0;
  auto appendf = [&](const char* fmt, ...) -> bool {
    if (idx >= sizeof(buf)) return false;
    va_list ap;
    va_start(ap, fmt);
    int w = vsnprintf(buf + idx, sizeof(buf) - idx, fmt, ap);
    va_end(ap);
    if (w < 0) return false;
    if ((size_t)w >= sizeof(buf) - idx) {
      idx = sizeof(buf) - 1;
      buf[idx] = '\0';
      return false;
    }
    idx += (size_t)w;
    return true;
  };

  appendf("{\"ok\":true,\"items\":[");
  for (int i = 0; i < cnt; i++) {
    if (i) appendf(",");
    char esc[80];
    json_escape(ssids[i].c_str(), esc, sizeof(esc));
    appendf("{\"ssid\":\"%s\",\"rssi\":%ld,\"enc\":%ld}",
            esc, (long)rssis[i], (long)encs[i]);
  }
  appendf("]}");
  server.send(200, "application/json", buf);
}

void wifi_manager_init(const char* defaultSsid, const char* defaultPass) {
  copy_str_trunc(gDefaultSsid, sizeof(gDefaultSsid), defaultSsid);
  copy_str_trunc(gDefaultPass, sizeof(gDefaultPass), defaultPass ? defaultPass : "");
  profiles_load();
}

void wifi_manager_begin_with_fallback() {
  WiFi.disconnect(true, true);
  delay(50);
  if (!wifi_connect_visible_saved()) {
    wifi_start_ap_fallback();
  }
}

void wifi_manager_loop() {
  if (!gApplyPending) return;
  gApplyPending = false;

  profile_upsert(gPendingSsid, gPendingPass);
  profile_set_preferred(gPendingSsid);
  profiles_save();

  Serial.printf("[WiFi] Applying new config ssid='%s'\n", gPendingSsid);
  WiFi.disconnect(true, true);
  delay(100);
  if (!wifi_connect_sta(gPendingSsid, gPendingPass, 12000)) {
    // If direct connect failed, try all visible saved profiles by RSSI.
    if (!wifi_connect_visible_saved()) {
      wifi_start_ap_fallback();
    }
  }
}

void wifi_manager_register_routes(WebServer& server) {
  server.on("/wifi_info", [&server]() { handle_wifi_info(server); });
  server.on("/wifi_set", [&server]() { handle_wifi_set(server); });
  server.on("/wifi_scan", [&server]() { handle_wifi_scan(server); });
  server.on("/wifi_profiles", [&server]() { handle_wifi_profiles(server); });
  server.on("/wifi_use", [&server]() { handle_wifi_use(server); });
  server.on("/wifi_forget", [&server]() { handle_wifi_forget(server); });
}
