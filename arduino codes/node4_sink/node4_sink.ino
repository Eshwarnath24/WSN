// ==================== NODE 4: SINK / GATEWAY ====================
// Full mesh participant: BLE + CSMA/CA MAC + TDMA + Routing
// Roles:
//   - SINK: final destination for all sensor / aggregated data (sends app ACKs)
//   - TIME BEACON MASTER: broadcasts one sync beacon per superframe (BEACON epoch)
//   - ROUTING ROOT: advertises hops_to_sink = 0 in HELLOs
//   - BACKUP SCHEDULER: takes over TDMA scheduling when N3 is silent
//   - CONTROL ORIGIN: broadcasts CTRL commands (serial / dashboard)
//   - CLOUD GATEWAY: uploads data + metrics to Firebase RTDB via Wi-Fi
//   - LOCALIZATION ENGINE: trilateration + localization error for all nodes
//
// Architecture: FreeRTOS dual-core
//   Arduino loop (core 1): ESP-NOW, MAC, routing, TDMA, BLE, metrics
//   cloudTask   (core 0): Firebase HTTPS requests (queued, non-blocking for the mesh)
// ====================================================================

#define MY_NODE_ID 4
#if __has_include("wsn_common.h")
#include "wsn_common.h"
#include "wsn_mesh.h"
#include "wsn_ble.h"
#else
#include "../common/wsn_common.h"
#include "../common/wsn_mesh.h"
#include "../common/wsn_ble.h"
#endif

#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <time.h>

// Wi-Fi / Firebase credentials live in wsn_config.h (keep it out of Git).
#if __has_include("wsn_config.h")
#include "wsn_config.h"
#else
#error "Create wsn_config.h next to node4_sink.ino defining WSN_WIFI_SSID, WSN_WIFI_PASS, WSN_FIREBASE_HOST"
#endif

const char* WIFI_SSID     = WSN_WIFI_SSID;
const char* WIFI_PASS     = WSN_WIFI_PASS;
const char* FIREBASE_HOST = WSN_FIREBASE_HOST;

#define WIFI_TIMEOUT_ATTEMPTS 20
#define FIREBASE_QUEUE_SIZE   3       // batches waiting for upload
#define FB_PAYLOAD_SIZE       3700    // one batched multi-path PATCH
#define FB_BATCH_SIZE         3600
#define FB_FLUSH_PERIOD_MS    2000    // one HTTPS request every 2 s instead of ~4 per second
#define NODE_ACTIVE_TIMEOUT_MS 8000
#define N3_SILENT_MS          10000
#define METRICS_PERIOD_MS     5000
#define DATA_PUSH_MIN_MS      1000

// ==================== FIREBASE CLIENT ====================
WiFiClientSecure fbClient;
HTTPClient fbHttp;
volatile bool wifiConnected = false;

// ==================== REMOTE DASHBOARD COMMANDS ====================
struct remote_cmd_t {
  volatile bool pending;
  char cmd[16];
  uint8_t p1;
  uint8_t p2;
};
remote_cmd_t remote_cmd = { false, "", 0, 0 };
uint32_t lastControlPoll = 0;

// ==================== FreeRTOS QUEUE ====================
struct firebase_msg_t {
  char method[8];                 // "POST", "PUT" or "PATCH"
  char path[64];                  // Firebase path
  char payload[FB_PAYLOAD_SIZE];  // JSON payload
};
QueueHandle_t firebaseQueue;

// ==================== PERFORMANCE TRACKING (at the sink) ====================
// Counted per ORIGIN on UNIQUE packets (duplicates are filtered in the mesh
// layer). Readings delivered inside an AGG count for their original source.
struct origin_stats_t {
  bool     ever_seen;
  uint16_t first_seq;
  uint16_t highest_seq;
  uint32_t unique_rx;
  uint32_t out_of_order;
  uint32_t total_delay_ms;
  uint32_t delay_samples;
  uint32_t last_delay_ms;
  uint32_t last_rx_ms;
  int      last_rssi;
  char     last_path[24];
};
origin_stats_t origin_stats[NUM_NODES + 1];
portMUX_TYPE sink_mux = portMUX_INITIALIZER_UNLOCKED;

uint32_t win_wire_bytes = 0;      // unique delivered DATA/AGG frame bytes in metrics window
uint32_t win_payload_bytes = 0;   // unique delivered application payload bytes
uint32_t last_metrics_ms = 0;
uint32_t agg_bytes_in_total = 0, agg_bytes_out_total = 0;
uint32_t agg_pkts_in_total = 0, agg_pkts_out_total = 0;
float    loc_error[NUM_NODES + 1] = { -1, -1, -1, -1, -1 };

// Node liveness for dashboard status
uint32_t last_activity_ms[NUM_NODES + 1] = {0};
bool     node_active[NUM_NODES + 1] = {false};

// Scheduler failover
bool     failoverActive = false;
bool     failN1 = true, failN2 = true;
uint16_t failoverCount = 0;

// ==================== TIMERS ====================
uint32_t lastStatusPrint = 0;
uint32_t lastBleReport = 0;
uint32_t lastPerfPush = 0;
uint32_t lastWifiCheck = 0;
uint32_t lastWatchdog = 0;
uint32_t lastFailoverCheck = 0;

// ==================== FIREBASE HELPERS ====================

void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    return;
  }
  Serial.printf("[WiFi] Connecting to: %s\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < WIFI_TIMEOUT_ATTEMPTS) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.printf("\n[WiFi] CONNECTED | IP=%s | Channel=%d (ESP-NOW mesh uses this channel)\n",
                  WiFi.localIP().toString().c_str(), WiFi.channel());
  } else {
    wifiConnected = false;
    Serial.printf("\n[WiFi] FAILED (status=%d) — mesh runs, cloud upload disabled until reconnect\n", WiFi.status());
  }
}

// ---------- Batched Firebase upload ----------
// Every update is collected into ONE multi-path PATCH ("path": value, ...)
// that is sent every FB_FLUSH_PERIOD_MS. The real hardware test showed that
// one HTTPS request per update (~4/s) overflowed the queue on the ESP32.
// List entries ("POST") get a time-ordered key (NTP time when available).
static char fb_batch[FB_BATCH_SIZE];
static int  fb_batch_len = 0;
static uint32_t fb_dropped = 0;

void fb_key(char* out, size_t n) {
  static uint8_t ctr = 0;
  time_t t = time(nullptr);
  ctr = (ctr + 1) % 100;
  if (t > 1700000000) {
    snprintf(out, n, "t%010lu%03u%02u", (unsigned long)t, (unsigned)(millis() % 1000), ctr);
  } else {
    snprintf(out, n, "m%010lu%02u", (unsigned long)millis(), ctr);  // before NTP sync
  }
}

void fb_flush() {
  if (fb_batch_len == 0) return;
  static firebase_msg_t msg;   // large: keep off the stack (loop task only)
  strcpy(msg.method, "PATCH");
  strcpy(msg.path, "/.json");
  snprintf(msg.payload, sizeof(msg.payload), "{%s}", fb_batch);
  if (xQueueSend(firebaseQueue, &msg, 0) != pdTRUE) {
    fb_dropped++;
    Serial.printf("[Firebase] Upload still busy — batch of %d B skipped (total skipped %lu)\n",
                  fb_batch_len, (unsigned long)fb_dropped);
  }
  fb_batch_len = 0;
  fb_batch[0] = '\0';
}

void fb_append(const char* key, const char* json) {
  char probe[80];
  snprintf(probe, sizeof(probe), "\"%s\":", key);
  if (strstr(fb_batch, probe)) fb_flush();                 // same path twice → send previous first
  int need = strlen(probe) + strlen(json) + 1;
  if (need >= FB_BATCH_SIZE) { Serial.printf("[Firebase] Update for %s too large — skipped\n", key); return; }
  if (fb_batch_len + need >= FB_BATCH_SIZE) fb_flush();
  fb_batch_len += snprintf(fb_batch + fb_batch_len, FB_BATCH_SIZE - fb_batch_len, "%s%s%s",
                           fb_batch_len ? "," : "", probe, json);
}

// Same interface as before: PUT path, POST to a list, or PATCH "/.json" {"a/b":…}
void queueFirebase(const char* method, const char* path, const char* payload) {
  char rel[96];
  const char* p = (path[0] == '/') ? path + 1 : path;
  strncpy(rel, p, sizeof(rel) - 1);
  rel[sizeof(rel) - 1] = '\0';
  char* dot = strstr(rel, ".json");
  if (dot) *dot = '\0';

  if (strcmp(method, "PATCH") == 0 && rel[0] == '\0') {
    // Root multi-path object: append its members
    // strip exactly ONE outer pair of braces: "{ members }" → "members"
    const char* q = payload;
    while (*q == ' ') q++;
    if (*q == '{') q++;
    int len = strlen(q);
    while (len > 0 && q[len - 1] == ' ') len--;
    if (len > 0 && q[len - 1] == '}') len--;
    // If this block's first path is already in the batch, send the batch first (no duplicate keys)
    const char* k2 = strchr(q + 1, '"');
    if (q[0] == '"' && k2) {
      char probe[80];
      snprintf(probe, sizeof(probe), "%.*s:", (int)(k2 - q + 1), q);
      if (strstr(fb_batch, probe)) fb_flush();
    }
    if (fb_batch_len + len + 2 >= FB_BATCH_SIZE) fb_flush();
    if (len + 2 >= FB_BATCH_SIZE) { Serial.println("[Firebase] Metrics block too large — skipped"); return; }
    fb_batch_len += snprintf(fb_batch + fb_batch_len, FB_BATCH_SIZE - fb_batch_len, "%s%.*s",
                             fb_batch_len ? "," : "", len, q);
    return;
  }
  if (strcmp(method, "POST") == 0) {
    char key[24];
    fb_key(key, sizeof(key));
    strncat(rel, "/", sizeof(rel) - strlen(rel) - 1);
    strncat(rel, key, sizeof(rel) - strlen(rel) - 1);
  }
  fb_append(rel, payload);
}

// Execute a Firebase HTTP request (called from cloudTask)
bool executeFirebase(const firebase_msg_t& msg) {
  if (!wifiConnected || WiFi.status() != WL_CONNECTED) return false;
  String url = String(FIREBASE_HOST) + msg.path;
  for (int attempt = 0; attempt < 2; attempt++) {
    fbHttp.begin(fbClient, url);
    fbHttp.setReuse(true);
    fbHttp.setTimeout(5000);   // TLS handshake on ESP32 can take 1–3 s
    fbHttp.addHeader("Content-Type", "application/json");
    int code = 0;
    if (strcmp(msg.method, "POST") == 0)       code = fbHttp.POST(msg.payload);
    else if (strcmp(msg.method, "PATCH") == 0) code = fbHttp.PATCH(msg.payload);
    else                                       code = fbHttp.PUT(msg.payload);
    fbHttp.end();
    if (code > 0 && code < 400) return true;
    Serial.printf("[Cloud] HTTP %s %s → code %d (%s), free heap %u B, payload %u B\n",
                  msg.method, msg.path, code, code < 0 ? "connection/TLS error" : "server rejected",
                  (unsigned)ESP.getFreeHeap(), (unsigned)strlen(msg.payload));
    fbClient.stop();
    if (attempt == 0) delay(200);
  }
  return false;
}

// ==================== SINK STATISTICS ====================

// Record `count` unique readings with sequence numbers in [seq_start, seq_end]
void stats_record(uint8_t oid, uint16_t seq_start, uint16_t seq_end, uint32_t count,
                  int32_t delay_ms, int rssi, const char* path) {
  if (oid < 1 || oid > NUM_NODES) return;
  portENTER_CRITICAL(&sink_mux);
  origin_stats_t& os = origin_stats[oid];
  if (!os.ever_seen) {
    os.ever_seen = true;
    os.first_seq = seq_start;
    os.highest_seq = seq_end;
  } else {
    if (seq_start < os.first_seq) os.first_seq = seq_start;
    if (seq_start <= os.highest_seq) os.out_of_order++;
    if (seq_end > os.highest_seq) os.highest_seq = seq_end;
  }
  os.unique_rx += count;
  if (delay_ms >= 0) {
    os.total_delay_ms += (uint32_t)delay_ms;
    os.delay_samples++;
    os.last_delay_ms = (uint32_t)delay_ms;
  }
  os.last_rx_ms = millis();
  if (rssi != 0) os.last_rssi = rssi;
  strncpy(os.last_path, path, sizeof(os.last_path) - 1);
  os.last_path[sizeof(os.last_path) - 1] = '\0';
  portEXIT_CRITICAL(&sink_mux);
  last_activity_ms[oid] = millis();
}

void note_delivered_bytes(const wsn_packet_t& pkt) {
  portENTER_CRITICAL(&sink_mux);
  win_wire_bytes += wsn_wire_len(pkt);
  win_payload_bytes += pkt.payload_len;
  portEXIT_CRITICAL(&sink_mux);
}

// End-to-end delay from an origin timestamp (sink clock is the TDMA reference)
int32_t e2e_delay(uint32_t origin_ts) {
  int32_t d = (int32_t)(tdma_synced_time() - origin_ts);
  return (d < 0) ? 0 : d;
}

// "N1→N3→N4" from the packet's hop trace
void build_route(const wsn_packet_t& pkt, char* out, size_t n) {
  out[0] = '\0';
  uint8_t last = (pkt.hop_count < 4) ? pkt.hop_count : 3;
  for (uint8_t i = 0; i <= last; i++) {
    if (pkt.route_path[i] == 0) break;
    char tmp[8];
    snprintf(tmp, sizeof(tmp), "%sN%d", (i > 0 ? "→" : ""), pkt.route_path[i]);
    strncat(out, tmp, n - strlen(out) - 1);
  }
  strncat(out, "→N4", n - strlen(out) - 1);
}

void push_node_data(uint8_t oid, const char* type, uint16_t seq, float v1, float v2, float v3,
                    uint8_t hops, const char* route, int rssi, int32_t delay, bool aggregated) {
  static uint32_t lastPush[NUM_NODES + 1] = {0};
  if (millis() - lastPush[oid] < DATA_PUSH_MIN_MS) return;
  lastPush[oid] = millis();
  char path[64], payload[384];
  snprintf(path, sizeof(path), "/nodes/N%d/data.json", oid);
  snprintf(payload, sizeof(payload),
    "{\"node_id\":\"N%d\",\"sensor_type\":\"%s\",\"seq_num\":%u,"
    "\"val1\":%.2f,\"val2\":%.2f,\"val3\":%.2f,"
    "\"hops\":%u,\"route\":\"%s\",\"rssi\":%d,\"aggregated\":%s,"
    "\"e2e_delay_ms\":%ld,\"timestamp\":%lu}",
    oid, type, seq, v1, v2, v3, hops, route, rssi, aggregated ? "true" : "false",
    (long)delay, (unsigned long)millis());
  queueFirebase("POST", path, payload);
}

// ==================== DATA PROCESSING AT SINK ====================

void processSensorData(const wsn_packet_t& pkt, int rssi) {
  sensor_payload_t sp;
  getPayload(pkt, sp);
  uint8_t oid = pkt.origin_id;
  if (oid < 1 || oid > NUM_NODES) return;

  int32_t delay = e2e_delay(pkt.timestamp_ms);
  char route[32];
  build_route(pkt, route, sizeof(route));
  stats_record(oid, pkt.seq_num, pkt.seq_num, 1, delay, rssi, route);
  note_delivered_bytes(pkt);

  bool colltest = strncmp(sp.sensor_type, "colltest", sizeof(sp.sensor_type)) == 0;
  Serial.printf("[SINK] DATA from N%d: v1=%.2f v2=%.2f v3=%.2f sensor=%s seq=%d hops=%d route=%s delay=%ld ms RSSI(last hop)=%d\n",
    oid, sp.val1, sp.val2, sp.val3, sp.sensor_type, pkt.seq_num, pkt.hop_count, route, (long)delay, rssi);
  if (colltest) {
    Serial.println("[SINK] Collision-test frame delivered (counted for PDR, excluded from sensor charts)");
    return;
  }
  push_node_data(oid, sp.sensor_type, pkt.seq_num, sp.val1, sp.val2, sp.val3,
                 pkt.hop_count + 1, route, rssi, delay, false);
}

void processAggData(const wsn_packet_t& pkt, int rssi) {
  agg_payload_t ap;
  getPayload(pkt, ap);

  char route[32];
  build_route(pkt, route, sizeof(route));
  int32_t d3 = e2e_delay(pkt.timestamp_ms);
  stats_record(pkt.origin_id, pkt.seq_num, pkt.seq_num, 1, d3, rssi, route);
  note_delivered_bytes(pkt);

  // Constituent readings count for their original sources (exact counts)
  int32_t d1 = -1, d2 = -1;
  char sub[40];
  if (ap.n1_count > 0) {
    d1 = e2e_delay(ap.n1_oldest_ts);
    snprintf(sub, sizeof(sub), "N1→%s", route);
    stats_record(1, ap.n1_seq_start, ap.n1_seq_end, ap.n1_count, d1, 0, sub);
  }
  if (ap.n2_count > 0) {
    d2 = e2e_delay(ap.n2_oldest_ts);
    snprintf(sub, sizeof(sub), "N2→%s", route);
    stats_record(2, ap.n2_seq_start, ap.n2_seq_end, ap.n2_count, d2, 0, sub);
  }

  uint32_t pin = ap.n1_count + ap.n2_count;
  portENTER_CRITICAL(&sink_mux);
  agg_bytes_in_total += ap.win_bytes_in;
  agg_bytes_out_total += ap.win_bytes_out;
  agg_pkts_in_total += pin;
  agg_pkts_out_total += 1;
  float cum_red = agg_bytes_in_total ? (1.0f - (float)agg_bytes_out_total / agg_bytes_in_total) * 100.0f : 0;
  float pkt_red = agg_pkts_in_total ? (1.0f - (float)agg_pkts_out_total / agg_pkts_in_total) * 100.0f : 0;
  portEXIT_CRITICAL(&sink_mux);
  float win_red = ap.win_bytes_in ? (1.0f - (float)ap.win_bytes_out / ap.win_bytes_in) * 100.0f : 0;

  Serial.printf("[SINK] AGG from N%d seq=%d route=%s: N1[%d] mean=(%.2f,%.2f,%.2f) max=%.2f | N2[%d] flame max=%.0f min=%.0f DO=%d | N3 IR=%d\n",
    pkt.origin_id, pkt.seq_num, route, ap.n1_count, ap.n1_accel_mean_x, ap.n1_accel_mean_y, ap.n1_accel_mean_z,
    ap.n1_accel_max, ap.n2_count, ap.n2_flame_analog_max, ap.n2_flame_analog_min, ap.n2_flame_digital, ap.n3_ir_obstacle);
  Serial.printf("[SINK] AGG reduction: window %u B→%u B (%.1f%%) | cumulative bytes %.1f%%, packets %lu→%lu (%.1f%%) | delay N3=%ld ms, N1(oldest)=%ld ms, N2(oldest)=%ld ms (-1 = no reading)\n",
    ap.win_bytes_in, ap.win_bytes_out, win_red, cum_red,
    (unsigned long)agg_pkts_in_total, (unsigned long)agg_pkts_out_total, pkt_red, (long)d3, (long)d1, (long)d2);

  // Dashboard: aggregation card
  static uint32_t lastAggPush = 0;
  if (millis() - lastAggPush >= DATA_PUSH_MIN_MS) {
    lastAggPush = millis();
    char payload[FB_PAYLOAD_SIZE];
    snprintf(payload, sizeof(payload),
      "{\"n1_mean_x\":%.2f,\"n1_mean_y\":%.2f,\"n1_mean_z\":%.2f,\"n1_max\":%.2f,\"n1_count\":%u,"
      "\"n2_flame_max\":%.0f,\"n2_flame_min\":%.0f,\"n2_flame_digital\":%u,\"n2_count\":%u,\"n3_ir\":%u,"
      "\"n1_seq\":\"%u-%u\",\"n2_seq\":\"%u-%u\","
      "\"bytes_in\":%u,\"bytes_out\":%u,\"reduction_pct\":%.1f,"
      "\"cum_reduction_pct\":%.1f,\"pkt_reduction_pct\":%.1f,\"pkts_in\":%lu,\"pkts_out\":%lu,"
      "\"hops\":%u,\"route\":\"%s\",\"timestamp\":%lu}",
      ap.n1_accel_mean_x, ap.n1_accel_mean_y, ap.n1_accel_mean_z, ap.n1_accel_max, ap.n1_count,
      ap.n2_flame_analog_max, ap.n2_flame_analog_min, ap.n2_flame_digital, ap.n2_count, ap.n3_ir_obstacle,
      ap.n1_seq_start, ap.n1_seq_end, ap.n2_seq_start, ap.n2_seq_end,
      ap.win_bytes_in, ap.win_bytes_out, win_red, cum_red, pkt_red,
      (unsigned long)agg_pkts_in_total, (unsigned long)agg_pkts_out_total,
      pkt.hop_count + 1, route, (unsigned long)millis());
    queueFirebase("POST", "/agg/data.json", payload);
  }

  // Dashboard: per-node sensor cards (fed from the aggregate in clustered mode)
  if (ap.n1_count > 0) {
    snprintf(sub, sizeof(sub), "N1→%s", route);
    push_node_data(1, "vibration", ap.n1_seq_end, ap.n1_accel_mean_x, ap.n1_accel_mean_y, ap.n1_accel_mean_z,
                   pkt.hop_count + 2, sub, rssi, d1, true);
  }
  if (ap.n2_count > 0) {
    snprintf(sub, sizeof(sub), "N2→%s", route);
    push_node_data(2, "flame", ap.n2_seq_end, ap.n2_flame_analog_max, ap.n2_flame_digital, ap.n2_flame_analog_min,
                   pkt.hop_count + 2, sub, rssi, d2, true);
  }
  push_node_data(3, "ir", pkt.seq_num, ap.n3_ir_obstacle, 0, 0, pkt.hop_count + 1, route, rssi, d3, true);
}

void pushLocalization(uint8_t id, const ble_report_payload_t& rpt, bool ok, float ex, float ey, float err) {
  char path[64], payload[512];
  snprintf(path, sizeof(path), "/ble/N%d.json", id);
  snprintf(payload, sizeof(payload),
    "{\"localized\":%s,\"est_x\":%.2f,\"est_y\":%.2f,\"true_x\":%.2f,\"true_y\":%.2f,"
    "\"error_m\":%.2f,\"nearest\":\"N%d\",\"neighbors\":%d,"
    "\"a1\":\"N%d\",\"d1\":%.2f,\"rssi1\":%d,\"a2\":\"N%d\",\"d2\":%.2f,\"rssi2\":%d,"
    "\"a3\":\"N%d\",\"d3\":%.2f,\"rssi3\":%d,\"timestamp\":%lu}",
    ok ? "true" : "false", ex, ey, NODE_TRUE_POS[id][0], NODE_TRUE_POS[id][1], ok ? err : -1.0f,
    rpt.nearest_node_id, rpt.num_neighbors,
    rpt.entries[0].node_id, rpt.entries[0].distance_m, rpt.entries[0].rssi,
    rpt.entries[1].node_id, rpt.entries[1].distance_m, rpt.entries[1].rssi,
    rpt.entries[2].node_id, rpt.entries[2].distance_m, rpt.entries[2].rssi,
    (unsigned long)millis());
  queueFirebase("PUT", path, payload);
}

void localizeReport(const ble_report_payload_t& rpt) {
  uint8_t id = rpt.reporter_id;
  if (id < 1 || id > NUM_NODES) return;
  uint8_t ids[3] = {0, 0, 0};
  float dists[3] = {0, 0, 0};
  for (int i = 0; i < rpt.num_neighbors && i < 3; i++) {
    ids[i] = rpt.entries[i].node_id;
    dists[i] = rpt.entries[i].distance_m;
  }
  float ex = 0, ey = 0, err = -1, rms = 0;
  bool ok = ble_localize(id, ids, dists, rpt.num_neighbors, ex, ey, err, &rms);
  for (int i = 0; i < rpt.num_neighbors && i < 3; i++) {
    Serial.printf("[SINK]   N%d sees N%d: RSSI=%d dBm → %.2f m\n", id, ids[i], rpt.entries[i].rssi, dists[i]);
  }
  if (ok) {
    loc_error[id] = err;
    Serial.printf("[SINK] Localization N%d: est=(%.2f,%.2f) reference=(%.2f,%.2f) error=%.2f m | range residual %.2f m (%s) | nearest=N%d\n",
                  id, ex, ey, NODE_TRUE_POS[id][0], NODE_TRUE_POS[id][1], err, rms,
                  rms < 1.0f ? "good" : (rms < 3.0f ? "fair" : "poor: calibrate"), rpt.nearest_node_id);
  } else {
    Serial.printf("[SINK] Localization N%d: proximity only (%d anchors) | nearest=N%d\n",
                  id, rpt.num_neighbors, rpt.nearest_node_id);
  }
  pushLocalization(id, rpt, ok, ex, ey, err);
}

void processBleReport(const wsn_packet_t& pkt) {
  ble_report_payload_t rpt;
  getPayload(pkt, rpt);
  char route[32];
  build_route(pkt, route, sizeof(route));
  Serial.printf("[SINK] BLE Report from N%d via %s: %d neighbours\n", rpt.reporter_id, route, rpt.num_neighbors);
  last_activity_ms[pkt.origin_id] = millis();
  localizeReport(rpt);
}

// ==================== APP CALLBACKS ====================
// Runs in the Wi-Fi task: only copy the packet; processing happens in loop().
struct sink_rx_t {
  wsn_packet_t pkt;
  int rssi;
};
QueueHandle_t sinkRxQueue;

void app_on_data_for_me(const wsn_packet_t& pkt, int rssi) {
  sink_rx_t item;
  item.pkt = pkt;
  item.rssi = rssi;
  if (xQueueSend(sinkRxQueue, &item, 0) != pdTRUE) {
    Serial.println("[SINK] RX queue full — packet not processed");
  }
}

void sink_process(const wsn_packet_t& pkt, int rssi) {
  switch (pkt.msg_type) {
    case MSG_DATA:       processSensorData(pkt, rssi); break;
    case MSG_AGG:        processAggData(pkt, rssi); break;
    case MSG_BLE_REPORT: processBleReport(pkt); break;
    default:
      Serial.printf("[SINK] Received type=%s from N%d\n", msgTypeStr(pkt.msg_type), pkt.origin_id);
      break;
  }
}

void app_on_ctrl(ctrl_cmd_t cmd, uint8_t p1, uint8_t p2) {}

bool app_on_data_transit(const wsn_packet_t& pkt, int rssi) {
  return false;
}

// ==================== METRICS → FIREBASE ====================

bool recently(uint32_t t, uint32_t window) { return t != 0 && millis() - t < window; }

// Next hop of a node: from its own HELLO while N4 hears it directly,
// otherwise from the most recent observed hop trace ("N3→N2→N4" → N2).
const char* nextHopName(uint8_t id, const char* last_path) {
  route_neighbor_t& nb = mesh.neighbors[id];
  if (nb.link_alive && recently(nb.last_hello_ms, HELLO_TIMEOUT_MS) && nb.next_hop != 0) {
    return nodeIdStr(nb.next_hop);
  }
  const char* q = strstr(last_path, "→");   // UTF-8 arrow is 3 bytes
  if (q && q[3] == 'N' && q[4] >= '1' && q[4] <= '4') return nodeIdStr(q[4] - '0');
  return "?";
}

void pushPerformanceMetrics() {
  uint32_t now = millis();
  uint32_t elapsed = now - last_metrics_ms;
  last_metrics_ms = now;

  origin_stats_t snap[NUM_NODES + 1];
  uint32_t wire, payload_b;
  portENTER_CRITICAL(&sink_mux);
  memcpy(snap, origin_stats, sizeof(snap));
  wire = win_wire_bytes; payload_b = win_payload_bytes;
  win_wire_bytes = 0; win_payload_bytes = 0;
  float cum_red = agg_bytes_in_total ? (1.0f - (float)agg_bytes_out_total / agg_bytes_in_total) * 100.0f : 0;
  float pkt_red = agg_pkts_in_total ? (1.0f - (float)agg_pkts_out_total / agg_pkts_in_total) * 100.0f : 0;
  portEXIT_CRITICAL(&sink_mux);

  // ---- Part 1: per-node QoS + network summary ----
  char body[FB_PAYLOAD_SIZE];
  int len = snprintf(body, sizeof(body), "{");
  uint32_t tot_exp = 0, tot_rx = 0, tot_delay = 0, tot_samples = 0;
  for (int i = 1; i <= 3; i++) {
    origin_stats_t& os = snap[i];
    if (!os.ever_seen) continue;
    uint32_t expected = (uint32_t)(os.highest_seq - os.first_seq) + 1;
    uint32_t rx = (os.unique_rx > expected) ? expected : os.unique_rx;
    uint32_t lost = expected - rx;
    float pdr = expected ? rx * 100.0f / expected : 0;
    float loss = expected ? lost * 100.0f / expected : 0;
    float avg_delay = os.delay_samples ? (float)os.total_delay_ms / os.delay_samples : 0;
    tot_exp += expected; tot_rx += rx; tot_delay += os.total_delay_ms; tot_samples += os.delay_samples;
    len += snprintf(body + len, sizeof(body) - len,
      "%s\"nodes/N%d/qos\":{\"expected\":%lu,\"received\":%lu,\"lost\":%lu,\"pdr\":%.1f,\"loss_pct\":%.1f,"
      "\"duplicates\":%lu,\"out_of_order\":%lu,\"avg_delay_ms\":%.1f,\"last_delay_ms\":%lu,\"rssi_dbm\":%d,"
      "\"retries\":%u,\"path\":\"%s\"}",
      (len > 1 ? "," : ""), i, (unsigned long)expected, (unsigned long)rx, (unsigned long)lost, pdr, loss,
      (unsigned long)mesh.dup_rx_by_origin[i], (unsigned long)os.out_of_order, avg_delay,
      (unsigned long)os.last_delay_ms, mesh.neighbors[i].link_alive ? mesh.neighbors[i].wifi_rssi : os.last_rssi,
      mesh.neighbors[i].reported_retries, os.last_path);
  }
  uint32_t net_retx = mesh.mac_stats.total_retries;
  for (int i = 1; i <= 3; i++) net_retx += mesh.neighbors[i].reported_retries;
  float net_pdr = tot_exp ? tot_rx * 100.0f / tot_exp : 0;
  float net_delay = tot_samples ? (float)tot_delay / tot_samples : 0;
  float kbps = elapsed ? (wire * 8.0f) / elapsed : 0;            // bits/ms == kbit/s
  float goodput = elapsed ? (payload_b * 8.0f) / elapsed : 0;
  float loc_sum = 0; int loc_n = 0;
  for (int i = 1; i <= NUM_NODES; i++) if (loc_error[i] >= 0) { loc_sum += loc_error[i]; loc_n++; }
  len += snprintf(body + len, sizeof(body) - len,
    "%s\"perf/summary\":{\"pdr\":%.1f,\"loss_pct\":%.1f,\"expected\":%lu,\"received\":%lu,"
    "\"avg_delay_ms\":%.1f,\"throughput_kbps\":%.3f,\"goodput_kbps\":%.3f,\"retransmissions\":%lu,"
    "\"data_reduction_pct\":%.1f,\"packet_reduction_pct\":%.1f,\"avg_loc_error_m\":%.2f,\"timestamp\":%lu}}",
    (len > 1 ? "," : ""), net_pdr, tot_exp ? 100.0f - net_pdr : 0, (unsigned long)tot_exp, (unsigned long)tot_rx,
    net_delay, kbps, goodput, (unsigned long)net_retx, cum_red, pkt_red,
    loc_n ? loc_sum / loc_n : -1.0f, (unsigned long)now);
  queueFirebase("PATCH", "/.json", body);

  Serial.printf("[PERF] PDR=%.1f%% (%lu/%lu) loss=%.1f%% avg_delay=%.1f ms throughput=%.3f kbps goodput=%.3f kbps retransmissions=%lu reduction=%.1f%% bytes/%.1f%% pkts avg_loc_err=%.2f m\n",
    net_pdr, (unsigned long)tot_rx, (unsigned long)tot_exp, tot_exp ? 100.0f - net_pdr : 0, net_delay, kbps, goodput,
    (unsigned long)net_retx, cum_red, pkt_red, loc_n ? loc_sum / loc_n : -1.0f);

  // ---- Part 2: MAC activity, duty cycle, topology ----
  uint32_t net_cca = mesh.mac_stats.cca_busy_count, net_bo = mesh.mac_stats.total_backoffs;
  uint32_t net_to = mesh.mac_stats.total_acks_timeout, net_drop = mesh.mac_stats.total_drops;
  for (int i = 1; i <= 3; i++) {
    net_cca += mesh.neighbors[i].reported_cca_busy;
    net_bo += mesh.neighbors[i].reported_backoffs;
    net_to += mesh.neighbors[i].reported_ack_timeouts;
    net_drop += mesh.neighbors[i].reported_drops;
  }
  uint32_t total = mesh.tdma.active_time_ms + mesh.tdma.sleep_time_ms;
  float duty4 = total ? mesh.tdma.active_time_ms * 100.0f / total : 100.0f;
  float duty_sum = 0; int duty_n = 0;
  for (int i = 1; i <= 3; i++) if (recently(mesh.neighbors[i].last_hello_ms, HELLO_TIMEOUT_MS)) { duty_sum += mesh.neighbors[i].reported_duty_pct; duty_n++; }

  bool n3_link = mesh.neighbors[3].link_alive;
  len = snprintf(body, sizeof(body),
    "{\"perf/throughput\":{\"kbps\":%.3f,\"goodput_kbps\":%.3f,\"window_ms\":%lu,\"timestamp\":%lu},"
    "\"perf/mac\":{\"cca_busy\":%lu,\"backoffs\":%lu,\"retries\":%lu,\"network_retries\":%lu,"
    "\"network_cca_busy\":%lu,\"network_backoffs\":%lu,\"network_ack_timeouts\":%lu,\"network_drops\":%lu,"
    "\"drops\":%lu,\"acks_ok\":%lu,\"acks_timeout\":%lu,\"dup_rx\":%lu},",
    kbps, goodput, (unsigned long)elapsed, (unsigned long)now,
    (unsigned long)mesh.mac_stats.cca_busy_count, (unsigned long)mesh.mac_stats.total_backoffs,
    (unsigned long)mesh.mac_stats.total_retries, (unsigned long)net_retx,
    (unsigned long)net_cca, (unsigned long)net_bo, (unsigned long)net_to, (unsigned long)net_drop,
    (unsigned long)mesh.mac_stats.total_drops, (unsigned long)mesh.mac_stats.total_acks_ok,
    (unsigned long)mesh.mac_stats.total_acks_timeout, (unsigned long)mesh.mac_stats.dup_rx);
  len += snprintf(body + len, sizeof(body) - len, "\"perf/mac_nodes\":{");
  for (int i = 1; i <= 3; i++) {
    route_neighbor_t& nb = mesh.neighbors[i];
    len += snprintf(body + len, sizeof(body) - len,
      "%s\"N%d\":{\"retries\":%u,\"cca_busy\":%u,\"backoffs\":%u,\"ack_timeouts\":%u,\"drops\":%u,\"duty_pct\":%u}",
      i > 1 ? "," : "", i, nb.reported_retries, nb.reported_cca_busy, nb.reported_backoffs,
      nb.reported_ack_timeouts, nb.reported_drops, nb.reported_duty_pct);
  }
  len += snprintf(body + len, sizeof(body) - len,
    "},\"perf/duty_cycle\":{\"duty_cycle_pct\":%.1f,\"sink_duty_pct\":%.1f,\"active_ms\":%lu,\"sleep_ms\":%lu,\"timestamp\":%lu},",
    duty_n ? duty_sum / duty_n : duty4, duty4, (unsigned long)mesh.tdma.active_time_ms,
    (unsigned long)mesh.tdma.sleep_time_ms, (unsigned long)now);
  len += snprintf(body + len, sizeof(body) - len,
    "\"route/topology\":{\"N1_next\":\"%s\",\"N2_next\":\"%s\",\"N3_next\":\"%s\","
    "\"N1_alive\":%s,\"N2_alive\":%s,\"N3_alive\":%s,\"N3_N4_link\":%s,"
    "\"N1_path\":\"%s\",\"N2_path\":\"%s\",\"N3_path\":\"%s\",\"scheduler\":\"N%d\",\"timestamp\":%lu}}",
    nextHopName(1, snap[1].last_path), nextHopName(2, snap[2].last_path), nextHopName(3, snap[3].last_path),
    node_active[1] ? "true" : "false", node_active[2] ? "true" : "false", node_active[3] ? "true" : "false",
    n3_link ? "true" : "false",
    snap[1].last_path, snap[2].last_path, snap[3].last_path,
    mesh.tdma.sched_issuer ? mesh.tdma.sched_issuer : 3, (unsigned long)now);
  queueFirebase("PATCH", "/.json", body);
}

// Push the current TDMA schedule whenever it changes
void pushScheduleIfChanged() {
  static uint16_t lastVer = 0xFFFF;
  static uint8_t lastIssuer = 0xFF;
  if (!mesh.tdma.schedule_received) return;
  if (mesh.tdma.sched_version == lastVer && mesh.tdma.sched_issuer == lastIssuer) return;
  lastVer = mesh.tdma.sched_version;
  lastIssuer = mesh.tdma.sched_issuer;
  char payload[256];
  char s[4][8];
  for (int e = EPOCH_SLOT1; e <= EPOCH_SLOT4; e++) {
    uint8_t o = mesh.tdma.slot_owner[e];
    if (o) snprintf(s[e - EPOCH_SLOT1], 8, "N%d", o); else snprintf(s[e - EPOCH_SLOT1], 8, "UNUSED");
  }
  snprintf(payload, sizeof(payload),
    "{\"version\":%u,\"issuer\":\"N%d\",\"reason\":\"%s\",\"S1\":\"%s\",\"S2\":\"%s\",\"S3\":\"%s\",\"S4\":\"%s\","
    "\"superframe_ms\":%d,\"timestamp\":%lu}",
    mesh.tdma.sched_version, mesh.tdma.sched_issuer, mesh.tdma.sched_reason, s[0], s[1], s[2], s[3],
    SUPERFRAME_MS, (unsigned long)millis());
  queueFirebase("PUT", "/tdma/schedule.json", payload);
}

// Node status for the dashboard (active / inactive) based on what N4 hears
void nodeWatchdog() {
  for (int i = 1; i <= 3; i++) {
    uint32_t t = max(last_activity_ms[i], mesh.last_heard_ms[i]);
    bool active = recently(t, NODE_ACTIVE_TIMEOUT_MS);
    if (active != node_active[i]) {
      node_active[i] = active;
      char path[64];
      snprintf(path, sizeof(path), "/nodes/N%d/status.json", i);
      queueFirebase("PUT", path, active ? "\"active\"" : "\"inactive\"");
      Serial.printf("[N4] Node N%d is now %s\n", i, active ? "ACTIVE" : "INACTIVE");
    }
  }
}

// N4 takes over TDMA scheduling if the cluster head (N3) is silent
void checkSchedulerFailover() {
  uint32_t n3_last = max(last_activity_ms[3], mesh.last_heard_ms[3]);
  bool n3_silent = (n3_last == 0) ? (millis() > 30000) : (millis() - n3_last > N3_SILENT_MS);
  bool primary_stale = mesh.tdma.last_primary_sched_ms == 0 ||
                       millis() - mesh.tdma.last_primary_sched_ms > SCHED_PRIMARY_TIMEOUT_MS;

  if (n3_silent && primary_stale) {
    bool n1 = mesh.neighbors[1].link_alive || recently(last_activity_ms[1], NODE_ACTIVE_TIMEOUT_MS);
    bool n2 = mesh.neighbors[2].link_alive || recently(last_activity_ms[2], NODE_ACTIVE_TIMEOUT_MS);
    if (!failoverActive || n1 != failN1 || n2 != failN2) {
      if (!failoverActive) {
        Serial.printf("[N4] *** N3 silent for %lu ms — N4 takes over TDMA scheduling ***\n",
                      (unsigned long)(n3_last ? millis() - n3_last : millis()));
      }
      failoverActive = true;
      failN1 = n1; failN2 = n2;
      schedule_payload_t sp;
      tdma_build_schedule(sp, n1, n2, false, false, "FAILOVER: N3 offline", (uint16_t)(0x8000 + (++failoverCount)), SINK_ID);
      tdma_publish_schedule(sp);
    }
  } else if (failoverActive && !n3_silent) {
    failoverActive = false;
    mesh.sched_is_coordinator = false;
    Serial.println("[N4] N3 is back — schedule control returned to the cluster head");
  }
}

// ==================== CONTROL COMMANDS (serial / dashboard) ====================

void broadcastCtrl(uint8_t cmd, uint8_t p1, uint8_t p2) {
  wsn_packet_t pkt;
  initPacket(pkt, MSG_CTRL, BROADCAST_ID, SINK_ID, BROADCAST_ID, mesh.ctrl_seq_counter++);
  ctrl_payload_t cp = { cmd, p1, p2 };
  setPayload(pkt, cp);
  for (int i = 0; i < 3; i++) {     // repeated for reliability; receivers de-duplicate by seq
    mac_send_broadcast(pkt, i == 0);
    delay(15);
  }
}

void checkDashboardControls() {
  String cmd = "";
  int p1 = 0, p2 = 0;

  if (remote_cmd.pending) {
    cmd = String(remote_cmd.cmd);
    p1 = remote_cmd.p1;
    p2 = remote_cmd.p2;
    remote_cmd.pending = false;
    Serial.printf("[CTRL-REMOTE] Executing dashboard command: %s (p1=%d p2=%d)\n", cmd.c_str(), p1, p2);
  } else if (wsn_read_serial_line(cmd)) {
    if (cmd.startsWith("kill ") && cmd.length() >= 8) {
      p1 = cmd.charAt(5) - '0';
      p2 = cmd.charAt(7) - '0';
      cmd = "kill";
    }
  }
  if (cmd.length() == 0) return;

  if (cmd == "collision") {
    Serial.println("[CTRL] Broadcasting COLLISION_TEST (N1+N2 contend at next CAP)");
    broadcastCtrl(CTRL_COLLISION_TEST, 0, 0);
  } else if (cmd == "kill") {
    if (p1 < 1 || p1 > NUM_NODES || p2 < 1 || p2 > NUM_NODES || p1 == p2) {
      Serial.println("[CTRL] Usage: kill <a> <b>   e.g. kill 3 4");
      return;
    }
    Serial.printf("[CTRL] Broadcasting KILL_LINK N%d↔N%d (software failure injection)\n", p1, p2);
    broadcastCtrl(CTRL_KILL_LINK, (uint8_t)p1, (uint8_t)p2);
    if (p1 == SINK_ID || p2 == SINK_ID) {
      uint8_t other = (p1 == SINK_ID) ? p2 : p1;
      routing_mark_link_dead(other, "software link kill");
      mesh.killed_links[other] = true;
    }
  } else if (cmd == "restore") {
    Serial.println("[CTRL] Broadcasting RESTORE_LINKS");
    broadcastCtrl(CTRL_RESTORE_LINKS, 0, 0);
    for (int i = 1; i <= NUM_NODES; i++) mesh.killed_links[i] = false;
    mesh.hello_triggered = true;
  } else if (cmd == "status") {
    mesh_print_status();
    ble_print_status();
    pushPerformanceMetrics();
  } else if (!ble_command(cmd)) {
    Serial.printf("[CTRL] Unknown command '%s'. Commands: collision | kill A B | restore | status | ble | cal <id> <m> | setn <n>\n", cmd.c_str());
  }
}

// Extract "cmd" string value from a small JSON object
bool jsonCmd(const String& resp, char* out, size_t n) {
  int k = resp.indexOf("\"cmd\"");
  if (k < 0) return false;
  int q1 = resp.indexOf("\"", k + 5);
  if (q1 < 0) return false;
  int q2 = resp.indexOf("\"", q1 + 1);
  if (q2 < 0) return false;
  String v = resp.substring(q1 + 1, q2);
  strncpy(out, v.c_str(), n - 1);
  out[n - 1] = '\0';
  return true;
}

// ==================== FreeRTOS TASKS ====================

void cloudTask(void* param) {
  Serial.println("[Cloud] Task started on Core 0");
  fbClient.setInsecure();
  fbClient.setTimeout(2);

  static firebase_msg_t init;
  strcpy(init.method, "PATCH");
  strcpy(init.path, "/.json");
  strcpy(init.payload, "{\"nodes/N1/status\":\"inactive\",\"nodes/N2/status\":\"inactive\",\"nodes/N3/status\":\"inactive\"}");
  executeFirebase(init);

  static firebase_msg_t msg;
  while (true) {
    if (xQueueReceive(firebaseQueue, &msg, pdMS_TO_TICKS(100)) == pdTRUE) {
      if (!executeFirebase(msg)) {
        Serial.printf("[Cloud] Firebase %s %s FAILED\n", msg.method, msg.path);
      }
    }

    // Poll /control.json for dashboard commands
    if (millis() - lastControlPoll > 3000 && wifiConnected && !remote_cmd.pending) {
      lastControlPoll = millis();
      String url = String(FIREBASE_HOST) + "/control.json";
      fbHttp.begin(fbClient, url);
      fbHttp.setReuse(true);
      fbHttp.setTimeout(1200);
      int code = fbHttp.GET();
      if (code == 200) {
        String resp = fbHttp.getString();
        char c[16];
        if (resp != "null" && jsonCmd(resp, c, sizeof(c)) && strcmp(c, "idle") != 0) {
          remote_cmd.p1 = 0; remote_cmd.p2 = 0;
          int i1 = resp.indexOf("\"p1\":");
          if (i1 != -1) remote_cmd.p1 = resp.substring(i1 + 5).toInt();
          int i2 = resp.indexOf("\"p2\":");
          if (i2 != -1) remote_cmd.p2 = resp.substring(i2 + 5).toInt();
          strncpy(remote_cmd.cmd, c, sizeof(remote_cmd.cmd) - 1);
          remote_cmd.cmd[sizeof(remote_cmd.cmd) - 1] = '\0';
          remote_cmd.pending = true;
          fbHttp.end();
          fbHttp.begin(fbClient, url);
          fbHttp.addHeader("Content-Type", "application/json");
          fbHttp.PUT("{\"cmd\":\"idle\"}");   // acknowledge so the command runs once
        }
      }
      fbHttp.end();
    }

    // Wi-Fi watchdog
    if (millis() - lastWifiCheck > 10000) {
      lastWifiCheck = millis();
      if (WiFi.status() != WL_CONNECTED && wifiConnected) {
        Serial.println("[WiFi] Connection LOST");
        wifiConnected = false;
        fbClient.stop();
      } else if (WiFi.status() != WL_CONNECTED) {
        connectWiFi();
      } else if (!wifiConnected) {
        wifiConnected = true;
        fbClient.stop();
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ==================== SETUP ====================
void setup() {
  Serial.setTxBufferSize(4096);
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n===== NODE 4 (Sink / Gateway) BOOTING =====");

  memset(origin_stats, 0, sizeof(origin_stats));
  firebaseQueue = xQueueCreate(FIREBASE_QUEUE_SIZE, sizeof(firebase_msg_t));
  sinkRxQueue = xQueueCreate(16, sizeof(sink_rx_t));

  // 1. BLE + boot-time discovery
  ble_init();
  ble_boot_discovery();

  // 2. Wi-Fi FIRST (the AP decides the channel the whole mesh will use)
  connectWiFi();

  // NTP time only orders dashboard history keys; the mesh does not depend on it
  configTime(0, 0, "pool.ntp.org", "time.google.com");

  // 3. ESP-NOW mesh on the same channel
  fbClient.setInsecure();
  fbClient.setTimeout(2);
  mesh_init();

  // 4. N4 is the time reference and routing root
  mesh.tdma.synced = true;
  mesh.tdma.synced_from_beacon = true;
  mesh.tdma.clock_offset = 0;
  mesh.neighbors[SINK_ID].hops_to_sink = 0;
  mesh.neighbors[SINK_ID].link_alive = true;
  mesh.next_hop_to_sink = SINK_ID;
  last_metrics_ms = millis();

  // 5. Cloud uploads on core 0
  xTaskCreatePinnedToCore(cloudTask, "cloudTask", 12288, NULL, 1, NULL, 0);

  Serial.println("===== N4 SETUP COMPLETE =====\n");
  Serial.println("Serial commands: 'collision', 'kill X Y', 'restore', 'status'");
}

// ==================== MAIN LOOP ====================
void loop() {
  uint32_t now = millis();

  // 1. Mesh periodic tasks (beacon in BEACON epoch, HELLO in CAP, ...)
  mesh_loop();

  // 1b. Process packets delivered to the sink
  sink_rx_t rx;
  while (xQueueReceive(sinkRxQueue, &rx, 0) == pdTRUE) {
    sink_process(rx.pkt, rx.rssi);
  }

  // 2. BLE scan burst in SLEEP epoch + N4 self-localization every 10 s
  ble_tick();
  if (now - lastBleReport >= BLE_REPORT_PERIOD_MS) {
    lastBleReport = now;
    ble_report_payload_t rpt = ble_build_report();
    if (rpt.num_neighbors > 0) {
      Serial.println("[SINK] BLE self-report (N4):");
      localizeReport(rpt);
    }
  }

  // 3. Metrics
  if (now - lastPerfPush >= METRICS_PERIOD_MS) {
    lastPerfPush = now;
    pushPerformanceMetrics();
  }

  // 4. Node status + scheduler failover + schedule display
  if (now - lastWatchdog >= 1000) {
    lastWatchdog = now;
    nodeWatchdog();
    checkSchedulerFailover();
  }
  pushScheduleIfChanged();

  // 5. Commands
  checkDashboardControls();

  // 6. One batched Firebase upload every 2 s
  static uint32_t lastFbFlush = 0;
  if (now - lastFbFlush >= FB_FLUSH_PERIOD_MS) {
    lastFbFlush = now;
    fb_flush();
  }

  // 6. Status
  if (now - lastStatusPrint >= 15000) {
    lastStatusPrint = now;
    mesh_print_status();
    Serial.printf("--- Gateway: Wi-Fi %s, free heap %u B, cloud batches skipped %lu ---\n",
                  wifiConnected ? "connected" : "DISCONNECTED", (unsigned)ESP.getFreeHeap(), (unsigned long)fb_dropped);
    Serial.println("--- Sink Stats (unique packets per origin) ---");
    for (int i = 1; i <= 3; i++) {
      origin_stats_t& os = origin_stats[i];
      if (!os.ever_seen) continue;
      uint32_t expected = (uint32_t)(os.highest_seq - os.first_seq) + 1;
      uint32_t rx = (os.unique_rx > expected) ? expected : os.unique_rx;
      float pdr = expected ? rx * 100.0f / expected : 0;
      float avg_delay = os.delay_samples ? (float)os.total_delay_ms / os.delay_samples : 0;
      Serial.printf("  N%d: rx=%lu exp=%lu PDR=%.1f%% lost=%lu dups=%lu ooo=%lu avg_delay=%.0f ms last_path=%s\n",
        i, (unsigned long)rx, (unsigned long)expected, pdr, (unsigned long)(expected - rx),
        (unsigned long)mesh.dup_rx_by_origin[i], (unsigned long)os.out_of_order, avg_delay, os.last_path);
    }
  }
}
