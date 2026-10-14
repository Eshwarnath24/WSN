// ==================== NODE 4: SINK / GATEWAY ====================
// Full mesh participant: BLE + CSMA/CA MAC + TDMA + Routing + Forwarding
// Additional roles:
//   - SINK: final destination for all sensor data
//   - TIME BEACON MASTER: broadcasts superframe sync beacons
//   - CLOUD GATEWAY: uploads data to Firebase RTDB via Wi-Fi
//   - LOCALIZATION ENGINE: runs BLE trilateration for all nodes
//   - DASHBOARD BACKEND: pushes all metrics to Firebase for web dashboard
//
// Architecture: FreeRTOS dual-core
//   Core 1 (high priority): meshTask — ESP-NOW, MAC, routing, TDMA, BLE
//   Core 0 (low priority):  cloudTask — Firebase HTTP uploads (non-blocking)
//
// Primary route: N4 IS the sink (hops_to_sink = 0)
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

#if __has_include("wsn_config.h")
#include "wsn_config.h"
#endif

// ==================== WIFI / FIREBASE CONFIG ====================
#ifndef WSN_WIFI_SSID
#define WSN_WIFI_SSID     "iQOO Neo6"
#define WSN_WIFI_PASS     "eshwar24"
#define WSN_FIREBASE_HOST "https://wsn-project-bb012-default-rtdb.asia-southeast1.firebasedatabase.app"
#endif

const char* WIFI_SSID     = WSN_WIFI_SSID;
const char* WIFI_PASS     = WSN_WIFI_PASS;
const char* FIREBASE_HOST = WSN_FIREBASE_HOST;

#define WIFI_TIMEOUT_ATTEMPTS 20
#define FIREBASE_QUEUE_SIZE   32

// ==================== FIREBASE CLIENT ====================
WiFiClientSecure fbClient;
HTTPClient fbHttp;
bool wifiConnected = false;

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
// JSON payloads are queued from meshTask (Core 1) and consumed by cloudTask (Core 0)
struct firebase_msg_t {
  char method[5];    // "POST" or "PUT"
  char path[64];     // Firebase path
  char payload[512]; // JSON payload
};
QueueHandle_t firebaseQueue;

// ==================== PERFORMANCE TRACKING ====================
// Per-origin tracking at the sink
struct origin_stats_t {
  uint16_t first_seq;
  uint16_t highest_seq;
  uint32_t received_count;
  uint32_t duplicate_count;
  uint32_t out_of_order_count;
  uint32_t total_delay_ms;
  uint32_t last_rx_ms;
  bool     ever_seen;
};
origin_stats_t origin_stats[NUM_NODES + 1];

// Network-wide MAC stats and throughput (Defect 7 fix)
uint32_t total_retransmissions_network = 0;
uint32_t rx_bytes_window = 0;
uint32_t last_throughput_calc = 0;
float    current_throughput_kbps = 0.0f;

// ==================== TIMERS ====================
uint32_t lastStatusPrint = 0;
uint32_t lastBleReport = 0;
uint32_t lastBeaconSend = 0;
uint32_t lastPerfPush = 0;
uint32_t lastQosPush = 0;
uint32_t lastWifiCheck = 0;

// Track N3 liveness for fallback scheduling
uint32_t lastN3Seen = 0;
bool n3WasOnline = false;

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
    Serial.printf("\n[WiFi] CONNECTED | IP=%s | Channel=%d\n",
                  WiFi.localIP().toString().c_str(), WiFi.channel());
  } else {
    wifiConnected = false;
    Serial.printf("\n[WiFi] FAILED (status=%d)\n", WiFi.status());
  }
}

// Queue a Firebase request (called from Core 1, non-blocking)
void queueFirebase(const char* method, const char* path, const char* payload) {
  firebase_msg_t msg;
  strncpy(msg.method, method, sizeof(msg.method) - 1);
  msg.method[sizeof(msg.method) - 1] = '\0';
  strncpy(msg.path, path, sizeof(msg.path) - 1);
  msg.path[sizeof(msg.path) - 1] = '\0';
  strncpy(msg.payload, payload, sizeof(msg.payload) - 1);
  msg.payload[sizeof(msg.payload) - 1] = '\0';

  if (xQueueSend(firebaseQueue, &msg, 0) != pdTRUE) {
    Serial.println("[Firebase] Queue FULL — dropping message");
  }
}

// Execute a Firebase HTTP request (called from Core 0)
bool executeFirebase(const firebase_msg_t& msg) {
  if (!wifiConnected || WiFi.status() != WL_CONNECTED) return false;

  String url = String(FIREBASE_HOST) + msg.path;

  for (int attempt = 0; attempt < 2; attempt++) {
    fbHttp.begin(fbClient, url);
    fbHttp.setReuse(true);
    fbHttp.setTimeout(1500);
    fbHttp.addHeader("Content-Type", "application/json");

    int code = 0;
    if (strcmp(msg.method, "POST") == 0) {
      code = fbHttp.POST(msg.payload);
    } else {
      code = fbHttp.PUT(msg.payload);
    }

    if (code > 0) {
      fbHttp.end();
      return true;
    }
    fbHttp.end();
    fbClient.stop();
    if (attempt == 0) delay(20);
  }
  return false;
}

// ==================== DATA PROCESSING AT SINK ====================

void processSensorData(const wsn_packet_t& pkt, int rssi) {
  sensor_payload_t sp;
  getPayload(pkt, sp);

  uint8_t oid = pkt.origin_id;
  if (oid < 1 || oid > NUM_NODES) return;

  origin_stats_t& os = origin_stats[oid];

  // Sequence analysis
  bool isDup = false, isOOO = false;
  if (!os.ever_seen) {
    os.first_seq = pkt.seq_num;
    os.highest_seq = pkt.seq_num;
    os.received_count = 1;
    os.ever_seen = true;
  } else if (pkt.seq_num == os.highest_seq) {
    isDup = true;
    os.duplicate_count++;
  } else if (pkt.seq_num < os.highest_seq) {
    isOOO = true;
    os.out_of_order_count++;
    os.received_count++;
  } else {
    os.highest_seq = pkt.seq_num;
    os.received_count++;
  }

  // End-to-end delay (accurate calculation using synchronized TDMA clock - Defect 5 fix)
  uint32_t now_synced = tdma_synced_time();
  uint32_t e2e_delay = 1;
  if (now_synced >= pkt.timestamp_ms) {
    e2e_delay = now_synced - pkt.timestamp_ms;
  }
  if (e2e_delay < 1) e2e_delay = 1;
  if (e2e_delay > 5000) e2e_delay = 5000;
  os.total_delay_ms += e2e_delay;
  os.last_rx_ms = millis();

  // Track network throughput bytes
  static uint32_t totalBytesReceived = 0;
  static uint32_t sinkStartTime = 0;
  if (sinkStartTime == 0) sinkStartTime = millis();
  totalBytesReceived += sizeof(wsn_packet_t);

  // Build route path string
  char routeStr[32] = "";
  for (int i = 0; i < 4 && pkt.route_path[i] != 0; i++) {
    char tmp[8];
    snprintf(tmp, sizeof(tmp), "%sN%d", (i > 0 ? "→" : ""), pkt.route_path[i]);
    strncat(routeStr, tmp, sizeof(routeStr) - strlen(routeStr) - 1);
  }

  Serial.printf("[SINK] DATA from N%d: v1=%.2f v2=%.2f v3=%.2f sensor=%s seq=%d hops=%d route=%s delay=%lu ms %s%s\n",
    oid, sp.val1, sp.val2, sp.val3, sp.sensor_type,
    pkt.seq_num, pkt.hop_count, routeStr, e2e_delay,
    isDup ? "[DUP]" : "", isOOO ? "[OOO]" : "");

  // Filter collision-test packets from polluting live sensor telemetry charts
  if (strcmp(sp.sensor_type, "colltest") == 0) {
    Serial.println("[SINK] Collision-test frame logged (isolated from sensor telemetry)");
    return;
  }

  // Queue for Firebase (throttled to avoid TLS backlog)
  static uint32_t lastDataPush[NUM_NODES + 1] = {0};
  if (millis() - lastDataPush[oid] >= 500) {
    lastDataPush[oid] = millis();
    char path[64], payload[384];
    snprintf(path, sizeof(path), "/nodes/N%d/data.json", oid);
    snprintf(payload, sizeof(payload),
      "{\"node_id\":\"N%d\",\"sensor_type\":\"%s\",\"seq_num\":%d,"
      "\"val1\":%.2f,\"val2\":%.2f,\"val3\":%.2f,"
      "\"hops\":%d,\"route\":\"%s\",\"rssi\":%d,"
      "\"e2e_delay_ms\":%lu,\"is_dup\":%s,\"is_ooo\":%s,"
      "\"timestamp\":%lu}",
      oid, sp.sensor_type, pkt.seq_num,
      sp.val1, sp.val2, sp.val3,
      pkt.hop_count, routeStr, rssi,
      e2e_delay, isDup ? "true" : "false", isOOO ? "true" : "false",
      millis());
    queueFirebase("POST", path, payload);
  }

  // Update node status only on first encounter
  static bool nodeActiveReported[NUM_NODES + 1] = {false};
  if (!nodeActiveReported[oid]) {
    nodeActiveReported[oid] = true;
    char path[64];
    snprintf(path, sizeof(path), "/nodes/N%d/status.json", oid);
    queueFirebase("PUT", path, "\"active\"");
  }

  // Track N3 liveness
  if (oid == 3 || pkt.src_id == 3) {
    lastN3Seen = millis();
    n3WasOnline = true;
  }
}

void processAggData(const wsn_packet_t& pkt, int rssi) {
  agg_payload_t ap;
  getPayload(pkt, ap);

  // Track N3 AGG stats in Sink stats table (Defect 6 fix)
  origin_stats_t& os3 = origin_stats[3];
  if (!os3.ever_seen) {
    os3.first_seq = pkt.seq_num;
    os3.highest_seq = pkt.seq_num;
    os3.received_count = 1;
    os3.ever_seen = true;
  } else {
    os3.highest_seq = pkt.seq_num;
    os3.received_count++;
  }
  os3.last_rx_ms = millis();

  // Track N1 constituent packets received via AGG (Defect 6 fix)
  if (ap.n1_seq_start > 0) {
    origin_stats_t& os1 = origin_stats[1];
    uint32_t n1_pkts = (ap.n1_seq_end >= ap.n1_seq_start) ? (ap.n1_seq_end - ap.n1_seq_start + 1) : 1;
    if (!os1.ever_seen) {
      os1.first_seq = ap.n1_seq_start;
      os1.highest_seq = ap.n1_seq_end;
      os1.received_count = n1_pkts;
      os1.ever_seen = true;
    } else {
      if (ap.n1_seq_end > os1.highest_seq) os1.highest_seq = ap.n1_seq_end;
      os1.received_count += n1_pkts;
    }
    os1.last_rx_ms = millis();
  }

  // Track N2 constituent packets received via AGG (Defect 6 fix)
  if (ap.n2_seq_start > 0) {
    origin_stats_t& os2 = origin_stats[2];
    uint32_t n2_pkts = (ap.n2_seq_end >= ap.n2_seq_start) ? (ap.n2_seq_end - ap.n2_seq_start + 1) : 1;
    if (!os2.ever_seen) {
      os2.first_seq = ap.n2_seq_start;
      os2.highest_seq = ap.n2_seq_end;
      os2.received_count = n2_pkts;
      os2.ever_seen = true;
    } else {
      if (ap.n2_seq_end > os2.highest_seq) os2.highest_seq = ap.n2_seq_end;
      os2.received_count += n2_pkts;
    }
    os2.last_rx_ms = millis();
  }

  Serial.printf("[SINK] AGG from N%d: N1_mean=(%.2f,%.2f,%.2f) N1_max=%.2f "
                "N2_flame=%.0f/%d N3_IR=%d reduction=%.1f%%\n",
    pkt.origin_id, ap.n1_accel_mean_x, ap.n1_accel_mean_y, ap.n1_accel_mean_z,
    ap.n1_accel_max, ap.n2_flame_analog_max, ap.n2_flame_digital,
    ap.n3_ir_obstacle, ap.reduction_pct);

  // Throttled AGG queueing to Firebase
  static uint32_t lastAggPush = 0;
  if (millis() - lastAggPush >= 800) {
    lastAggPush = millis();
    char path[64], payload[384];
    snprintf(path, sizeof(path), "/agg/data.json");
    snprintf(payload, sizeof(payload),
      "{\"n1_mean_x\":%.2f,\"n1_mean_y\":%.2f,\"n1_mean_z\":%.2f,\"n1_max\":%.2f,"
      "\"n2_flame_max\":%.0f,\"n2_flame_digital\":%d,\"n3_ir\":%d,"
      "\"n1_seq\":\"%d-%d\",\"n2_seq\":\"%d-%d\","
      "\"bytes_in\":%d,\"bytes_out\":%d,\"reduction_pct\":%.1f,"
      "\"hops\":%d,\"timestamp\":%lu}",
      ap.n1_accel_mean_x, ap.n1_accel_mean_y, ap.n1_accel_mean_z, ap.n1_accel_max,
      ap.n2_flame_analog_max, ap.n2_flame_digital, ap.n3_ir_obstacle,
      ap.n1_seq_start, ap.n1_seq_end, ap.n2_seq_start, ap.n2_seq_end,
      ap.bytes_in, ap.bytes_out, ap.reduction_pct,
      pkt.hop_count, millis());
    queueFirebase("POST", path, payload);
  }
}

void processBleReport(const wsn_packet_t& pkt) {
  ble_report_payload_t rpt;
  getPayload(pkt, rpt);

  Serial.printf("[SINK] BLE Report from N%d: %d neighbors\n",
                rpt.reporter_id, rpt.num_neighbors);

  // Attempt trilateration if we have 3 distances
  if (rpt.num_neighbors >= 3) {
    float anchors[3][2];
    float dists[3];
    for (int i = 0; i < 3; i++) {
      uint8_t aid = rpt.entries[i].node_id;
      anchors[i][0] = NODE_TRUE_POS[aid][0];
      anchors[i][1] = NODE_TRUE_POS[aid][1];
      dists[i] = rpt.entries[i].distance_m;
    }

    float est_x, est_y;
    if (ble_trilaterate(anchors, dists, est_x, est_y)) {
      float true_x = NODE_TRUE_POS[rpt.reporter_id][0];
      float true_y = NODE_TRUE_POS[rpt.reporter_id][1];
      float error = sqrtf((est_x - true_x) * (est_x - true_x) +
                          (est_y - true_y) * (est_y - true_y));

      Serial.printf("[SINK] Trilateration N%d: est=(%.2f,%.2f) true=(%.2f,%.2f) error=%.2f m\n",
                    rpt.reporter_id, est_x, est_y, true_x, true_y, error);

      char path[64], payload[256];
      snprintf(path, sizeof(path), "/ble/N%d.json", rpt.reporter_id);
      snprintf(payload, sizeof(payload),
        "{\"est_x\":%.2f,\"est_y\":%.2f,\"true_x\":%.2f,\"true_y\":%.2f,"
        "\"error_m\":%.2f,\"nearest\":\"N%d\","
        "\"d1\":%.2f,\"d2\":%.2f,\"d3\":%.2f,"
        "\"rssi1\":%d,\"rssi2\":%d,\"rssi3\":%d,\"timestamp\":%lu}",
        est_x, est_y, true_x, true_y, error,
        rpt.nearest_node_id,
        dists[0], dists[1], dists[2],
        rpt.entries[0].rssi, rpt.entries[1].rssi, rpt.entries[2].rssi,
        millis());
      queueFirebase("PUT", path, payload);
    }
  }
}

// ==================== APP CALLBACKS ====================
// This is the main data sink — all data arriving at N4 comes through here.
void app_on_data_for_me(const wsn_packet_t& pkt, int rssi) {
  rx_bytes_window += sizeof(wsn_packet_t);
  switch (pkt.msg_type) {
    case MSG_DATA:
      processSensorData(pkt, rssi);
      break;
    case MSG_AGG:
      processAggData(pkt, rssi);
      break;
    case MSG_BLE_REPORT:
      processBleReport(pkt);
      break;
    default:
      Serial.printf("[SINK] Received type=%s from N%d\n",
                    msgTypeStr(pkt.msg_type), pkt.origin_id);
      break;
  }
}

void app_on_ctrl(ctrl_cmd_t cmd, uint8_t p1, uint8_t p2) {
  // N4 originates CTRL commands; it doesn't need to react to them
}

// Called when data transits through us (intermediate hop). Return true to consume.
bool app_on_data_transit(const wsn_packet_t& pkt, int rssi) {
  return false;  // N4 is the sink — forward normally (rare case)
}

// ==================== PUSH QoS & PERFORMANCE TO FIREBASE ====================

void pushPerformanceMetrics() {
  // Per-node QoS
  for (int i = 1; i <= 3; i++) {
    origin_stats_t& os = origin_stats[i];
    if (!os.ever_seen) continue;

    uint32_t expected = os.highest_seq - os.first_seq + 1;
    uint32_t lost = (expected > os.received_count) ? (expected - os.received_count) : 0;
    float pdr = (expected > 0) ? (os.received_count * 100.0f / expected) : 100.0f;
    float avg_delay = (os.received_count > 0) ? ((float)os.total_delay_ms / os.received_count) : 0;

    char path[64], payload[256];
    snprintf(path, sizeof(path), "/nodes/N%d/qos.json", i);
    snprintf(payload, sizeof(payload),
      "{\"expected\":%lu,\"received\":%lu,\"lost\":%lu,\"pdr\":%.1f,"
      "\"duplicates\":%lu,\"out_of_order\":%lu,\"avg_delay_ms\":%.1f}",
      expected, os.received_count, lost, pdr,
      os.duplicate_count, os.out_of_order_count, avg_delay);
    queueFirebase("PUT", path, payload);
  }

  // Throughput Calculation (Defect 7 fix)
  uint32_t now = millis();
  uint32_t elapsed_tp = now - last_throughput_calc;
  if (elapsed_tp > 0) {
    current_throughput_kbps = (rx_bytes_window * 8.0f) / elapsed_tp;
  }
  rx_bytes_window = 0;
  last_throughput_calc = now;

  char path[64], payload[256];
  snprintf(path, sizeof(path), "/perf/throughput.json");
  snprintf(payload, sizeof(payload),
    "{\"kbps\":%.2f,\"timestamp\":%lu}",
    current_throughput_kbps, now);
  queueFirebase("PUT", path, payload);

  // Network-wide MAC stats & Retransmissions (Defect 7 fix)
  total_retransmissions_network = mesh.mac_stats.total_retries +
                                  mesh.neighbors[1].reported_retries +
                                  mesh.neighbors[2].reported_retries +
                                  mesh.neighbors[3].reported_retries;

  snprintf(path, sizeof(path), "/perf/mac.json");
  snprintf(payload, sizeof(payload),
    "{\"cca_busy\":%lu,\"backoffs\":%lu,\"retries\":%lu,\"network_retries\":%lu,"
    "\"drops\":%lu,\"acks_ok\":%lu,\"acks_timeout\":%lu}",
    mesh.mac_stats.cca_busy_count, mesh.mac_stats.total_backoffs,
    mesh.mac_stats.total_retries, total_retransmissions_network,
    mesh.mac_stats.total_drops,
    mesh.mac_stats.total_acks_ok, mesh.mac_stats.total_acks_timeout);
  queueFirebase("PUT", path, payload);

  // Duty Cycle calculation (Defect 5 fix)
  uint32_t total_time = mesh.tdma.active_time_ms + mesh.tdma.sleep_time_ms;
  float duty_pct = (total_time > 0) ? (mesh.tdma.active_time_ms * 100.0f / total_time) : 100.0f;
  snprintf(path, sizeof(path), "/perf/duty_cycle.json");
  snprintf(payload, sizeof(payload),
    "{\"duty_cycle_pct\":%.1f,\"active_ms\":%lu,\"sleep_ms\":%lu,\"timestamp\":%lu}",
    duty_pct, mesh.tdma.active_time_ms, mesh.tdma.sleep_time_ms, now);
  queueFirebase("PUT", path, payload);

  // Route topology info
  snprintf(path, sizeof(path), "/route/topology.json");
  snprintf(payload, sizeof(payload),
    "{\"N1_next\":\"%s\",\"N2_next\":\"%s\",\"N3_next\":\"%s\","
    "\"N1_alive\":%s,\"N2_alive\":%s,\"N3_alive\":%s}",
    mesh.neighbors[3].link_alive ? "N3" : (mesh.neighbors[1].link_alive ? "N4" : "NONE"),
    mesh.neighbors[3].link_alive ? "N3" : (mesh.neighbors[2].link_alive ? "N4" : "NONE"),
    mesh.neighbors[3].link_alive ? "N4" : "DEAD",
    mesh.neighbors[1].link_alive ? "true" : "false",
    mesh.neighbors[2].link_alive ? "true" : "false",
    mesh.neighbors[3].link_alive ? "true" : "false");
  queueFirebase("PUT", path, payload);
}

// ==================== PUSH CTRL COMMANDS FROM DASHBOARD ====================

void checkDashboardControls() {
  String cmd = "";
  int p1 = 0, p2 = 0;

  if (remote_cmd.pending) {
    cmd = String(remote_cmd.cmd);
    p1 = remote_cmd.p1;
    p2 = remote_cmd.p2;
    remote_cmd.pending = false;
    Serial.printf("[CTRL-REMOTE] Executing dashboard command: %s (p1=%d p2=%d)\n",
                  cmd.c_str(), p1, p2);
  } else if (Serial.available()) {
    cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.startsWith("kill ")) {
      p1 = cmd.charAt(5) - '0';
      p2 = cmd.charAt(7) - '0';
      cmd = "kill";
    }
  }

  if (cmd.length() == 0) return;

  if (cmd == "collision") {
    Serial.println("[CTRL] Broadcasting COLLISION_TEST");
    wsn_packet_t pkt;
    initPacket(pkt, MSG_CTRL, BROADCAST_ID, 4, BROADCAST_ID, mesh.ctrl_seq_counter++);
    ctrl_payload_t cp = { CTRL_COLLISION_TEST, 0, 0 };
    setPayload(pkt, cp);
    espnow_raw_send(BROADCAST_ID, pkt);
  }
  else if (cmd == "kill") {
    Serial.printf("[CTRL] Broadcasting KILL_LINK N%d↔N%d\n", p1, p2);
    wsn_packet_t pkt;
    initPacket(pkt, MSG_CTRL, BROADCAST_ID, 4, BROADCAST_ID, mesh.ctrl_seq_counter++);
    ctrl_payload_t cp = { CTRL_KILL_LINK, (uint8_t)p1, (uint8_t)p2 };
    setPayload(pkt, cp);
    espnow_raw_send(BROADCAST_ID, pkt);
    // Also apply locally
    if (p1 == 4) { mesh.killed_links[p2] = true; mesh.neighbors[p2].link_alive = false; }
    if (p2 == 4) { mesh.killed_links[p1] = true; mesh.neighbors[p1].link_alive = false; }
    routing_recompute_next_hop();
  }
  else if (cmd == "restore") {
    Serial.println("[CTRL] Broadcasting RESTORE_LINKS");
    wsn_packet_t pkt;
    initPacket(pkt, MSG_CTRL, BROADCAST_ID, 4, BROADCAST_ID, mesh.ctrl_seq_counter++);
    ctrl_payload_t cp = { CTRL_RESTORE_LINKS, 0, 0 };
    setPayload(pkt, cp);
    espnow_raw_send(BROADCAST_ID, pkt);
    for (int i = 1; i <= NUM_NODES; i++) mesh.killed_links[i] = false;
    routing_recompute_next_hop();
  }
  else if (cmd == "status") {
    mesh_print_status();
    ble_print_status();
  }
}

// ==================== FreeRTOS TASKS ====================

// Cloud task: runs on Core 0, pops Firebase messages from queue
void cloudTask(void* param) {
  Serial.println("[Cloud] Task started on Core 0");

  fbClient.setInsecure();
  fbClient.setTimeout(2);

  // Set all nodes inactive initially
  for (int i = 1; i <= 3; i++) {
    char path[64];
    snprintf(path, sizeof(path), "/nodes/N%d/status.json", i);
    firebase_msg_t msg;
    strcpy(msg.method, "PUT");
    strncpy(msg.path, path, sizeof(msg.path));
    strcpy(msg.payload, "\"inactive\"");
    executeFirebase(msg);
  }

  // Main cloud loop
  firebase_msg_t msg;
  while (true) {
    // 1. Process queued Firebase messages
    if (xQueueReceive(firebaseQueue, &msg, pdMS_TO_TICKS(100)) == pdTRUE) {
      if (!executeFirebase(msg)) {
        Serial.printf("[Cloud] Firebase %s %s FAILED\n", msg.method, msg.path);
      }
    }

    // 2. Poll Firebase /control.json for remote dashboard commands every 2.5s
    if (millis() - lastControlPoll > 2500 && wifiConnected && !remote_cmd.pending) {
      lastControlPoll = millis();
      String url = String(FIREBASE_HOST) + "/control.json";
      fbHttp.begin(fbClient, url);
      fbHttp.setReuse(true);
      fbHttp.setTimeout(1200);
      int code = fbHttp.GET();
      if (code == 200) {
        String resp = fbHttp.getString();
        if (resp.indexOf("\"cmd\"") != -1 && resp.indexOf("\"idle\"") == -1 && resp != "null") {
          if (resp.indexOf("collision") != -1) {
            strncpy(remote_cmd.cmd, "collision", sizeof(remote_cmd.cmd) - 1);
            remote_cmd.pending = true;
          } else if (resp.indexOf("kill") != -1) {
            strncpy(remote_cmd.cmd, "kill", sizeof(remote_cmd.cmd) - 1);
            remote_cmd.p1 = 3; remote_cmd.p2 = 4;
            int p1_idx = resp.indexOf("\"p1\":");
            if (p1_idx != -1) remote_cmd.p1 = resp.substring(p1_idx + 5).toInt();
            int p2_idx = resp.indexOf("\"p2\":");
            if (p2_idx != -1) remote_cmd.p2 = resp.substring(p2_idx + 5).toInt();
            remote_cmd.pending = true;
          } else if (resp.indexOf("restore") != -1) {
            strncpy(remote_cmd.cmd, "restore", sizeof(remote_cmd.cmd) - 1);
            remote_cmd.pending = true;
          }

          if (remote_cmd.pending) {
            fbHttp.end();
            fbHttp.begin(fbClient, url);
            fbHttp.PUT("{\"cmd\":\"idle\"}");
          }
        }
      }
      fbHttp.end();
    }

    // 3. WiFi watchdog
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
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n===== NODE 4 (Sink / Gateway) BOOTING =====");

  // Initialize origin stats
  memset(origin_stats, 0, sizeof(origin_stats));

  // Create Firebase queue
  firebaseQueue = xQueueCreate(FIREBASE_QUEUE_SIZE, sizeof(firebase_msg_t));

  // 1. Initialize BLE
  ble_init();

  // 2. Connect WiFi FIRST (so we know the channel)
  connectWiFi();

  // 3. Initialize mesh stack (ESP-NOW on same channel as WiFi)
  fbClient.setInsecure();
  fbClient.setTimeout(2);
  mesh_init();

  // 4. Start cloud task on Core 0
  xTaskCreatePinnedToCore(cloudTask, "cloudTask", 8192, NULL, 1, NULL, 0);

  // 5. Self-sync TDMA (N4 is the time reference)
  mesh.tdma.synced = true;
  mesh.tdma.clock_offset = 0;
  mesh.neighbors[4].hops_to_sink = 0;
  mesh.neighbors[4].link_alive = true;
  mesh.next_hop_to_sink = 4;  // We ARE the sink

  Serial.println("===== N4 SETUP COMPLETE =====\n");
  Serial.println("Serial commands: 'collision', 'kill X Y', 'restore', 'status'");
}

// ==================== MAIN LOOP (runs on Core 1) ====================
void loop() {
  uint32_t now = millis();

  // 1. Mesh periodic tasks (including beacon broadcast handled inside mesh_loop)
  mesh_loop();

  // 2. BLE scan during sleep epoch
  tdma_epoch_t epoch = tdma_current_epoch();
  if (epoch == EPOCH_SLEEP && now - mesh.last_ble_scan_ms >= 3000) {
    ble_scan();
    mesh.last_ble_scan_ms = now;
  }

  // 4. N4's own BLE report (self-localization using other nodes as anchors)
  if (now - lastBleReport >= 10000) {
    lastBleReport = now;
    // N4 processes its own BLE data locally (no need to send to itself)
    ble_report_payload_t rpt = ble_build_report();
    if (rpt.num_neighbors >= 3) {
      float anchors[3][2], dists[3];
      for (int i = 0; i < 3; i++) {
        uint8_t aid = rpt.entries[i].node_id;
        anchors[i][0] = NODE_TRUE_POS[aid][0];
        anchors[i][1] = NODE_TRUE_POS[aid][1];
        dists[i] = rpt.entries[i].distance_m;
      }
      float ex, ey;
      if (ble_trilaterate(anchors, dists, ex, ey)) {
        float error = sqrtf((ex - NODE_TRUE_POS[4][0]) * (ex - NODE_TRUE_POS[4][0]) +
                            (ey - NODE_TRUE_POS[4][1]) * (ey - NODE_TRUE_POS[4][1]));
        Serial.printf("[BLE] Self-localization: est=(%.2f,%.2f) error=%.2f m\n", ex, ey, error);
      }
    }
    ble_print_status();
  }

  // 5. Push performance metrics periodically
  if (now - lastPerfPush >= 5000) {
    lastPerfPush = now;
    pushPerformanceMetrics();
  }

  // 6. Check if N3 went offline — take over scheduling
  if (n3WasOnline && !mesh.neighbors[3].link_alive && now - lastN3Seen > 10000) {
    Serial.println("[N4] N3 offline — taking over TDMA scheduling");
    tdma_update_schedule_if_needed(false, false, true);
    n3WasOnline = false;
  }

  // 7. Serial command interface for demo control
  checkDashboardControls();

  // 8. Print status periodically
  if (now - lastStatusPrint >= 15000) {
    lastStatusPrint = now;
    mesh_print_status();

    Serial.println("--- Sink Stats ---");
    for (int i = 1; i <= 3; i++) {
      origin_stats_t& os = origin_stats[i];
      if (!os.ever_seen) continue;
      uint32_t expected = os.highest_seq - os.first_seq + 1;
      float pdr = (expected > 0) ? (os.received_count * 100.0f / expected) : 100.0f;
      float avg_delay = (os.received_count > 0) ? ((float)os.total_delay_ms / os.received_count) : 0;
      Serial.printf("  N%d: rx=%lu exp=%lu PDR=%.1f%% dups=%lu ooo=%lu avg_delay=%.0f ms\n",
        i, os.received_count, expected, pdr, os.duplicate_count, os.out_of_order_count, avg_delay);
    }
  }
}
