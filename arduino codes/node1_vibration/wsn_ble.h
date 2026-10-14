// ==================== WSN BLE LOCALIZATION LAYER ====================
// NimBLE-based BLE advertising, scanning, RSSI distance estimation,
// nearest-node identification, trilateration and localization error.
// Runs on ALL 4 nodes.
//
// Include after wsn_common.h and wsn_mesh.h.
// Requires the NimBLE-Arduino library v2.x (h2zero) — installed separately
// from the ESP32 Arduino core (Library Manager: "NimBLE-Arduino").
// ====================================================================
#ifndef WSN_BLE_H
#define WSN_BLE_H

#include "wsn_common.h"
#include <NimBLEDevice.h>
#include <Preferences.h>

// ==================== BLE CONFIGURATION ====================
// Manufacturer ID (0xFFFF = reserved for testing), used to identify WSN adverts
#define WSN_BLE_MFG_ID  0xFFFF

// BLE advertising payload: [company ID (2)] [node_id (1)] [tx_power_1m (1, int8)]

// REFERENCE (ground-truth) positions of each node in metres, 2D.
// *** Tape-measure your lab layout and enter the real positions here. ***
// Every node uses this table: the other three nodes are the anchors when a
// node localizes itself, and the node's own entry is the reference used for
// the localization error. Keep the 4 points non-collinear.
const float NODE_TRUE_POS[NUM_NODES + 1][2] = {
  {0.0f, 0.0f},   // Index 0: unused
  {0.0f, 0.0f},   // N1: (0, 0)     — ADJUST to actual
  {2.0f, 0.0f},   // N2: (2, 0)     — ADJUST to actual
  {1.0f, 1.5f},   // N3: (1, 1.5)   — ADJUST to actual
  {1.0f, -1.0f},  // N4: (1, -1.0)  — ADJUST to actual
};

// ==================== BLE STATE ====================
static NimBLEAdvertising* pAdvertising = nullptr;
static NimBLEScan*         pScan = nullptr;
static bool ble_initialized = false;

// ==================== RSSI → DISTANCE CALIBRATION ====================
// Receiver-side calibration, stored in flash (NVS) so it survives reboots.
// ble_cal_A = RSSI at 1 m (0 = use the value advertised by the sender),
// ble_cal_n = path-loss exponent. Set from the Serial Monitor (see ble_command).
static float ble_cal_A = 0;
static float ble_cal_n = BLE_PATH_LOSS_N;

void ble_load_calibration() {
  Preferences p;
  if (p.begin("wsnble", true)) {
    ble_cal_A = p.getFloat("A", 0);
    ble_cal_n = p.getFloat("n", BLE_PATH_LOSS_N);
    p.end();
  }
}

void ble_save_calibration() {
  Preferences p;
  if (p.begin("wsnble", false)) {
    p.putFloat("A", ble_cal_A);
    p.putFloat("n", ble_cal_n);
    p.end();
  }
}

float ble_rssi_to_distance(float rssi_f, int8_t advertised_A) {
  float A = (ble_cal_A != 0) ? ble_cal_A : (float)advertised_A;
  float d = powf(10.0f, (A - rssi_f) / (10.0f * ble_cal_n));
  if (d < BLE_MIN_DIST_M) d = BLE_MIN_DIST_M;
  if (d > BLE_MAX_DIST_M) d = BLE_MAX_DIST_M;
  return d;
}

// ==================== SCAN CALLBACK ====================
// Duplicates are enabled, so every advertisement heard during a burst
// updates the RSSI filter (several samples per neighbour per burst).
static volatile uint8_t ble_seen_mask = 0;     // neighbours heard in the current burst
static volatile uint8_t ble_samples[NUM_NODES + 1];
static uint8_t  ble_known_mask = 0;            // neighbours known after the last burst
static uint16_t ble_scan_count = 0;

class WsnScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* device) override {
    if (!device->haveManufacturerData()) return;

    // NimBLE returns manufacturer data as std::string: [companyID_lo, companyID_hi, data...]
    std::string raw = device->getManufacturerData();
    if (raw.length() < 4) return;

    uint16_t companyId = (uint8_t)raw[0] | ((uint8_t)raw[1] << 8);
    if (companyId != WSN_BLE_MFG_ID) return;

    uint8_t node_id = (uint8_t)raw[2];
    int8_t  tx_power = (int8_t)raw[3];
    if (node_id < 1 || node_id > NUM_NODES || node_id == MY_NODE_ID) return;

    int rssi = device->getRSSI();
    if (rssi >= 0 || rssi < -110) return;   // invalid reading

    ble_neighbor_t& bn = mesh.ble[node_id];
    bool first = !bn.discovered;
    bn.node_id = node_id;
    bn.rssi_raw = rssi;
    bn.last_seen_ms = millis();
    bn.discovered = true;

    // EWMA filter on RSSI (first sample initialises the filter)
    if (first || bn.rssi_filtered == 0) {
      bn.rssi_filtered = (float)rssi;
    } else {
      bn.rssi_filtered = BLE_EWMA_ALPHA * (float)rssi +
                         (1.0f - BLE_EWMA_ALPHA) * bn.rssi_filtered;
    }

    // Log-distance path loss model: d = 10^((A - RSSI) / (10 * n)), clamped
    bn.distance_m = ble_rssi_to_distance(bn.rssi_filtered, tx_power);

    ble_seen_mask |= (1 << node_id);
    if (ble_samples[node_id] < 255) ble_samples[node_id]++;
  }

  void onScanEnd(const NimBLEScanResults& results, int reason) override {
    ble_scan_count++;
    uint8_t known = 0;
    for (int i = 1; i <= NUM_NODES; i++) if (i != MY_NODE_ID && mesh.ble[i].discovered) known |= (1 << i);
    // Log on change, and every 10th burst
    if (known != ble_known_mask || ble_scan_count % 10 == 1) {
      char line[160];
      int len = snprintf(line, sizeof(line), "[BLE] Scan #%u:", ble_scan_count);
      for (int i = 1; i <= NUM_NODES; i++) {
        if (i == MY_NODE_ID) continue;
        ble_neighbor_t& bn = mesh.ble[i];
        if (bn.discovered) {
          len += snprintf(line + len, sizeof(line) - len, " N%d %d dBm(f %.0f)→%.1fm%s", i, bn.rssi_raw,
                          bn.rssi_filtered, bn.distance_m, (ble_seen_mask & (1 << i)) ? "" : "*");
        } else {
          len += snprintf(line + len, sizeof(line) - len, " N%d --", i);
        }
        if (len >= (int)sizeof(line)) break;
      }
      Serial.println(line);
      for (int i = 1; i <= NUM_NODES; i++) {
        if (i == MY_NODE_ID) continue;
        bool now = known & (1 << i), before = ble_known_mask & (1 << i);
        if (now && !before) Serial.printf("[BLE] Discovered N%d (RSSI %d dBm → %.2f m)\n", i, mesh.ble[i].rssi_raw, mesh.ble[i].distance_m);
      }
    }
    ble_known_mask = known;
    ble_seen_mask = 0;
    for (int i = 0; i <= NUM_NODES; i++) ble_samples[i] = 0;
  }
};

static WsnScanCallbacks scanCallbacks;

// ==================== BLE INITIALIZATION ====================
void ble_init() {
  char name[16];
  snprintf(name, sizeof(name), "WSN_N%d", MY_NODE_ID);

  ble_load_calibration();
  NimBLEDevice::init(name);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  pAdvertising = NimBLEDevice::getAdvertising();

  char mfgBytes[4];
  mfgBytes[0] = (char)(WSN_BLE_MFG_ID & 0xFF);
  mfgBytes[1] = (char)((WSN_BLE_MFG_ID >> 8) & 0xFF);
  mfgBytes[2] = (char)MY_NODE_ID;
  mfgBytes[3] = (char)(int8_t)BLE_TX_POWER_1M;
  std::string mfgData(mfgBytes, 4);

  NimBLEAdvertisementData advData;
  advData.setName(name);
  advData.setManufacturerData(mfgData);

  pAdvertising->setAdvertisementData(advData);
  pAdvertising->setMinInterval(BLE_ADV_INTERVAL_MS / 0.625f);
  pAdvertising->setMaxInterval((BLE_ADV_INTERVAL_MS + 20) / 0.625f);
  pAdvertising->start();

  pScan = NimBLEDevice::getScan();
  pScan->setScanCallbacks(&scanCallbacks, true);   // true = report every advert (more RSSI samples)
  pScan->setActiveScan(false);   // passive: RSSI only, no scan requests
  pScan->setInterval(BLE_SCAN_DURATION_MS);
  pScan->setWindow(BLE_SCAN_DURATION_MS);           // listen 100% of the burst
  pScan->setMaxResults(0);       // callbacks only; don't store every BLE device in the lab

  ble_initialized = true;
  if (ble_cal_A != 0) {
    Serial.printf("[BLE] Initialized: advertising as %s | calibration from flash: A(1 m)=%.1f dBm, n=%.2f\n",
                  name, ble_cal_A, ble_cal_n);
  } else {
    Serial.printf("[BLE] Initialized: advertising as %s | default model: A(1 m)=%d dBm, n=%.2f (uncalibrated — use 'cal')\n",
                  name, BLE_TX_POWER_1M, ble_cal_n);
  }
}

// ==================== BLE SCAN (Non-blocking burst) ====================
void ble_scan(uint32_t duration_ms = BLE_SCAN_DURATION_MS) {
  if (!ble_initialized || !pScan) return;
  if (!pScan->isScanning()) {
    pScan->start(duration_ms, false);  // duration in ms, non-blocking (NimBLE 2.x)
  }
}

// ==================== NEAREST NODE ====================
uint8_t ble_nearest_node() {
  float min_dist = 999.0f;
  uint8_t nearest = 0;
  for (int i = 1; i <= NUM_NODES; i++) {
    if (i == MY_NODE_ID) continue;
    if (!mesh.ble[i].discovered) continue;
    if (mesh.ble[i].distance_m < min_dist) {
      min_dist = mesh.ble[i].distance_m;
      nearest = i;
    }
  }
  return nearest;
}

// ==================== LOCALIZATION ====================
// Weighted non-linear least squares (Gauss-Newton) on the 3 anchor ranges:
//   minimise  sum_i w_i * (|p - a_i| - d_i)^2 ,  w_i = 1 / d_i^2
// Closer anchors (more reliable RSSI) weigh more; distances are clamped and
// the estimate is kept inside the deployment area (reference positions
// ± BLE_AREA_MARGIN_M). This is far more robust to one bad RSSI reading than
// the closed-form linearised trilateration, which can return points hundreds
// of metres away. rms_m = residual (how consistent the 3 ranges are).
bool ble_trilaterate(const float anchor_pos[3][2], const float distances[3],
                     float& out_x, float& out_y, float& rms_m) {
  float d[3], w[3];
  float minX = 1e9f, maxX = -1e9f, minY = 1e9f, maxY = -1e9f;
  for (int i = 1; i <= NUM_NODES; i++) {
    minX = fminf(minX, NODE_TRUE_POS[i][0]); maxX = fmaxf(maxX, NODE_TRUE_POS[i][0]);
    minY = fminf(minY, NODE_TRUE_POS[i][1]); maxY = fmaxf(maxY, NODE_TRUE_POS[i][1]);
  }
  minX -= BLE_AREA_MARGIN_M; maxX += BLE_AREA_MARGIN_M;
  minY -= BLE_AREA_MARGIN_M; maxY += BLE_AREA_MARGIN_M;

  float wsum = 0, x = 0, y = 0;
  for (int i = 0; i < 3; i++) {
    d[i] = fminf(fmaxf(distances[i], BLE_MIN_DIST_M), BLE_MAX_DIST_M);
    w[i] = 1.0f / (d[i] * d[i]);
    wsum += w[i];
    x += w[i] * anchor_pos[i][0];
    y += w[i] * anchor_pos[i][1];
  }
  x /= wsum; y /= wsum;   // start at the weighted centroid of the anchors

  for (int it = 0; it < 30; it++) {
    float h00 = 0, h01 = 0, h11 = 0, g0 = 0, g1 = 0;
    for (int i = 0; i < 3; i++) {
      float dx = x - anchor_pos[i][0], dy = y - anchor_pos[i][1];
      float r = sqrtf(dx * dx + dy * dy);
      if (r < 1e-3f) r = 1e-3f;
      float jx = dx / r, jy = dy / r, f = r - d[i];
      h00 += w[i] * jx * jx; h01 += w[i] * jx * jy; h11 += w[i] * jy * jy;
      g0 += w[i] * jx * f;   g1 += w[i] * jy * f;
    }
    h00 += 1e-4f; h11 += 1e-4f;                 // damping
    float det = h00 * h11 - h01 * h01;
    if (fabsf(det) < 1e-9f) break;
    float sx = -( h11 * g0 - h01 * g1) / det;
    float sy = -(-h01 * g0 + h00 * g1) / det;
    x = fminf(fmaxf(x + sx, minX), maxX);
    y = fminf(fmaxf(y + sy, minY), maxY);
    if (fabsf(sx) + fabsf(sy) < 1e-3f) break;
  }

  float se = 0;
  for (int i = 0; i < 3; i++) {
    float dx = x - anchor_pos[i][0], dy = y - anchor_pos[i][1];
    float f = sqrtf(dx * dx + dy * dy) - d[i];
    se += f * f;
  }
  rms_m = sqrtf(se / 3.0f);
  out_x = x;
  out_y = y;
  return true;
}

// Localize a reporter from its (up to 3) neighbour distances.
// Returns true and fills est/err if 3 anchors are available.
bool ble_localize(uint8_t reporter, const uint8_t ids[3], const float dists[3], uint8_t n,
                  float& est_x, float& est_y, float& err_m, float* rms_out = nullptr) {
  if (n < 3) return false;
  float anchors[3][2];
  for (int i = 0; i < 3; i++) {
    anchors[i][0] = NODE_TRUE_POS[ids[i]][0];
    anchors[i][1] = NODE_TRUE_POS[ids[i]][1];
  }
  float rms = 0;
  if (!ble_trilaterate(anchors, dists, est_x, est_y, rms)) return false;
  float dx = est_x - NODE_TRUE_POS[reporter][0];
  float dy = est_y - NODE_TRUE_POS[reporter][1];
  err_m = sqrtf(dx * dx + dy * dy);
  if (rms_out) *rms_out = rms;
  return true;
}

// ==================== HOUSEKEEPING ====================
// Forget neighbours not heard recently; start a scan burst in the SLEEP epoch.
void ble_tick() {
  uint32_t now = millis();
  for (int i = 1; i <= NUM_NODES; i++) {
    if (i == MY_NODE_ID) continue;
    ble_neighbor_t& bn = mesh.ble[i];
    if (bn.discovered && now - bn.last_seen_ms > BLE_NEIGHBOR_EXPIRY_MS) {
      bn.discovered = false;
      bn.rssi_filtered = 0;
      Serial.printf("[BLE] N%d expired (not seen for %lu ms)\n", i, (unsigned long)(now - bn.last_seen_ms));
    }
  }
  // Scan only inside the SLEEP epoch, where no Wi-Fi traffic is scheduled
  if (mesh.tdma.synced && tdma_current_epoch() == EPOCH_SLEEP &&
      tdma_epoch_remaining_ms() > BLE_SCAN_DURATION_MS &&
      now - mesh.last_ble_scan_ms >= BLE_SCAN_PERIOD_MS) {
    ble_scan(BLE_SCAN_DURATION_MS);
    mesh.last_ble_scan_ms = now;
  }
}

// ==================== BUILD BLE REPORT ====================
ble_report_payload_t ble_build_report() {
  ble_report_payload_t rpt;
  memset(&rpt, 0, sizeof(rpt));
  rpt.reporter_id = MY_NODE_ID;
  rpt.nearest_node_id = ble_nearest_node();
  rpt.num_neighbors = 0;

  uint8_t ids[3];
  float dists[3];
  for (int i = 1; i <= NUM_NODES && rpt.num_neighbors < 3; i++) {
    if (i == MY_NODE_ID) continue;
    if (!mesh.ble[i].discovered) continue;
    uint8_t idx = rpt.num_neighbors;
    rpt.entries[idx].node_id = i;
    rpt.entries[idx].rssi = (int8_t)lroundf(mesh.ble[i].rssi_filtered);
    rpt.entries[idx].distance_m = mesh.ble[i].distance_m;
    ids[idx] = i;
    dists[idx] = mesh.ble[i].distance_m;
    rpt.num_neighbors++;
  }

  rpt.true_x = NODE_TRUE_POS[MY_NODE_ID][0];
  rpt.true_y = NODE_TRUE_POS[MY_NODE_ID][1];
  rpt.error_m = -1.0f;
  float ex, ey, err;
  if (ble_localize(MY_NODE_ID, ids, dists, rpt.num_neighbors, ex, ey, err)) {
    rpt.est_x = ex;
    rpt.est_y = ey;
    rpt.error_m = err;
  }
  return rpt;
}

// ==================== PRINT BLE STATUS ====================
void ble_print_status() {
  Serial.println("\n====== BLE STATUS ======");
  for (int i = 1; i <= NUM_NODES; i++) {
    if (i == MY_NODE_ID) continue;
    ble_neighbor_t& bn = mesh.ble[i];
    if (bn.discovered) {
      Serial.printf("  N%d: RSSI=%d dBm filtered=%.1f dBm dist=%.2f m (last %lu ms ago)\n",
                    i, bn.rssi_raw, bn.rssi_filtered, bn.distance_m,
                    (unsigned long)(millis() - bn.last_seen_ms));
    } else {
      Serial.printf("  N%d: not discovered\n", i);
    }
  }
  uint8_t nearest = ble_nearest_node();
  if (nearest > 0) {
    Serial.printf("  Nearest node: N%d (%.2f m) → proximity: N%d is closest to N%d\n",
                  nearest, mesh.ble[nearest].distance_m, MY_NODE_ID, nearest);
  }
  ble_report_payload_t rpt = ble_build_report();
  if (rpt.error_m >= 0) {
    uint8_t ids[3]; float d[3]; float ex, ey, err, rms = 0;
    for (int i = 0; i < 3; i++) { ids[i] = rpt.entries[i].node_id; d[i] = rpt.entries[i].distance_m; }
    ble_localize(MY_NODE_ID, ids, d, 3, ex, ey, err, &rms);
    Serial.printf("  Localization: est=(%.2f, %.2f) m  reference=(%.2f, %.2f) m  error=%.2f m  (range residual %.2f m → %s)\n",
                  rpt.est_x, rpt.est_y, rpt.true_x, rpt.true_y, rpt.error_m, rms,
                  rms < 1.0f ? "good" : (rms < 3.0f ? "fair" : "poor: calibrate BLE_TX_POWER_1M / BLE_PATH_LOSS_N"));
  } else {
    Serial.printf("  Localization: needs 3 anchors (have %d) — proximity only\n", rpt.num_neighbors);
  }
  Serial.println("========================\n");
}

// ==================== BOOT-TIME DISCOVERY ====================
// Demonstration flow step 1: BLE discovery → RSSI → localization, before Wi-Fi.
void ble_boot_discovery(uint32_t duration_ms = BLE_BOOT_DISCOVERY_MS) {
  if (!ble_initialized || !pScan) return;
  Serial.printf("[BLE] Boot discovery: scanning %lu ms before Wi-Fi start...\n", (unsigned long)duration_ms);
  pScan->start(duration_ms, false);
  uint32_t t0 = millis();
  while (pScan->isScanning() && millis() - t0 < duration_ms + 500) delay(10);
  ble_print_status();
}

// ==================== SERIAL COMMANDS ====================
// Non-blocking line reader (works with any Serial Monitor line-ending setting).
bool wsn_read_serial_line(String& out) {
  static char buf[48];
  static uint8_t len = 0;
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (len == 0) continue;
      buf[len] = '\0';
      out = String(buf);
      out.trim();
      len = 0;
      return true;
    }
    if (len < sizeof(buf) - 1) buf[len++] = c;
  }
  return false;
}

// BLE commands available on every node:
//   ble              → print BLE status (RSSI, distances, nearest, localization)
//   cal <id> <m>     → place node <id> exactly <m> metres away, then run: sets A(1 m)
//   setn <n>         → path-loss exponent (typ. 2.0 open space … 3.5 cluttered lab)
//   seta <dBm>       → set A(1 m) directly
//   blereset         → back to defaults
// Returns true if the command was a BLE command.
bool ble_command(const String& cmd) {
  if (cmd == "ble") {
    ble_print_status();
    return true;
  }
  if (cmd.startsWith("cal ")) {
    int sp = cmd.indexOf(" ", 4);
    int id = cmd.substring(4, sp > 0 ? sp : cmd.length()).toInt();
    float m = (sp > 0) ? cmd.substring(sp + 1).toFloat() : 1.0f;
    if (id < 1 || id > NUM_NODES || id == MY_NODE_ID || m <= 0.05f) {
      Serial.println("[BLE-CAL] Usage: cal <node 1-4> <metres>   e.g. 'cal 2 1.0'");
      return true;
    }
    if (!mesh.ble[id].discovered) {
      Serial.printf("[BLE-CAL] N%d not discovered yet — wait for a scan and retry\n", id);
      return true;
    }
    ble_cal_A = mesh.ble[id].rssi_filtered + 10.0f * ble_cal_n * log10f(m);
    ble_save_calibration();
    Serial.printf("[BLE-CAL] Using N%d at %.2f m (filtered RSSI %.1f dBm): A(1 m) = %.1f dBm, n = %.2f — saved\n",
                  id, m, mesh.ble[id].rssi_filtered, ble_cal_A, ble_cal_n);
    for (int i = 1; i <= NUM_NODES; i++) {
      if (mesh.ble[i].discovered) mesh.ble[i].distance_m = ble_rssi_to_distance(mesh.ble[i].rssi_filtered, BLE_TX_POWER_1M);
    }
    return true;
  }
  if (cmd.startsWith("setn ")) {
    float n = cmd.substring(5).toFloat();
    if (n < 1.5f || n > 5.0f) { Serial.println("[BLE-CAL] n must be 1.5 … 5.0"); return true; }
    ble_cal_n = n;
    ble_save_calibration();
    Serial.printf("[BLE-CAL] Path-loss exponent n = %.2f — saved\n", ble_cal_n);
    return true;
  }
  if (cmd.startsWith("seta ")) {
    float a = cmd.substring(5).toFloat();
    if (a > -20 || a < -100) { Serial.println("[BLE-CAL] A must be -100 … -20 dBm"); return true; }
    ble_cal_A = a;
    ble_save_calibration();
    Serial.printf("[BLE-CAL] A(1 m) = %.1f dBm — saved\n", ble_cal_A);
    return true;
  }
  if (cmd == "blereset") {
    ble_cal_A = 0;
    ble_cal_n = BLE_PATH_LOSS_N;
    ble_save_calibration();
    Serial.println("[BLE-CAL] Calibration reset to defaults");
    return true;
  }
  return false;
}

// For N1–N3: read Serial Monitor commands (N4 has its own command handler)
void ble_serial_poll() {
  String cmd;
  if (!wsn_read_serial_line(cmd)) return;
  if (cmd == "status") { mesh_print_status(); ble_print_status(); return; }
  if (!ble_command(cmd)) {
    Serial.printf("[CMD] Unknown '%s'. Commands: status | ble | cal <id> <m> | setn <n> | seta <dBm> | blereset\n", cmd.c_str());
  }
}

#endif // WSN_BLE_H
