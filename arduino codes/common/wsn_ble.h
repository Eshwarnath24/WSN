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

// ==================== SCAN CALLBACK ====================
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

    // Log-distance path loss model: d = 10^((A - RSSI) / (10 * n))
    float A = (float)tx_power;  // Calibrated RSSI at 1 m (advertised by the sender)
    float exponent = (A - bn.rssi_filtered) / (10.0f * BLE_PATH_LOSS_N);
    bn.distance_m = powf(10.0f, exponent);

    Serial.printf("[BLE] %s N%d: RSSI=%d dBm filtered=%.1f dBm → est. distance=%.2f m\n",
                  first ? "Discovered" : "Seen", node_id, rssi, bn.rssi_filtered, bn.distance_m);
  }

  void onScanEnd(const NimBLEScanResults& results, int reason) override {
    int n = 0;
    for (int i = 1; i <= NUM_NODES; i++) if (i != MY_NODE_ID && mesh.ble[i].discovered) n++;
    Serial.printf("[BLE] Scan complete: %d WSN neighbour(s) known\n", n);
  }
};

static WsnScanCallbacks scanCallbacks;

// ==================== BLE INITIALIZATION ====================
void ble_init() {
  char name[16];
  snprintf(name, sizeof(name), "WSN_N%d", MY_NODE_ID);

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
  pAdvertising->setMaxInterval((BLE_ADV_INTERVAL_MS * 2) / 0.625f);
  pAdvertising->start();

  pScan = NimBLEDevice::getScan();
  pScan->setScanCallbacks(&scanCallbacks, false);
  pScan->setActiveScan(false);   // passive: RSSI only, no scan requests
  pScan->setInterval(100);
  pScan->setWindow(80);
  pScan->setMaxResults(0);       // callbacks only; don't store every BLE device in the lab

  ble_initialized = true;
  Serial.printf("[BLE] Initialized: advertising as %s (TX power at 1m = %d dBm, n = %.1f)\n",
                name, BLE_TX_POWER_1M, BLE_PATH_LOSS_N);
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

// ==================== TRILATERATION ====================
// Given distances from a target node to 3 anchors, estimate 2D position
// (linearised: subtract anchor-0 circle equation from the other two).
bool ble_trilaterate(const float anchor_pos[3][2], const float distances[3],
                     float& out_x, float& out_y) {
  float x1 = anchor_pos[0][0], y1 = anchor_pos[0][1], d1 = distances[0];
  float x2 = anchor_pos[1][0], y2 = anchor_pos[1][1], d2 = distances[1];
  float x3 = anchor_pos[2][0], y3 = anchor_pos[2][1], d3 = distances[2];

  float A00 = 2 * (x2 - x1), A01 = 2 * (y2 - y1);
  float A10 = 2 * (x3 - x1), A11 = 2 * (y3 - y1);
  float b0 = d1*d1 - d2*d2 + x2*x2 - x1*x1 + y2*y2 - y1*y1;
  float b1 = d1*d1 - d3*d3 + x3*x3 - x1*x1 + y3*y3 - y1*y1;

  float det = A00 * A11 - A01 * A10;
  if (fabsf(det) < 1e-6f) {
    Serial.println("[BLE] Trilateration: singular matrix (anchors collinear?)");
    return false;
  }
  out_x = (A11 * b0 - A01 * b1) / det;
  out_y = (A00 * b1 - A10 * b0) / det;
  return true;
}

// Localize a reporter from its (up to 3) neighbour distances.
// Returns true and fills est/err if 3 anchors are available.
bool ble_localize(uint8_t reporter, const uint8_t ids[3], const float dists[3], uint8_t n,
                  float& est_x, float& est_y, float& err_m) {
  if (n < 3) return false;
  float anchors[3][2];
  for (int i = 0; i < 3; i++) {
    anchors[i][0] = NODE_TRUE_POS[ids[i]][0];
    anchors[i][1] = NODE_TRUE_POS[ids[i]][1];
  }
  if (!ble_trilaterate(anchors, dists, est_x, est_y)) return false;
  float dx = est_x - NODE_TRUE_POS[reporter][0];
  float dy = est_y - NODE_TRUE_POS[reporter][1];
  err_m = sqrtf(dx * dx + dy * dy);
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
    Serial.printf("  Localization: est=(%.2f, %.2f) m  reference=(%.2f, %.2f) m  error=%.2f m\n",
                  rpt.est_x, rpt.est_y, rpt.true_x, rpt.true_y, rpt.error_m);
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

#endif // WSN_BLE_H
